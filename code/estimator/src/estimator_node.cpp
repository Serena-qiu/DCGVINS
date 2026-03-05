#include <stdio.h>
#include <queue>
#include <map>
#include <cmath>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <signal.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <gvins/LocalSensorExternalTrigger.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/package.h>
#include <boost/bind.hpp>
#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"
#include "real_time_dd_processor.hpp"
#include "inter_agent_communication.hpp"
#include "utility/timing_statistics.h"

using namespace gnss_comm;

double max_gnss_camera_delay = DD_TIME_SYNC_THRESHOLD;

std::unique_ptr<Estimator> estimator_ptr;

std::condition_variable con;
double current_time = -1;
queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<std::vector<ObsPtr>> gnss_meas_buf;
queue<sensor_msgs::PointCloudConstPtr> relo_buf;
int sum_of_wait = 0;

std::mutex m_buf;
std::mutex m_state;
std::mutex i_buf;
std::mutex m_estimator;

double latest_time;
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;
Eigen::Vector3d gyr_0;
bool init_feature = 0;
bool init_imu = 1;
double last_imu_t = -1;
bool rt_dd_initialized = false;

std::mutex m_time;
double next_pulse_time;
bool next_pulse_time_valid;
double time_diff_gnss_local;
bool time_diff_valid;
double latest_gnss_time;
int skip_parameter;

// ===== Feature 降采样相关 =====
int downsample_N = 3;              // 降采样因子：3->10Hz, 6->5Hz（30Hz输入时）
int keep_phase = -1;               // 保留相位，0 ~ N-1，-1 表示尚未确定
uint64_t feature_msg_counter = 0;  // 保持 uint64_t 避免溢出
double tmp_last_feature_time = -1.0;

uint64_t debug_feature_received_count = 0;
uint64_t debug_feature_pushed_count = 0;

std::unique_ptr<InterAgentCommunication> inter_agent_comm;
std::unique_ptr<RealTimeDDProcessor> rt_dd_processor;
FaultToleranceManager* fault_tolerance_mgr = nullptr;
std::mutex estimator_access_mutex;  // 保护estimator_ptr的访问



void gnss_meas_callback_realtime(const GnssMeasMsgConstPtr &meas_msg);
void secondary_gnss_callback(const GnssMeasMsgConstPtr &meas_msg);
void gnss_ephem_callback_with_agent(const GnssEphemMsgConstPtr &ephem_msg, int agent_id);
void gnss_glo_ephem_callback_with_agent(const GnssGloEphemMsgConstPtr &glo_ephem_msg, int agent_id);

void timingShutdownCallback(int sig)
{
    ROS_INFO("=== DCGVINS Timing Statistics ===");
    
    // 获取实验配置
    bool dd_enabled = true;
    
    // 打印表格到终端
    TimingStatistics::getInstance().printTable("DCGVINS", dd_enabled);
    
    // 导出文件
    std::string output_dir = "/tmp/dcgvins_timing/";
    
    // 创建输出目录
    int ret = system(("mkdir -p " + output_dir).c_str());
    (void)ret;  // 避免警告
    
    // 导出CSV
    std::string suffix = dd_enabled ? "_dd_on" : "_dd_off";
    TimingStatistics::getInstance().exportCSV(output_dir + "timing" + suffix + ".csv");
    
    // 导出原始数据
    TimingStatistics::getInstance().exportRawData(output_dir + "timing" + suffix + "_raw.csv");
    
    // 导出LaTeX表格
    std::string caption = dd_enabled ? "DCGVINS Runtime (DD Enabled)" : "DCGVINS Runtime (DD Disabled)";
    TimingStatistics::getInstance().exportLatex(output_dir + "timing" + suffix + ".tex", caption);
    
    ROS_INFO("Timing statistics exported to: %s", output_dir.c_str());
    
    ros::shutdown();
}


void gnss_meas_callback_realtime(const GnssMeasMsgConstPtr &meas_msg)
{
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);
    if (gnss_meas.empty()) return;
    
    latest_gnss_time = time2sec(gnss_meas[0]->time);
    
    // CRITICAL: Add to traditional buffer for estimator
    m_buf.lock();
    gnss_meas_buf.push(gnss_meas);
    m_buf.unlock();
    con.notify_one();
    
    // 实时双差处理 - 添加线程保护
    if (ENABLE_REAL_TIME_DD && rt_dd_processor) {
        AgentGnssData agent_data;
        agent_data.agent_id = SELF_AGENT_ID;
        agent_data.timestamp = latest_gnss_time;
        agent_data.observations = gnss_meas;
        
        // 使用读锁保护estimator访问，并增加更严格的检查
        {
            std::lock_guard<std::mutex> lock(estimator_access_mutex);
            
            // 检查1: Estimator 是否存在
            if (!estimator_ptr) {
                ROS_WARN_THROTTLE(5.0, "DD skipping: Estimator not yet initialized.");
                return;
            }
            
            // 检查2: GNSS 是否已对齐
            if (!estimator_ptr->gnss_ready) {
                ROS_DEBUG_THROTTLE(5.0, "DD skipping: GNSS not ready.");
                return;
            }
            
            // 检查3: VIO 是否已初始化
            if (estimator_ptr->solver_flag != Estimator::SolverFlag::NON_LINEAR) {
                ROS_DEBUG_THROTTLE(5.0, "DD skipping: Solver not in NON_LINEAR state.");
                return;
            }
            
            // 检查4: ECEF 位置是否有效 (地心半径检查)
            double ecef_norm = estimator_ptr->ecef_pos.norm();
            if (ecef_norm < 6.0e6 || ecef_norm > 7.0e6) {
                ROS_WARN_THROTTLE(2.0, "DD skipping: Invalid ECEF position norm: %.1f", ecef_norm);
                return;
            }
            
            // 如果所有检查都通过，才打包星历和位置
            agent_data.ephemeris = estimator_ptr->getCurrentValidEphems(gnss_meas);
            agent_data.approximate_position = estimator_ptr->ecef_pos;
            
            static Eigen::Vector3d last_broadcast_pos = Eigen::Vector3d::Zero();
            double pos_change = (agent_data.approximate_position - last_broadcast_pos).norm();
            
            // 检查5: 位置变化是否合理 (防止发送跳变点)
            if (pos_change > 50.0 && last_broadcast_pos.norm() > 0) {
                 ROS_WARN("DD skipping: Large position jump detected: %.1f meters", pos_change);
                 
                 // 🔧 关键修复：接受跳变后的新位置作为下一次的参考点
                 // 这很可能是VIO-GNSS刚对齐后的第一次优化导致的合理跳变
                 // 我们跳过这一帧的处理，但更新参考位置，以便下一帧可以通过检查
                 last_broadcast_pos = agent_data.approximate_position;
                 
                 return; // 跳过这一帧异常数据，但为下一帧做好准备
            }
            last_broadcast_pos = agent_data.approximate_position;
        }
        
        // 输入到实时处理器
        rt_dd_processor->inputAgentData(agent_data);
        
        // 广播给其他agents
        if (inter_agent_comm) {
            inter_agent_comm->broadcastGnssData(
                gnss_meas, agent_data.ephemeris, agent_data.approximate_position);
        }   
        
        ROS_DEBUG("Agent %d: processed %zu GNSS observations at %.6f", 
                 SELF_AGENT_ID, gnss_meas.size(), latest_gnss_time);
    }
}

// 副设备数据接收回调 - 添加完整的空指针检查
void secondary_gnss_callback(const GnssMeasMsgConstPtr &meas_msg)
{
    if (!ENABLE_REAL_TIME_DD || !rt_dd_processor) {
        return;
    }
    
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);
    if (gnss_meas.empty()) return;
    
    double timestamp = time2sec(gnss_meas[0]->time);
    
    // 创建agent数据
    AgentGnssData agent_data;
    agent_data.agent_id = TARGET_AGENT_ID;
    agent_data.timestamp = timestamp;
    agent_data.observations = gnss_meas;
    
    // 使用统一的星历池获取星历
    for (const auto& obs : gnss_meas) {
        EphemBasePtr ephem = nullptr;
        
        // 先尝试从目标agent获取
        ephem = rt_dd_processor->getEphemerisForAgent(TARGET_AGENT_ID, obs->sat, timestamp);
        
        // 如果没有，从主agent借用
        if (!ephem) {
            ephem = rt_dd_processor->getEphemerisForAgent(SELF_AGENT_ID, obs->sat, timestamp);
            if (ephem) {
                rt_dd_processor->addEphemeris(TARGET_AGENT_ID, ephem);
                ROS_DEBUG("Borrowed ephemeris for sat %d from main to secondary", obs->sat);
            }
        }
        
        agent_data.ephemeris.push_back(ephem);
    }
    
    static int callback_count = 0;
    if (++callback_count % 10 == 0) {
        int valid_ephem = 0;
        for (const auto& e : agent_data.ephemeris) {
            if (e) valid_ephem++;
        }
        ROS_INFO("Secondary data #%d: %zu obs, %d valid ephem at %.6f", 
                 callback_count, gnss_meas.size(), valid_ephem, timestamp);
    }
    
    rt_dd_processor->inputAgentData(agent_data);
}

void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu)
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time;
    latest_time = t;

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator_ptr->g;

    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator_ptr->g;

    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);

    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;
    tmp_V = tmp_V + dt * un_acc;

    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void update()
{
    TicToc t_predict;
    latest_time = current_time;
    tmp_P = estimator_ptr->Ps[WINDOW_SIZE];
    tmp_Q = estimator_ptr->Rs[WINDOW_SIZE];
    tmp_V = estimator_ptr->Vs[WINDOW_SIZE];
    tmp_Ba = estimator_ptr->Bas[WINDOW_SIZE];
    tmp_Bg = estimator_ptr->Bgs[WINDOW_SIZE];
    acc_0 = estimator_ptr->acc_0;
    gyr_0 = estimator_ptr->gyr_0;

    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

bool getMeasurements(std::vector<sensor_msgs::ImuConstPtr> &imu_msg, sensor_msgs::PointCloudConstPtr &img_msg)
{
    if (imu_buf.empty() || feature_buf.empty())
        return false;

    // 确保有足够新的IMU数据来覆盖图像帧
    if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator_ptr->td))
        return false;

    // 丢弃比IMU还早的图像帧（通常只在启动时发生）
    if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator_ptr->td))
    {
        feature_buf.pop();
        return false;
    }

    img_msg = feature_buf.front();
    feature_buf.pop();

    std::vector<sensor_msgs::ImuConstPtr> imu_msg_tmp;
    while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator_ptr->td)
    {
        imu_msg_tmp.push_back(imu_buf.front());
        imu_buf.pop();
        if (imu_buf.empty()) break;
    }
    if (!imu_buf.empty()) {
        imu_msg_tmp.push_back(imu_buf.front());
    }
    
    imu_msg = imu_msg_tmp;
    return true;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one();

    last_imu_t = imu_msg->header.stamp.toSec();

    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        if (estimator_ptr->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

void gnss_ephem_callback(const GnssEphemMsgConstPtr &ephem_msg)
{
    EphemPtr ephem = msg2ephem(ephem_msg);
    if (!ephem) {
        ROS_WARN("Failed to parse ephemeris message");
        return;
    }
    
    // 直接加锁，不使用try_lock
    std::lock_guard<std::mutex> lock(estimator_access_mutex);
    
    if (estimator_ptr) {
        estimator_ptr->inputEphem(ephem);
        
        // 每10个星历打印一次统计
        static int ephem_count = 0;
        ephem_count++;
        if (ephem_count % 10 == 0) {
            ROS_INFO("Ephemeris status: %zu satellites, sat=%d", 
                     estimator_ptr->sat2ephem.size(), ephem->sat);
        }
    }
    
    // 同时添加到DD处理器
    if (ENABLE_REAL_TIME_DD && rt_dd_processor) {
        rt_dd_processor->addEphemeris(SELF_AGENT_ID, ephem);
        rt_dd_processor->addEphemeris(TARGET_AGENT_ID, ephem);
    }
}

void gnss_glo_ephem_callback(const GnssGloEphemMsgConstPtr &glo_ephem_msg)
{
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    if (!glo_ephem) return;
    
    std::unique_lock<std::mutex> lock(estimator_access_mutex, std::try_to_lock);
    
    if (lock.owns_lock()) {
        if (estimator_ptr) {
            estimator_ptr->inputEphem(glo_ephem);
        }
        
        if (ENABLE_REAL_TIME_DD && rt_dd_processor) {
            rt_dd_processor->addEphemeris(SELF_AGENT_ID, glo_ephem);
            rt_dd_processor->addEphemeris(TARGET_AGENT_ID, glo_ephem);
        }
    } else {
        if (ENABLE_REAL_TIME_DD && rt_dd_processor) {
            rt_dd_processor->addEphemeris(SELF_AGENT_ID, glo_ephem);
            rt_dd_processor->addEphemeris(TARGET_AGENT_ID, glo_ephem);
        }
    }
}

void secondary_ephem_callback_wrapper(const GnssEphemMsgConstPtr &ephem_msg)
{
    static int count = 0;
    count++;
    
    EphemPtr ephem = msg2ephem(ephem_msg);
    if (!ephem) {
        ROS_WARN("Failed to parse secondary ephemeris message");
        return;
    }
    
    std::unique_lock<std::mutex> lock(estimator_access_mutex, std::try_to_lock);
    
    if (lock.owns_lock()) {
        if (estimator_ptr) {
            estimator_ptr->inputEphem(ephem);
            ROS_INFO("Secondary ephem callback #%d: Added to estimator, sat=%d", 
                    count, ephem->sat);
        }
        
        if (rt_dd_processor) {
            rt_dd_processor->addEphemeris(TARGET_AGENT_ID, ephem);
            rt_dd_processor->addEphemeris(SELF_AGENT_ID, ephem);
            ROS_INFO("Secondary ephem for sat %d added to both agents", ephem->sat);
        }
    } else {
        if (rt_dd_processor) {
            rt_dd_processor->addEphemeris(TARGET_AGENT_ID, ephem);
            rt_dd_processor->addEphemeris(SELF_AGENT_ID, ephem);
        }
    }
}

void gnss_iono_params_callback(const StampedFloat64ArrayConstPtr &iono_msg)
{
    double ts = iono_msg->header.stamp.toSec();
    std::vector<double> iono_params;
    std::copy(iono_msg->data.begin(), iono_msg->data.end(), std::back_inserter(iono_params));
    assert(iono_params.size() == 8);
    estimator_ptr->inputIonoParams(ts, iono_params);
}

void gnss_meas_callback(const GnssMeasMsgConstPtr &meas_msg)
{
    std::vector<ObsPtr> gnss_meas = msg2meas(meas_msg);

    latest_gnss_time = time2sec(gnss_meas[0]->time);

    // cerr << "gnss ts is " << std::setprecision(20) << time2sec(gnss_meas[0]->time) << endl;
    if (!time_diff_valid)   return;

    m_buf.lock();
    gnss_meas_buf.push(std::move(gnss_meas));
    m_buf.unlock();
    con.notify_one();
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    ++feature_msg_counter;
    debug_feature_received_count++;
    
    // 【调试】第一条feature消息
    if (debug_feature_received_count == 1)
    {
        ROS_INFO("[DEBUG] 收到第一条Feature消息! ts=%.3f, 特征点数=%zu",
                 feature_msg->header.stamp.toSec(), feature_msg->points.size());
    }
    
    // 降采样比例: 30Hz Camera / 10Hz GNSS = 3
    const int downsample_ratio = 3;
    
    // 【调试】打印相位锁定前的状态
    if (debug_feature_received_count % 100 == 1)
    {
        ROS_INFO("[DEBUG] feature #%lu: skip_param=%d, time_diff_valid=%d, latest_gnss_time=%.3f",
                 debug_feature_received_count, skip_parameter, time_diff_valid, latest_gnss_time);
    }
    
    // 相位锁定逻辑
    if (skip_parameter < 0 && time_diff_valid)
    {
        const double this_feature_ts = feature_msg->header.stamp.toSec() + time_diff_gnss_local;
        
        // 【调试】打印相位锁定尝试
        if (feature_msg_counter % 30 == 1)
        {
            ROS_INFO("[DEBUG] 相位锁定检查: feature_ts=%.3f, latest_gnss=%.3f, tmp_last=%.3f",
                     this_feature_ts, latest_gnss_time, tmp_last_feature_time);
        }
        
        if (latest_gnss_time > 0 && tmp_last_feature_time > 0)
        {
            // 使用 std::abs
            double current_diff = std::abs(this_feature_ts - latest_gnss_time);
            double last_diff = std::abs(tmp_last_feature_time - latest_gnss_time);
            
            // 【调试】
            ROS_INFO("[DEBUG] 相位比较: current_diff=%.4f, last_diff=%.4f", current_diff, last_diff);

            // 如果当前误差比上一帧大，说明上一帧是最佳时刻
            if (current_diff > last_diff) 
            {
                // 【小优化】防止 counter=0 时溢出 (虽然这里counter肯定>0)
                skip_parameter = (feature_msg_counter + downsample_ratio - 1) % downsample_ratio;
                
                // 打印一下锁定信息，方便调试
                ROS_INFO("=========================================");
                ROS_INFO("GVINS Time Alignment Locked!");
                ROS_INFO("  Ratio: %d, Phase: %d", downsample_ratio, skip_parameter);
                ROS_INFO("  feature_counter: %lu", feature_msg_counter);
                ROS_INFO("=========================================");
            }
        }
        tmp_last_feature_time = this_feature_ts;
    }
    
    // 【调试】如果skip_parameter还是负数，说明相位锁定条件不满足
    if (skip_parameter < 0)
    {
        if (debug_feature_received_count % 100 == 1)
        {
            ROS_WARN("[DEBUG] skip_parameter < 0, feature不会被处理!");
            ROS_WARN("[DEBUG]   原因检查: time_diff_valid=%d, latest_gnss_time=%.1f", 
                     time_diff_valid, latest_gnss_time);
        }
        return; // 提前返回，避免下面的条件判断
    }
    
    // 执行降采样
    if (skip_parameter >= 0 && int(feature_msg_counter % downsample_ratio) == skip_parameter)
    {
        debug_feature_pushed_count++;
        
        m_buf.lock();
        feature_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
        
        // 【调试】
        if (debug_feature_pushed_count % 30 == 1)
        {
            ROS_INFO("[DEBUG] Feature #%lu 推入buffer (总共推入%lu个), feature_buf size=%zu",
                     feature_msg_counter, debug_feature_pushed_count, feature_buf.size());
        }
    }
}
/*
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    ++ feature_msg_counter;

    if (skip_parameter < 0 && time_diff_valid)
    {
        const double this_feature_ts = feature_msg->header.stamp.toSec()+time_diff_gnss_local;
        if (latest_gnss_time > 0 && tmp_last_feature_time > 0)
        {
            if (abs(this_feature_ts - latest_gnss_time) > abs(tmp_last_feature_time - latest_gnss_time))
                skip_parameter = feature_msg_counter%2;       // skip this frame and afterwards
            else
                skip_parameter = 1 - (feature_msg_counter%2);   // skip next frame and afterwards
        }
        // cerr << "feature counter is " << feature_msg_counter << ", skip parameter is " << int(skip_parameter) << endl;
        tmp_last_feature_time = this_feature_ts;
    }

    if (skip_parameter >= 0 && int(feature_msg_counter%2) != skip_parameter)
    {
        m_buf.lock();
        feature_buf.push(feature_msg);
        m_buf.unlock();
        con.notify_one();
    }
}
*/
void local_trigger_info_callback(const gvins::LocalSensorExternalTriggerConstPtr &trigger_msg)
{
    std::lock_guard<std::mutex> lg(m_time);

    if (next_pulse_time_valid)
    {
        time_diff_gnss_local = next_pulse_time - trigger_msg->header.stamp.toSec();
        estimator_ptr->inputGNSSTimeDiff(time_diff_gnss_local);
        if (!time_diff_valid)       // just get calibrated
            std::cout << "time difference between GNSS and VI-Sensor got calibrated: "
                << std::setprecision(15) << time_diff_gnss_local << " s\n";
        time_diff_valid = true;
    }
}

void gnss_tp_info_callback(const GnssTimePulseInfoMsgConstPtr &tp_msg)
{
    gtime_t tp_time = gpst2time(tp_msg->time.week, tp_msg->time.tow);
    if (tp_msg->utc_based || tp_msg->time_sys == SYS_GLO)
        tp_time = utc2gpst(tp_time);
    else if (tp_msg->time_sys == SYS_GAL)
        tp_time = gst2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_BDS)
        tp_time = bdt2time(tp_msg->time.week, tp_msg->time.tow);
    else if (tp_msg->time_sys == SYS_NONE)
    {
        std::cerr << "Unknown time system in GNSSTimePulseInfoMsg.\n";
        return;
    }
    double gnss_ts = time2sec(tp_time);

    std::lock_guard<std::mutex> lg(m_time);
    next_pulse_time = gnss_ts;
    next_pulse_time_valid = true;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator_ptr->clearState();
        estimator_ptr->setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

// estimator/src/estimator_node.cpp

void process()
{
    while (true)
    {
        // 1. 数据同步
        std::vector<sensor_msgs::ImuConstPtr> imu_msg_buffer;
        sensor_msgs::PointCloudConstPtr img_msg;
        std::vector<ObsPtr> gnss_msg_buffer;

        {
            std::unique_lock<std::mutex> lk(m_buf);
            con.wait(lk, [&] {
                return getMeasurements(imu_msg_buffer, img_msg);
            });
            
            if (GNSS_ENABLE && !gnss_meas_buf.empty()) {
                while(gnss_meas_buf.size() > 1) {
                    gnss_meas_buf.pop();
                }
                gnss_msg_buffer = gnss_meas_buf.front();
                gnss_meas_buf.pop();
            }
        }

        // 2. 主处理（持锁保护）
        m_estimator.lock();
        
        // a. IMU处理
        for (auto &imu_data : imu_msg_buffer)
        {
            double t = imu_data->header.stamp.toSec();
            double img_t = img_msg->header.stamp.toSec() + estimator_ptr->td;
            
            if (t <= img_t)
            { 
                if (current_time < 0)
                    current_time = t;
                double dt = t - current_time;
                ROS_ASSERT(dt >= 0);
                current_time = t;
                double dx = imu_data->linear_acceleration.x;
                double dy = imu_data->linear_acceleration.y;
                double dz = imu_data->linear_acceleration.z;
                double rx = imu_data->angular_velocity.x;
                double ry = imu_data->angular_velocity.y;
                double rz = imu_data->angular_velocity.z;
                estimator_ptr->processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
            }
            else
            {
                double dt_1 = img_t - current_time;
                double dt_2 = t - img_t;
                current_time = img_t;
                ROS_ASSERT(dt_1 >= 0);
                ROS_ASSERT(dt_2 >= 0);
                ROS_ASSERT(dt_1 + dt_2 > 0);
                double w1 = dt_2 / (dt_1 + dt_2);
                double w2 = dt_1 / (dt_1 + dt_2);
                double dx = w1 * acc_0.x() + w2 * imu_data->linear_acceleration.x;
                double dy = w1 * acc_0.y() + w2 * imu_data->linear_acceleration.y;
                double dz = w1 * acc_0.z() + w2 * imu_data->linear_acceleration.z;
                double rx = w1 * gyr_0.x() + w2 * imu_data->angular_velocity.x;
                double ry = w1 * gyr_0.y() + w2 * imu_data->angular_velocity.y;
                double rz = w1 * gyr_0.z() + w2 * imu_data->angular_velocity.z;
                estimator_ptr->processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
            }
        }
        
        // b. GNSS处理
        if (!gnss_msg_buffer.empty())
        {
            estimator_ptr->processGNSS(gnss_msg_buffer);
        }

        // c. 图像处理
        ROS_DEBUG("processing vision data with stamp %f", img_msg->header.stamp.toSec());
        TicToc t_s;
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
        for (unsigned int i = 0; i < img_msg->points.size(); i++)
        {
            int v = img_msg->channels[0].values[i] + 0.5;
            int feature_id = v / NUM_OF_CAM;
            int camera_id = v % NUM_OF_CAM;
            double x = img_msg->points[i].x;
            double y = img_msg->points[i].y;
            double z = img_msg->points[i].z;
            double p_u = img_msg->channels[1].values[i];
            double p_v = img_msg->channels[2].values[i];
            double velocity_x = img_msg->channels[3].values[i];
            double velocity_y = img_msg->channels[4].values[i];
            ROS_ASSERT(z == 1);
            Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
            xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
            image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
        }
        estimator_ptr->processImage(image, img_msg->header);

        // d. 发布可视化
        double whole_t = t_s.toc();
        printStatistics(*estimator_ptr, whole_t);
        std_msgs::Header header = img_msg->header;
        header.frame_id = "world";

        pubOdometry(*estimator_ptr, header);
        pubKeyPoses(*estimator_ptr, header);
        pubCameraPose(*estimator_ptr, header);
        pubPointCloud(*estimator_ptr, header);
        pubTF(*estimator_ptr, header);
        pubKeyframe(*estimator_ptr);

        // 🆕 GNSS和双差可视化
        if (estimator_ptr->gnss_ready) {
            pubGnssResult(*estimator_ptr, header);
            
            // 双差可视化
            #ifdef ENABLE_REAL_TIME_DD
            #if ENABLE_REAL_TIME_DD == 1
            auto rt_processor = estimator_ptr->getRealTimeDDProcessor();
            if (rt_processor != nullptr) {
                auto dd_obs_list = rt_processor->getCurrentDDObservations();
                
                if (!dd_obs_list.empty()) {
                    // 发布副设备路径（只取最新的位置）
                    for (const auto& dd : dd_obs_list) {
                        Eigen::Vector3d secondary_ecef;
                        if (estimator_ptr->getSecondaryPositionAtTime(dd.timestamp, secondary_ecef)) {
                            pubSecondaryGNSSPath(secondary_ecef, header, 
                                               estimator_ptr->anc_ecef, 
                                               estimator_ptr->R_ecef_enu);
                            break;
                        }
                    }
                    
                    // 发布双差约束、卫星、信息
                    pubDDConstraints(*estimator_ptr, dd_obs_list, header);
                    pubSatelliteMarkers(*estimator_ptr, dd_obs_list, header);
                    pubDDInfoText(*estimator_ptr, dd_obs_list, header);
                    
                    ROS_DEBUG("Published %zu DD constraints", dd_obs_list.size());
                }
            }
            #endif
            #endif
        }

        m_estimator.unlock();

        // 3. 状态更新
        m_buf.lock();
        m_state.lock();
        if (estimator_ptr->solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

void gnss_ephem_callback_with_agent(const GnssEphemMsgConstPtr &ephem_msg, int agent_id) 
{
    if (!ENABLE_REAL_TIME_DD || !rt_dd_processor) return;
    
    EphemPtr ephem = msg2ephem(ephem_msg);
    rt_dd_processor->addEphemeris(agent_id, ephem);
    
    // 添加调试信息
    static int secondary_ephem_count = 0;
    secondary_ephem_count++;
    ROS_INFO_THROTTLE(1.0, "Agent %d ephem received: count=%d, sat_id=%d", 
                       agent_id, secondary_ephem_count, ephem->sat);
}

void gnss_glo_ephem_callback_with_agent(const GnssGloEphemMsgConstPtr &glo_ephem_msg, int agent_id) 
{
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    
    if (agent_id == SELF_AGENT_ID) {
        estimator_ptr->inputEphem(glo_ephem);
    }
    
    if (rt_dd_processor) {
        rt_dd_processor->addEphemeris(agent_id, glo_ephem);
    }
}

void secondary_glo_ephem_callback_wrapper(const GnssGloEphemMsgConstPtr &glo_ephem_msg)
{
    if (!glo_ephem_msg) return;
    if (!rt_dd_processor) return;
    
    GloEphemPtr glo_ephem = msg2glo_ephem(glo_ephem_msg);
    if (!glo_ephem) {
        ROS_WARN("Failed to parse secondary GLONASS ephemeris");
        return;
    }
    // Add to estimator
    if (estimator_ptr) {
        estimator_ptr->inputEphem(glo_ephem);
    }
    
    // Add to DD processor
    if (rt_dd_processor) {
        rt_dd_processor->addEphemeris(TARGET_AGENT_ID, glo_ephem);
        rt_dd_processor->addEphemeris(SELF_AGENT_ID, glo_ephem);
    }
}

void secondary_pvt_callback(const gnss_comm::GnssPVTSolnMsgConstPtr& pvt_msg)
{
    if (!estimator_ptr || !ENABLE_REAL_TIME_DD) {
        return;
    }
    
    double lat_deg = pvt_msg->latitude;
    double lon_deg = pvt_msg->longitude;
    double alt_m = pvt_msg->altitude;
    
    const double deg2rad = M_PI / 180.0; 
    double WGS84_A = 6378137.0;
    double WGS84_E2 = 0.00669437999014;
    
    double lat_rad = lat_deg * deg2rad;
    double lon_rad = lon_deg * deg2rad;
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));
    
    double ecef_x = (N + alt_m) * std::cos(lat_rad) * std::cos(lon_rad);
    double ecef_y = (N + alt_m) * std::cos(lat_rad) * std::sin(lon_rad);
    double ecef_z = ((1.0 - WGS84_E2) * N + alt_m) * std::sin(lat_rad);
    
    Eigen::Vector3d pos_ecef(ecef_x, ecef_y, ecef_z);
    double pos_norm = pos_ecef.norm();
    
    if (pos_norm < 6.3e6 || pos_norm > 6.4e6) {
        ROS_WARN("Invalid secondary position");
        return;
    }
    
    gtime_t pvt_gtime = gpst2time(pvt_msg->time.week, pvt_msg->time.tow);
    double timestamp = time2sec(pvt_gtime);
    
    {
        std::lock_guard<std::mutex> lock(estimator_access_mutex);
        estimator_ptr->addSecondaryPosition(timestamp, pos_ecef);
    }
    
    static int callback_count = 0;
    callback_count++;
    if (callback_count % 100 == 0) {
        std::lock_guard<std::mutex> lock(estimator_access_mutex);
        estimator_ptr->cleanSecondaryPositionBuffer(timestamp, 10.0);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gvins");
    ros::NodeHandle n("~");
    signal(SIGINT, timingShutdownCallback);
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    
    ROS_INFO("========================================");
    ROS_INFO("Starting GVINS Estimator with Real-time DD");
    ROS_INFO("========================================");
    
    // ==================== 1. 参数加载 ====================
    readParameters(n);
    ROS_INFO("✓ Parameters loaded");
    
    // ==================== 2. 初始化Estimator ====================
    estimator_ptr.reset(new Estimator());
    estimator_ptr->setParameter();
    ROS_INFO("✓ GVINS estimator initialized");
    
    // ==================== 3. 注册发布器 ====================
    registerPub(n);
    ROS_INFO("✓ Publishers registered");
    
    // ==================== 4. 初始化DD处理器 ====================
    if (GNSS_ENABLE && ENABLE_REAL_TIME_DD) {
        ROS_INFO("=== Initializing Real-time Cooperative Localization ===");
        
        try {
            rt_dd_processor = std::make_unique<RealTimeDDProcessor>(DD_TIME_SYNC_THRESHOLD);
            ROS_INFO("✓ Real-time DD processor initialized with sync threshold: %.3f s", 
                     DD_TIME_SYNC_THRESHOLD);
            
            rt_dd_processor->initializeAgent(SELF_AGENT_ID);
            ROS_INFO("✓ Initialized ephemeris manager for agent %d", SELF_AGENT_ID);
            
            rt_dd_processor->initializeAgent(TARGET_AGENT_ID);
            ROS_INFO("✓ Initialized ephemeris manager for agent %d", TARGET_AGENT_ID);
            
            // 链接到estimator
            estimator_ptr->setRealTimeDDProcessor(rt_dd_processor.get());
            ROS_INFO("✓ DD processor linked to estimator");
            
        } catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize DD processor: %s", e.what());
            return -1;
        }
    }
    
    // ==================== 5. 时间同步初始化 ====================
    next_pulse_time_valid = false;
    latest_gnss_time = -1;
    tmp_last_feature_time = -1;
    feature_msg_counter = 0;
    /*
    skip_parameter = GNSS_ENABLE ? -1 : 0;
    */
    keep_phase = GNSS_ENABLE ? -1 : 0;  // GNSS禁用时直接开始采样
    
    if (!GNSS_LOCAL_ONLINE_SYNC && GNSS_ENABLE) {
        time_diff_gnss_local = GNSS_LOCAL_TIME_DIFF;
        estimator_ptr->inputGNSSTimeDiff(time_diff_gnss_local);
        time_diff_valid = true;
        skip_parameter = 0;
        ROS_INFO("✓ Time sync set to fixed value: %.6f", time_diff_gnss_local);
    }
    
    // ==================== 6. 基础订阅器 ====================
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, 
                                          ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_feature = n.subscribe("/gvins_feature_tracker/feature", 2000, 
                                              feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/gvins_feature_tracker/restart", 2000, 
                                              restart_callback);
    ROS_INFO("✓ Basic subscribers registered (IMU, Feature, Restart)");
    ros::Subscriber sub_secondary_pvt;
    if (GNSS_ENABLE && ENABLE_REAL_TIME_DD) {
        sub_secondary_pvt = n.subscribe("/ublox_driver_secondary/receiver_pvt", 
                                       100, secondary_pvt_callback);
        ROS_INFO("Subscribed to secondary PVT: /ublox_driver_secondary/receiver_pvt");
    }
    // ==================== 7. GNSS订阅器（分阶段） ====================
    ros::Subscriber sub_ephem, sub_glo_ephem, sub_gnss_meas, sub_gnss_iono_params;
    ros::Subscriber sub_gnss_time_pulse_info, sub_local_trigger_info;
    ros::Subscriber sub_secondary_gnss, sub_secondary_ephem, sub_secondary_glo_ephem;
    
    if (GNSS_ENABLE)
    {
        ROS_INFO("=== Configuring GNSS Subscribers ===");
        
        // 阶段1: 先订阅星历（主设备和副设备）
        ROS_INFO("Phase 1: Subscribing to ephemeris topics...");
        
        sub_ephem = n.subscribe(GNSS_EPHEM_TOPIC, 100, 
            gnss_ephem_callback, ros::TransportHints().tcpNoDelay());
        ROS_INFO(" Main device ephemeris: %s", GNSS_EPHEM_TOPIC.c_str());
        
        sub_glo_ephem = n.subscribe(GNSS_GLO_EPHEM_TOPIC, 100,
            gnss_glo_ephem_callback, ros::TransportHints().tcpNoDelay());
        ROS_INFO(" Main device GLO ephemeris: %s", GNSS_GLO_EPHEM_TOPIC.c_str());
        
        // 如果是实时DD模式，订阅副设备星历
        if (ENABLE_REAL_TIME_DD) {
            std::string secondary_ephem_topic = "/ublox_driver_secondary/ephem";
            std::string secondary_glo_ephem_topic = "/ublox_driver_secondary/glo_ephem";
            
            sub_secondary_ephem = n.subscribe(secondary_ephem_topic, 100,
                secondary_ephem_callback_wrapper, ros::TransportHints().tcpNoDelay());
            ROS_INFO(" Secondary device ephemeris: %s", secondary_ephem_topic.c_str());
            
            sub_secondary_glo_ephem = n.subscribe(secondary_glo_ephem_topic, 100,
                secondary_glo_ephem_callback_wrapper, ros::TransportHints().tcpNoDelay());
            ROS_INFO(" Secondary device GLO ephemeris: %s", 
                     secondary_glo_ephem_topic.c_str());
        }
        
        // 阶段1: 等待星历数据，使用非阻塞检查
        ROS_INFO("Waiting for ephemeris data...");
        ros::Duration wait_time(6.0);  // 减少到3秒，避免长时间阻塞
        ros::Time start_wait = ros::Time::now();

        int ephem_count = 0;
        bool ephemeris_ready = false;

        while ((ros::Time::now() - start_wait) < wait_time && !ephemeris_ready) {
            ros::spinOnce();
            ros::Duration(0.05).sleep();  // 减少检查间隔
            
            // 使用try_lock避免死锁
            std::unique_lock<std::mutex> lock(estimator_access_mutex, std::try_to_lock);
            if (lock.owns_lock() && estimator_ptr) {
                ephem_count = estimator_ptr->sat2ephem.size();
                if (ephem_count >= 2) {  // 降低要求，2颗卫星即可
                    ephemeris_ready = true;
                    ROS_INFO("✓ Ephemeris ready: %d satellites", ephem_count);
                    break;
                }
            }
            
            // 每秒打印一次状态，避免静默等待
            static ros::Time last_status_time = ros::Time::now();
            if ((ros::Time::now() - last_status_time).toSec() > 1.0) {
                ROS_INFO("Still waiting for ephemeris... (%d satellites so far)", ephem_count);
                last_status_time = ros::Time::now();
            }
        }

        // 验证星历接收状态
        if (ephemeris_ready) {
            ROS_INFO("Ephemeris ready with %d satellites", ephem_count);
        } else {
            ROS_WARN("Timeout waiting for ephemeris! Continuing with %d satellites", ephem_count);
            ROS_WARN("System may work with reduced accuracy");
            // 不要return -1，继续运行
        }

        
        // 阶段2: 订阅电离层参数
        ROS_INFO("Phase 2: Subscribing to ionosphere parameters...");
        sub_gnss_iono_params = n.subscribe(GNSS_IONO_PARAMS_TOPIC, 100, 
                                        gnss_iono_params_callback);
        // 阶段3: 最后订阅观测数据
        ROS_INFO("Phase 3: Subscribing to measurement topics...");
        
        if (ENABLE_REAL_TIME_DD) {
            // 主设备观测
            sub_gnss_meas = n.subscribe(GNSS_MEAS_TOPIC, 100, 
                gnss_meas_callback_realtime, ros::TransportHints().tcpNoDelay());
            ROS_INFO("  Main device measurements (RT mode): %s", GNSS_MEAS_TOPIC.c_str());
            
            // 副设备观测
            std::string secondary_meas_topic = "/ublox_driver_secondary/range_meas";
            sub_secondary_gnss = n.subscribe(secondary_meas_topic, 100, 
                secondary_gnss_callback, ros::TransportHints().tcpNoDelay());
            ROS_INFO("  Secondary device measurements: %s", secondary_meas_topic.c_str());
            
        } else {
            sub_gnss_meas = n.subscribe(GNSS_MEAS_TOPIC, 100, 
                gnss_meas_callback, ros::TransportHints().tcpNoDelay());
            ROS_INFO("  Main device measurements (normal mode): %s", 
                    GNSS_MEAS_TOPIC.c_str());
        }
        
        // 时间脉冲订阅（如果需要）
        if (GNSS_LOCAL_ONLINE_SYNC) {
            sub_gnss_time_pulse_info = n.subscribe(GNSS_TP_INFO_TOPIC, 100, 
                                                    gnss_tp_info_callback);
            sub_local_trigger_info = n.subscribe(LOCAL_TRIGGER_INFO_TOPIC, 100, 
                                                local_trigger_info_callback);
            ROS_INFO("  Time pulse synchronization enabled");
        }
        
        ROS_INFO("All GNSS subscribers configured");
    }
    
    // ==================== 8. 启动处理线程 ====================
    ROS_INFO("Starting measurement processing thread...");
    std::thread measurement_process{process};
    ROS_INFO("Processing thread started");
    
   // ==================== 9. 统计定时器（简洁修复版）====================
    ros::Timer stats_timer = n.createTimer(ros::Duration(2.0), 
        [&](const ros::TimerEvent&) {
            static int stats_count = 0;
            stats_count++;
            
            ROS_INFO("========== Status Report #%d ==========", stats_count);
            
            // DD处理器统计
            if (rt_dd_processor) {
                int total_obs = 0, valid_dd = 0;
                double avg_quality = 0.0;
                
                try {
                    rt_dd_processor->getStatistics(total_obs, valid_dd, avg_quality);
                    ROS_INFO("===== DD Processor Statistics =====");
                    ROS_INFO("Total observations: %d", total_obs);
                    ROS_INFO("Valid DD count: %d", valid_dd);
                    ROS_INFO("Average quality: %.2f", avg_quality);
                } catch (const std::exception& e) {
                    ROS_ERROR("Failed to get DD statistics: %s", e.what());
                }
                
                // 星历统计 - 注意：修复 ephemeris_manager.hpp 后这里就不会死锁了
                try {
                    ROS_INFO("=== Ephemeris Statistics ===");
                    rt_dd_processor->printEphemerisStatistics();
                } catch (const std::exception& e) {
                    ROS_ERROR("Failed to print ephemeris statistics: %s", e.what());
                }
            }
            
            // 估计器状态 - 使用try_lock避免阻塞主处理线程
            {
                std::unique_lock<std::mutex> lock(estimator_access_mutex, std::try_to_lock);
                if (lock.owns_lock() && estimator_ptr) {
                    ROS_INFO("===== Estimator Status =====");
                    ROS_INFO("GNSS ready: %s", estimator_ptr->gnss_ready ? "YES" : "NO");
                    ROS_INFO("Solver flag: %s", 
                            estimator_ptr->solver_flag == Estimator::NON_LINEAR ? 
                            "NON_LINEAR" : "INITIAL");
                    ROS_INFO("Frame count: %d", estimator_ptr->frame_count);
                    
                    if (!estimator_ptr->sat2ephem.empty()) {
                        ROS_INFO("Satellites with ephemeris: %zu", 
                                estimator_ptr->sat2ephem.size());
                    } else {
                        ROS_WARN("⚠ No ephemeris in estimator yet!");
                    }
                    
                    int total_gnss_obs = 0;
                    for (int i = 0; i <= WINDOW_SIZE; i++) {
                        total_gnss_obs += estimator_ptr->gnss_meas_buf[i].size();
                    }
                    if (total_gnss_obs > 0) {
                        ROS_INFO("Total GNSS observations in buffer: %d", total_gnss_obs);
                    }
                } else {
                    ROS_INFO("===== Estimator Status =====");
                    ROS_INFO("Estimator busy with optimization, skipping detailed stats");
                }
            }
            
            ROS_INFO("=====================================");
        });

    // ==================== 10. 启动完成 ====================
    ROS_INFO("========================================");
    ROS_INFO("✓✓✓ System initialization complete ✓✓✓");
    ROS_INFO("Waiting for sensor data...");
    ROS_INFO("========================================");
    
    ros::spin();
    
    return 0;
}