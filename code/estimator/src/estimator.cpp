#include "estimator.h"
#include <iomanip>
#include <fstream>
#include <mutex>
#include "real_time_dd_processor.hpp" 
#include "factor/gnss_dd_factor_realtime.hpp"
#include "utility/timing_statistics.h"

void Estimator::addGnssDDFactor(int frame_count, const ObsData &obs1, const ObsData &obs2)
{
    ROS_WARN("addGnssDDFactor function is deprecated, DD factors are added in optimization()");
}

Estimator::~Estimator()
{
    ROS_INFO("Destroying GVINS Estimator...");

    // 安全清理资源 (delete nullptr 是安全的)
    try {
        // 清理实时DD处理器指针 (如果 Estimator 拥有它，则需要 delete 或 reset unique_ptr)
        // rt_dd_processor_ = nullptr; // 如果是指针，清空；如果是 unique_ptr，会自动处理

        // 清理嵌套类 (unique_ptr 会自动处理)
        fault_tolerance_mgr_.reset();
        time_synchronizer_.reset();
        dd_cache_.reset();

        // 清理预积分
        for (int i = 0; i <= WINDOW_SIZE; i++) {
            delete pre_integrations[i]; // delete nullptr is safe
            pre_integrations[i] = nullptr;
        }

        delete tmp_pre_integration; // delete nullptr is safe
        tmp_pre_integration = nullptr;

        // 清理边际化信息
        delete last_marginalization_info; // delete nullptr is safe
        last_marginalization_info = nullptr;

    } catch (const std::exception& e) {
        ROS_ERROR("Exception during estimator destruction: %s", e.what());
    }

    ROS_INFO("✓ Estimator destruction complete");
}

Estimator::Estimator() :
    // --- 初始化列表 ---
    // 假设 f_manager 在头文件中先声明
    dd_manager(1000),
    f_manager{Rs},
    // 假设 dd_manager 在头文件中声明（注意：它不是指针，直接构造）
    // GnssDDManager 不是指针，在此构造
    // 假设指针成员在头文件中接着声明
    tmp_pre_integration(nullptr),
    last_marginalization_info(nullptr),
    rt_dd_processor_(nullptr),
    // 假设 debug_logger_ 在头文件中较后声明
    debug_logger_("/tmp/gvins_debug/dd_debug_" + std::to_string(ros::Time::now().toSec()) + ".csv") // <-- 在初始化列表中初始化
{
    // --- 构造函数体 ---
    ROS_INFO("Initializing GVINS Estimator..."); // 或者 "init begins"

    // 初始化 pre_integrations 数组
    for (int i = 0; i <= WINDOW_SIZE; ++i) {
        pre_integrations[i] = nullptr;
    }

    // 初始化标志位和其他简单成员
    failure_occur = false;
    first_imu = false; // 假设 first_imu 默认为 false，根据你的逻辑调整
    gnss_ready = false;
    first_optimization = true;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0; // 初始化时间戳
    td = TD; // 从 parameters.h 获取初始值

    // 调用 clearState() 会将许多成员重置，包括将指针设为 nullptr，这是安全的
    clearState(); // 调用 clearState 重置状态

    // 初始化嵌套类（从第二个构造函数合并过来的逻辑）
    try {
        fault_tolerance_mgr_ = std::make_unique<FaultToleranceManager>();
        ROS_INFO("Fault tolerance manager initialized");
        time_synchronizer_ = std::make_unique<TimeSynchronizer>();
        ROS_INFO("Time synchronizer initialized");
        dd_cache_ = std::make_unique<DDConstraintCache>();
        ROS_INFO("DD constraint cache initialized");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize estimator components: %s", e.what());
        // 设置安全的默认状态
        fault_tolerance_mgr_ = nullptr;
        time_synchronizer_ = nullptr;
        dd_cache_ = nullptr;
    }

    // 检查 DD 管理器（它已经在初始化列表中构造）
    // 通常不需要在这里重新构造或赋值，除非有特殊逻辑
    // try {
    //     // dd_manager 已经在初始化列表构造
    //     ROS_INFO("✓ GNSS DD manager already initialized via initializer list");
    // } catch (const std::exception& e) {
    //     ROS_ERROR("Error related to DD manager after initialization: %s", e.what());
    // }

    // 初始化统计信息（从第二个构造函数合并过来的逻辑）
    time_matching_stats_.total_attempts = 0;
    time_matching_stats_.successful_matches = 0;
    time_matching_stats_.avg_time_diff = 0.0;
    time_matching_stats_.max_time_diff = 0.0;
    // time_matching_stats_.recent_diffs.clear(); // 如果需要清空

    // 移除构造函数体内部的 debug_logger_ 赋值语句
    // debug_logger_ = DebugLogger("/tmp/gvins_debug"); // <--- 确保这行已被删除

    ROS_INFO("Estimator initialization complete");
}

void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
    }
    f_manager.setRic(ric);
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD;
}

// --- 替换掉旧的 clearState 函数 ---
void Estimator::clearState()
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        // 检查非空后删除，并置为 nullptr
        if (pre_integrations[i] != nullptr) {
            delete pre_integrations[i];
            pre_integrations[i] = nullptr;
        }
        
        // 清理GNSS缓冲区
        gnss_meas_buf[i].clear();
        gnss_ephem_buf[i].clear();
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    // 重置GNSS状态
    gnss_ready = false;
    anc_ecef.setZero();
    R_ecef_enu.setIdentity();
    para_yaw_enu_local[0] = 0;
    yaw_enu_local = 0;
    sat2ephem.clear();
    sat2time_index.clear();
    sat_track_status.clear();
    latest_gnss_iono_params.clear();
    std::copy(GNSS_IONO_DEFAULT_PARAMS.begin(), GNSS_IONO_DEFAULT_PARAMS.end(), 
        std::back_inserter(latest_gnss_iono_params));
    diff_t_gnss_local = 0;

    first_optimization = true;

    // 检查非空后删除，并置为 nullptr
    if (tmp_pre_integration != nullptr) {
        delete tmp_pre_integration;
        tmp_pre_integration = nullptr;
    }
    if (last_marginalization_info != nullptr) {
        delete last_marginalization_info;
        last_marginalization_info = nullptr;
    }
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState();

    failure_occur = false; // 确保重置

    // 清理副设备位置缓冲区
    {
        std::lock_guard<std::mutex> lock(secondary_buffer_mutex_);
        secondary_position_buffer_.clear();
    }
}
// --- clearState 替换结束 ---

// DDConstraintCache implementations
void Estimator::DDConstraintCache::updateCache(double window_start, double window_end,
                                              const std::vector<ObsData>& observations) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    for (size_t i = 0; i < observations.size(); i++) {
        const auto& obs = observations[i];
        
        if (obs.timestamp < window_start - 1.0 || obs.timestamp > window_end + 1.0) {
            continue;
        }
        
        if (cache_.find(obs.timestamp) == cache_.end()) {
            CachedConstraint constraint;
            constraint.timestamp = obs.timestamp;
            constraint.frame_idx = i;
            constraint.factor = nullptr;
            constraint.quality = GNSS_DD_WEIGHT;
            constraint.used = false;
            
            cache_[obs.timestamp].push_back(constraint);
        }
    }
}

std::vector<Estimator::DDConstraintCache::CachedConstraint*> 
Estimator::DDConstraintCache::getValidConstraints(double current_time) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    std::vector<CachedConstraint*> valid;
    
    for (auto& time_pair : cache_) {
        if (std::abs(time_pair.first - current_time) < 5.0) {
            for (auto& constraint : time_pair.second) {
                if (!constraint.used) {
                    valid.push_back(&constraint);
                }
            }
        }
    }
    return valid;
}

void Estimator::DDConstraintCache::clearOldCache(double current_time) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    auto it = cache_.begin();
    while (it != cache_.end()) {
        if (it->first < current_time - 10.0) {
            it = cache_.erase(it);
        } else {
            ++it;
        }
    }
}


void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    TIME_MODULE("IMU_Integration"); 
    
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }

    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g;
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j];
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix();
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g;
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc;
        Vs[j] += dt * un_acc;
    }
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    TIME_MODULE("ProcessImage_Total");

    if (image.empty()) {
        ROS_WARN("Empty image features received, skipping frame");
        return;
    }

    // 特征处理计时
    TIME_START(feature);  // <-- 添加
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());

    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD;
    else
        marginalization_flag = MARGIN_SECOND_NEW;
    TIME_END(feature, "Feature_Processing");  // <-- 添加

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header;

    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration;
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1;
            }
        }
    }
    
    // 添加调试信息
    if (frame_count % 10 == 0) {
        debugGNSSStatus();
        ROS_INFO("Frame %d: GNSS_ready=%d, solver_flag=%d, features=%d", 
                 frame_count, gnss_ready, solver_flag, f_manager.getFeatureCount());
        if (!gnss_ready && GNSS_ENABLE) {
            ROS_INFO("Waiting for GNSS-VI alignment...");
            // 打印GNSS缓冲区状态
            for (int i = 0; i <= WINDOW_SIZE; i++) {
                ROS_INFO("  gnss_meas_buf[%d]: %zu measurements", i, gnss_meas_buf[i].size());
            }
        }
    }

    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
                result = initialStructure();
                initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR;
                solveOdometry();
                slideWindow();
                f_manager.removeFailures();
                ROS_INFO("Initialization finish!");
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
            {
                slideWindow();
            }
        }
        else
            frame_count++;
    }
    else
    {
        TicToc t_solve;
        TIME_START(solve);  // <-- 添加
        solveOdometry();
        TIME_END(solve, "SolveOdometry_Total");  // <-- 添加

        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState();
            setParameter();
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        TIME_START(margin);  // <-- 添加
        slideWindow();
        f_manager.removeFailures();
        TIME_END(margin, "Marginalization");  // <-- 添加
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

void Estimator::inputEphem(EphemBasePtr ephem_ptr)
{
    double toe = time2sec(ephem_ptr->toe);
    // if a new ephemeris comes
    if (sat2time_index.count(ephem_ptr->sat) == 0 || sat2time_index.at(ephem_ptr->sat).count(toe) == 0)
    {
        sat2ephem[ephem_ptr->sat].emplace_back(ephem_ptr);
        sat2time_index[ephem_ptr->sat].emplace(toe, sat2ephem.at(ephem_ptr->sat).size()-1);
    }
}

void Estimator::inputIonoParams(double ts, const std::vector<double> &iono_params)
{
    if (iono_params.size() != 8)    return;

    // update ionosphere parameters
    latest_gnss_iono_params.clear();
    std::copy(iono_params.begin(), iono_params.end(), std::back_inserter(latest_gnss_iono_params));
}

void Estimator::inputGNSSTimeDiff(const double t_diff)
{
    diff_t_gnss_local = t_diff;
}

void Estimator::processGNSS(const std::vector<ObsPtr> &gnss_meas)
{
    TIME_MODULE("GNSS_Process");  // <-- 添加这行

    ROS_INFO_THROTTLE(1.0, "processGNSS: input %zu measurements", gnss_meas.size());

    // 🔥 关键：在函数开头声明所有统计变量
    int filtered_by_system = 0;
    int filtered_by_no_ephem = 0;
    int filtered_by_ephem_invalid = 0;
    int filtered_by_quality = 0;
    int filtered_by_tracking = 0;
    int filtered_by_elevation = 0;
    int filtered_by_no_freq = 0;
    std::vector<ObsPtr> valid_meas;
    std::vector<EphemBasePtr> valid_ephems;

    // 打印星历状态
    if (sat2ephem.empty()) {
        ROS_WARN_THROTTLE(2.0, "No ephemeris available yet! Observations will be filtered.");
    } else {
        ROS_DEBUG_THROTTLE(2.0, "Ephemeris available for %zu satellites", sat2ephem.size());
    }

    bool strict_filtering = gnss_ready && solver_flag == NON_LINEAR;

    // 主过滤循环
    for (auto obs : gnss_meas)
    {
        // 1. 系统过滤
        uint32_t sys = satsys(obs->sat, NULL);
        if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS) {
            filtered_by_system++;
            ROS_DEBUG("Sat %d filtered: unsupported system %d", obs->sat, sys);
            continue;
        }

        // 2. 检查星历
        if (sat2ephem.count(obs->sat) == 0) {
            filtered_by_no_ephem++;
            if (!gnss_ready) {
                ROS_DEBUG_THROTTLE(1.0, "Sat %d filtered: no ephemeris (will retry)", obs->sat);
            } else {
                ROS_WARN_THROTTLE(1.0, "Sat %d filtered: no ephemeris", obs->sat);
            }
            continue;
        }
        
        // 3. 检查频率
        if (obs->freqs.empty()) {
            filtered_by_no_freq++;
            continue;
        }
        
        int freq_idx = -1;
        L1_freq(obs, &freq_idx);
        if (freq_idx < 0) {
            filtered_by_no_freq++;
            ROS_DEBUG("Sat %d filtered: no L1 frequency", obs->sat);
            continue;
        }
        
        // 4. 获取最佳星历
        double obs_time = time2sec(obs->time);
        std::map<double, size_t> time2index = sat2time_index.at(obs->sat);
        
        double ephem_time = EPH_VALID_SECONDS;
        size_t ephem_index = -1;
        for (auto ti : time2index)
        {
            if (std::abs(ti.first - obs_time) < ephem_time)
            {
                ephem_time = std::abs(ti.first - obs_time);
                ephem_index = ti.second;
            }
        }
        
        if (ephem_index == size_t(-1)) {
            filtered_by_ephem_invalid++;
            continue;
        }
        
        const EphemBasePtr &best_ephem = sat2ephem.at(obs->sat).at(ephem_index);
        
        // 5. 检查伪距存在
        if (obs->psr[freq_idx] == 0) {
            filtered_by_quality++;
            ROS_DEBUG("Sat %d filtered: no pseudorange", obs->sat);
            continue;
        }
        
        // 6. 质量检查 - 放宽阈值用于调试
        double psr_threshold = strict_filtering ? GNSS_PSR_STD_THRES : 200.0;
        double dopp_threshold = strict_filtering ? GNSS_DOPP_STD_THRES : 100.0;
        
        // 打印第一个观测的详细信息用于调试
        static bool first_obs_printed = false;
        if (!first_obs_printed && obs->psr_std[freq_idx] > 0) {
            ROS_INFO("First obs details: sat=%d, psr_std=%.2f, dopp_std=%.2f, thresholds=(%.1f, %.1f)",
                     obs->sat, obs->psr_std[freq_idx], obs->dopp_std[freq_idx], 
                     psr_threshold, dopp_threshold);
            first_obs_printed = true;
        }
        
        if (obs->psr_std[freq_idx] == 0 || obs->psr_std[freq_idx] > psr_threshold) {
            filtered_by_quality++;
            ROS_DEBUG("Sat %d filtered: psr_std=%.1f > %.1f", 
                     obs->sat, obs->psr_std[freq_idx], psr_threshold);
            sat_track_status[obs->sat] = 0;
            continue;
        }
        
        if (obs->dopp_std[freq_idx] == 0 || obs->dopp_std[freq_idx] > dopp_threshold) {
            filtered_by_quality++;
            ROS_DEBUG("Sat %d filtered: dopp_std=%.1f > %.1f", 
                     obs->sat, obs->dopp_std[freq_idx], dopp_threshold);
            sat_track_status[obs->sat] = 0;
            continue;
        }
        
        // 7. 跟踪状态检查 - 初始化时完全跳过
        if (!gnss_ready) {
            // 初始化阶段，自动满足跟踪要求
            if (sat_track_status.count(obs->sat) == 0) {
                sat_track_status[obs->sat] = GNSS_TRACK_NUM_THRES + 1;
            }
        } else {
            uint32_t required_tracks = GNSS_TRACK_NUM_THRES;
            if (sat_track_status[obs->sat] < required_tracks) {
                filtered_by_tracking++;
                ROS_DEBUG("Sat %d filtered: tracking=%d < %d", 
                         obs->sat, sat_track_status[obs->sat], required_tracks);
                continue;
            }
        }
        
        // 8. 仰角检查 - 暂时跳过
        if (gnss_ready && false) {  // 暂时禁用仰角检查
            filtered_by_elevation++;
        }
        
        // 通过所有检查，添加到有效列表
        valid_meas.push_back(obs);
        valid_ephems.push_back(best_ephem);
        sat_track_status[obs->sat]++;
        
        ROS_DEBUG("Sat %d PASSED all checks", obs->sat);
    }
    
    // 输出统计信息
    ROS_INFO("GNSS Filter Statistics:");
    ROS_INFO("  System: %d, No ephem: %d, Invalid ephem: %d", 
             filtered_by_system, filtered_by_no_ephem, filtered_by_ephem_invalid);
    ROS_INFO("  Quality: %d, Tracking: %d, Elevation: %d, No freq: %d", 
             filtered_by_quality, filtered_by_tracking, filtered_by_elevation, filtered_by_no_freq);
    ROS_INFO("  Valid: %zu/%zu observations", valid_meas.size(), gnss_meas.size());
    
    // 存储结果
    gnss_meas_buf[frame_count] = valid_meas;
    gnss_ephem_buf[frame_count] = valid_ephems;
    
    ROS_INFO("Frame %d: stored %zu valid GNSS measurements (%d filtered by no ephem)", 
             frame_count, valid_meas.size(), filtered_by_no_ephem);
    
    // 检查星历状态
    ROS_INFO("Ephemeris status: %zu satellites in sat2ephem", sat2ephem.size());
    if (!sat2ephem.empty()) {
        std::stringstream ss;
        int count = 0;
        for (const auto& pair : sat2ephem) {
            ss << pair.first << "(" << pair.second.size() << ") ";
            count++;
            if (count > 10) {  // 只显示前10个
                ss << "...";
                break;
            }
        }
        ROS_INFO("Satellites with ephemeris: %s", ss.str().c_str());
    }

    // 如果有有效观测且solver已初始化，尝试对齐
    if (solver_flag == NON_LINEAR && !gnss_ready && valid_meas.size() >= 4) {
        ROS_INFO_THROTTLE(1.0, "Have %zu valid GNSS obs, will attempt alignment", 
                         valid_meas.size());
    }
}

bool Estimator::getSecondaryPositionAtTime(double timestamp, Eigen::Vector3d& pos_ecef) const
{
    std::lock_guard<std::mutex> lock(secondary_buffer_mutex_);
    if (secondary_position_buffer_.empty()) {
        ROS_WARN_THROTTLE(2.0, "Secondary position buffer is EMPTY! No data received.");
        return false;
    }
    
    // 打印缓冲区状态（调试用）
    static int query_count = 0;
    if (++query_count % 50 == 0) {
        double min_time = secondary_position_buffer_.begin()->first;
        double max_time = secondary_position_buffer_.rbegin()->first;
        ROS_INFO("Query #%d: buffer size=%zu, time range=[%.3f, %.3f], requested=%.3f",
                 query_count, secondary_position_buffer_.size(), 
                 min_time, max_time, timestamp);
    }
    
    // 找最接近的时间戳
    auto it = secondary_position_buffer_.lower_bound(timestamp);
    if (it == secondary_position_buffer_.end()) {
        it = std::prev(it);  // 使用最后一个
    } else if (it != secondary_position_buffer_.begin()) {
        auto prev_it = std::prev(it);
        // 选择更近的
        if (std::abs(prev_it->first - timestamp) < std::abs(it->first - timestamp)) {
            it = prev_it;
        }
    }
    
    double time_diff = std::abs(it->first - timestamp);
    
    // 增加更详细的时间差报告
    if (time_diff > 1.0) {
        ROS_WARN("Secondary position too old: time_diff=%.3fs (requested=%.3f, found=%.3f)", 
                 time_diff, timestamp, it->first);
        return false;
    }
    
    pos_ecef = it->second;
    
    // 成功时偶尔打印信息
    if (query_count % 100 == 0) {
        ROS_INFO("✓ Found secondary position: time_diff=%.3fs, pos=[%.1f, %.1f, %.1f]",
                 time_diff, pos_ecef.x(), pos_ecef.y(), pos_ecef.z());
    }
    
    return true;
}

bool Estimator::initialStructure()
{
    TicToc t_sfm;
    //check imu observibility
    {
        map<double, ImageFrame>::iterator frame_it;

        Vector3d sum_g;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
            //cout << "frame g " << tmp_g.transpose() << endl;
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        //ROS_WARN("IMU variation %f!", var);
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enough!");
            //return false;
        }
    }
    // global sfm
    Quaterniond Q[frame_count + 1];
    Vector3d T[frame_count + 1];
    map<int, Vector3d> sfm_tracked_points;
    vector<SFMFeature> sfm_f;
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 
    Matrix3d relative_R;
    Vector3d relative_T;
    int l;
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess
        cv::Mat r, rvec, t, D, tmp_r;
        Matrix3d tmp_R_pnp;
        Matrix3d R_pnp;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec);
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6)
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            cv::Mat cv_R_inital;
            cv::eigen2cv(R_inital, cv_R_inital); 
            cv::cv2eigen(cv_R_inital, tmp_R_pnp);
            cv::cv2eigen(tmp_r, tmp_R_pnp);
            R_pnp = tmp_R_pnp.transpose();
            MatrixXd T_pnp;
            cv::cv2eigen(t, T_pnp);
            T_pnp = R_pnp * (-T_pnp);
            frame_it->second.R = R_pnp * RIC[0].transpose();
            frame_it->second.T = T_pnp;
        }
        else
        { 
            if(cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, false))
            {
                cv::Rodrigues(rvec, r);  // 将旋转向量转换为旋转矩阵
                cv::cv2eigen(r, tmp_R_pnp);
                R_pnp = tmp_R_pnp.transpose();
                MatrixXd T_pnp;
                cv::cv2eigen(t, T_pnp);
                T_pnp = R_pnp * (-T_pnp);
                frame_it->second.R = R_pnp * RIC[0].transpose();
                frame_it->second.T = T_pnp;
            }
            else
            {
                ROS_WARN("solve pnp fail!");
                // 使用初始猜测作为备用方案
                cv::eigen2cv(R_inital, tmp_r);
                cv::cv2eigen(tmp_r, tmp_R_pnp);
                R_pnp = tmp_R_pnp.transpose();
                MatrixXd T_pnp;
                cv::cv2eigen(t, T_pnp);
                T_pnp = R_pnp * (-T_pnp);
                frame_it->second.R = R_pnp * RIC[0].transpose();
                frame_it->second.T = T_pnp;
            }
        }
    }

    if (!visualInitialAlign())
    {
        ROS_WARN("misalign visual structure with IMU");
        return false;
    }
    
    ROS_INFO("Initial structure complete! solver_flag will be set to NON_LINEAR");
    return true;
}


bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    //solve scale
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulate on cam pose , no tic
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    double s = (x.tail<1>())(0);
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]);
    }
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3);
        }
    }
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    Matrix3d R0 = Utility::g2R(g);
    double yaw = Utility::R2ypr(R0 * Rs[0]).x();
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0;
    g = R0 * g;
    //Matrix3d rot_diff = R0 * Rs[0].transpose();
    Matrix3d rot_diff = R0;
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }

    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose());

    return true;
}

bool Estimator::GNSSVIAlign()
{
    if (solver_flag == INITIAL)     // visual-inertial not initialized
        return false;
    
    if (gnss_ready)                 // GNSS-VI already initialized
        return true;
    
    // 只检查最近的帧（星历可能延迟到达）
    const int CHECK_WINDOW = 5;  // 只检查最近5帧
    int start_idx = std::max(0, int(WINDOW_SIZE+1) - CHECK_WINDOW);
    
    int valid_frames = 0;
    int total_obs = 0;
    
    for (int i = start_idx; i <= WINDOW_SIZE; ++i) {
        if (gnss_meas_buf[i].size() >= 4) {
            valid_frames++;
            total_obs += gnss_meas_buf[i].size();
        }
    }
   
    const int MIN_VALID_FRAMES = 1;
    const int MIN_TOTAL_OBS = 4;  // 或者总观测数足够
    
    if (valid_frames < MIN_VALID_FRAMES && total_obs < MIN_TOTAL_OBS) {
        ROS_INFO_THROTTLE(1.0, "GNSS-VI align: only %d/%d recent frames (total %d obs)", 
                         valid_frames, CHECK_WINDOW, total_obs);
        return false;
    }
    
    ROS_INFO("GNSS-VI alignment check passed: %d frames, %d observations", 
             valid_frames, total_obs);

    // check horizontal velocity excitation
    Eigen::Vector2d avg_hor_vel(0.0, 0.0);
    for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
        avg_hor_vel += Vs[i].head<2>().cwiseAbs();
    avg_hor_vel /= (WINDOW_SIZE+1);
    if (avg_hor_vel.norm() < 0.3)
    {
        std::cerr << "velocity excitation not enough for GNSS-VI alignment.\n";
        return false;
    }

    // 使用最近有有效数据的帧进行初始化
    std::vector<std::vector<ObsPtr>> curr_gnss_meas_buf;
    std::vector<std::vector<EphemBasePtr>> curr_gnss_ephem_buf;
    
    for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
    {
        // 如果没有观测，用空vector占位
        curr_gnss_meas_buf.push_back(gnss_meas_buf[i]);
        curr_gnss_ephem_buf.push_back(gnss_ephem_buf[i]);
    }

    GNSSVIInitializer gnss_vi_initializer(curr_gnss_meas_buf, curr_gnss_ephem_buf, latest_gnss_iono_params);
    
    // 1. get a rough global location
    Eigen::Matrix<double, 7, 1> rough_xyzt;
    rough_xyzt.setZero();
    if (!gnss_vi_initializer.coarse_localization(rough_xyzt))
    {
        std::cerr << "Fail to obtain a coarse location.\n";
        return false;
    }

    // 2. perform yaw alignment
    std::vector<Eigen::Vector3d> local_vs;
    for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
        local_vs.push_back(Vs[i]);
    Eigen::Vector3d rough_anchor_ecef = rough_xyzt.head<3>();
    double aligned_yaw = 0;
    double aligned_rcv_ddt = 0;
    if (!gnss_vi_initializer.yaw_alignment(local_vs, rough_anchor_ecef, aligned_yaw, aligned_rcv_ddt))
    {
        std::cerr << "Fail to align ENU and local frames.\n";
        return false;
    }
    // std::cout << "aligned_yaw is " << aligned_yaw*180.0/M_PI << '\n';

    // 3. perform anchor refinement
    std::vector<Eigen::Vector3d> local_ps;
    for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
        local_ps.push_back(Ps[i]);
    Eigen::Matrix<double, 7, 1> refined_xyzt;
    refined_xyzt.setZero();
    if (!gnss_vi_initializer.anchor_refinement(local_ps, aligned_yaw, 
        aligned_rcv_ddt, rough_xyzt, refined_xyzt))
    {
        std::cerr << "Fail to refine anchor point.\n";
        return false;
    }
    // std::cout << "refined anchor point is " << std::setprecision(20) 
    //           << refined_xyzt.head<3>().transpose() << '\n';

    // restore GNSS states
    uint32_t one_observed_sys = static_cast<uint32_t>(-1);
    for (uint32_t k = 0; k < 4; ++k)
    {
        if (rough_xyzt(k+3) != 0)
        {
            one_observed_sys = k;
            break;
        }
    }
    for (uint32_t i = 0; i < (WINDOW_SIZE+1); ++i)
    {
        para_rcv_ddt[i] = aligned_rcv_ddt;
        for (uint32_t k = 0; k < 4; ++k)
        {
            if (rough_xyzt(k+3) == 0)
                para_rcv_dt[i*4+k] = refined_xyzt(3+one_observed_sys) + aligned_rcv_ddt * i;
            else
                para_rcv_dt[i*4+k] = refined_xyzt(3+k) + aligned_rcv_ddt * i;
        }
    }
    anc_ecef = refined_xyzt.head<3>();
    R_ecef_enu = ecef2rotation(anc_ecef);

    yaw_enu_local = aligned_yaw;

    return true;
}

void Estimator::updateGNSSStatistics()
{
    R_enu_local = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ());
    enu_pos = R_enu_local * Ps[WINDOW_SIZE];
    enu_vel = R_enu_local * Vs[WINDOW_SIZE];
    enu_ypr = Utility::R2ypr(R_enu_local*Rs[WINDOW_SIZE]);
    ecef_pos = anc_ecef + R_ecef_enu * enu_pos;
}


bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20)
        {
            double sum_parallax = 0;
            double average_parallax;
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm();
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric);
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization();
        
        outputResults();
        
        if (GNSS_ENABLE)
        {
            if (!gnss_ready)
            {
                static int alignment_attempts = 0;
                alignment_attempts++;
                
                if (alignment_attempts % 5 == 0) {  // 每5帧尝试一次
                    ROS_INFO("Attempting GNSS-VI alignment (attempt #%d)...", alignment_attempts);
                }
                
                gnss_ready = GNSSVIAlign();
                
                if (gnss_ready) {
                    ROS_INFO("✓ GNSS-VI alignment successful after %d attempts!", alignment_attempts);
                }
            }
            if (gnss_ready)
            {
                updateGNSSStatistics();
            }
        }
    }
}

void Estimator::vector2double()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
    
    para_yaw_enu_local[0] = yaw_enu_local;
    for (uint32_t k = 0; k < 3; ++k)
        para_anc_ecef[k] = anc_ecef(k);
}

int Estimator::findMatchingDroneIndex(double gnss_timestamp) {
    // 转换GNSS时间到本地时间
    double local_timestamp = gnss_timestamp - diff_t_gnss_local;
    
    int closest_idx = -1;
    double min_diff = std::numeric_limits<double>::max();
    
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        double time_diff = std::abs(Headers[i].stamp.toSec() - local_timestamp);
        if (time_diff < min_diff) {
            min_diff = time_diff;
            closest_idx = i;
        }
    }
    
    // 添加详细日志
    if (min_diff > DD_TIME_SYNC_THRESHOLD) {
        ROS_WARN_THROTTLE(1.0, "Time sync failed: gnss_time=%.6f, local_time=%.6f, min_diff=%.3f > threshold=%.3f", 
                         gnss_timestamp, local_timestamp, min_diff, DD_TIME_SYNC_THRESHOLD);
        return -1;
    }
    
    return closest_idx;
}

void Estimator::double2vector()
{
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {

        Rs[i] = Quaterniond(para_Pose[i][6], para_Pose[i][3], 
                            para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        
        Ps[i] = Vector3d(para_Pose[i][0], para_Pose[i][1], para_Pose[i][2]);

        Vs[i] = Vector3d(para_SpeedBias[i][0], para_SpeedBias[i][1], para_SpeedBias[i][2]);

        Bas[i] = Vector3d(para_SpeedBias[i][3], para_SpeedBias[i][4], para_SpeedBias[i][5]);

        Bgs[i] = Vector3d(para_SpeedBias[i][6], para_SpeedBias[i][7], para_SpeedBias[i][8]);
    }

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0], para_Ex_Pose[i][1], para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6], para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4], para_Ex_Pose[i][5]).normalized().toRotationMatrix();
    }

    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    if (ESTIMATE_TD)
        td = para_Td[0][0];
    
    if (gnss_ready)
    {
        yaw_enu_local = para_yaw_enu_local[0];
        for (uint32_t k = 0; k < 3; ++k)
            anc_ecef(k) = para_anc_ecef[k];
        R_ecef_enu = ecef2rotation(anc_ecef);
    }
}

bool Estimator::failureDetection()
{
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
        //return true;
    }
    
    double acc_bias_threshold = gnss_ready ? 5.0 : 2.5;  
    if (Bas[WINDOW_SIZE].norm() > acc_bias_threshold)
    {
        ROS_WARN(" big IMU acc bias estimation %.3f (threshold %.1f)", 
                 Bas[WINDOW_SIZE].norm(), acc_bias_threshold);
        return true;
    }
    
    double gyr_bias_threshold = gnss_ready ? 2.0 : 1.0;
    if (Bgs[WINDOW_SIZE].norm() > gyr_bias_threshold)
    {
        ROS_WARN(" big IMU gyr bias estimation %.3f (threshold %.1f)", 
                 Bgs[WINDOW_SIZE].norm(), gyr_bias_threshold);
        return true;
    }
    
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true; 
    }
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0;
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

void Estimator::debugGNSSStatus() {
    ROS_INFO("===== GNSS Debug Info =====");
    ROS_INFO("gnss_ready: %d", gnss_ready);
    ROS_INFO("solver_flag: %d", solver_flag);
    
    for (int i = 0; i <= WINDOW_SIZE; i++) {
        ROS_INFO("gnss_meas_buf[%d]: %zu observations", i, gnss_meas_buf[i].size());
    }
    
    ROS_INFO("sat2ephem size: %zu", sat2ephem.size());
    ROS_INFO("sat_track_status size: %zu", sat_track_status.size());
    
    if (rt_dd_processor_) {
        int total_obs, valid_dd;
        double avg_quality;
        rt_dd_processor_->getStatistics(total_obs, valid_dd, avg_quality);
        ROS_INFO("DD Processor: obs=%d, dd=%d, quality=%.2f", 
                 total_obs, valid_dd, avg_quality);
    }
}

double Estimator::computePredictedDDResidual(const RealTimeDDObs& rt_obs, int frame_idx) {
    Eigen::Vector3d pos_local = Ps[frame_idx];
    
    Eigen::Matrix3d R_enu_local = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    Eigen::Vector3d enu_pos = R_enu_local * pos_local;
    Eigen::Vector3d ecef_pos = anc_ecef + R_ecef_enu * enu_pos;
    
    double range1 = (rt_obs.sat1_pos_ecef - ecef_pos).norm();
    double range2 = (rt_obs.sat2_pos_ecef - ecef_pos).norm();
    
    double predicted_dd = range1 - range2;
    double residual = rt_obs.psr_dd_observed - predicted_dd;
    
    return residual;
}


void Estimator::optimization()
{
   // 添加调试信息
    TIME_MODULE("Optimization_Total");  // <-- 添加: 优化总时间

    static int optimization_count = 0;
    optimization_count++;
    if (optimization_count % 10 == 0) {
        ROS_INFO("Optimization #%d: gnss_ready=%d, DD observations=%zu", 
                 optimization_count, gnss_ready, dd_manager.observations.size());
    }
    
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0);
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS);
    }
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]);
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
    }

    if (gnss_ready)
    {
        problem.AddParameterBlock(para_yaw_enu_local, 1);
        Eigen::Vector2d avg_hor_vel(0.0, 0.0);
        for (uint32_t i = 0; i <= WINDOW_SIZE; ++i)
            avg_hor_vel += Vs[i].head<2>().cwiseAbs();
        avg_hor_vel /= (WINDOW_SIZE+1);
        // cerr << "avg_hor_vel is " << avg_vel << endl;
        if (avg_hor_vel.norm() < 0.3)
        {
            // std::cerr << "velocity excitation not enough, fix yaw angle.\n";
            problem.SetParameterBlockConstant(para_yaw_enu_local);
        }

        for (uint32_t i = 0; i <= WINDOW_SIZE; ++i)
        {
            if (gnss_meas_buf[i].size() < 3)
                problem.SetParameterBlockConstant(para_yaw_enu_local);
        }
        
        problem.AddParameterBlock(para_anc_ecef, 3);
        // problem.SetParameterBlockConstant(para_anc_ecef);

        for (uint32_t i = 0; i <= WINDOW_SIZE; ++i)
        {
            for (uint32_t k = 0; k < 4; ++k)
                problem.AddParameterBlock(para_rcv_dt+i*4+k, 1);
            problem.AddParameterBlock(para_rcv_ddt+i, 1);
        }
    }

    // ========== 准备阶段计时 ==========
    TIME_START(prepare);  // <-- 添加
    TicToc t_whole, t_prepare;
    vector2double();
    TIME_END(prepare, "Opt_Prepare");  // <-- 添加

    // ========== 实时双差约束处理与结果写入 ==========
    TIME_START(dd);  // <-- 添加
    if (gnss_ready && 
        frame_count >= WINDOW_SIZE &&
        ENABLE_REAL_TIME_DD && 
        rt_dd_processor_) {
        
        ROS_DEBUG_THROTTLE(1.0, "Checking for DD observations...");

        double current_time_local = Headers[WINDOW_SIZE].stamp.toSec();
        double current_time_gnss = current_time_local + diff_t_gnss_local;
        auto dd_observations = rt_dd_processor_->getAvailableDDObservations(
            current_time_gnss, 1.0);

        if (!dd_observations.empty()) {
            ROS_INFO("Found %zu DD observations to process.", dd_observations.size());
            
            int dd_added = 0;
            int dd_rejected_no_secondary_pos = 0;
            int dd_rejected_quality = 0;

            for (const auto& dd_obs : dd_observations) {
                // 时间匹配
                int frame_idx = findMatchingDroneIndex(dd_obs.timestamp);
                if (frame_idx == -1) {
                    continue;
                }

                // 质量检查
                if (dd_obs.quality_score < DD_QUALITY_THRESHOLD || 
                    dd_added >= DD_MAX_CONSTRAINTS_PER_OPTIMIZATION) {
                    dd_rejected_quality++;
                    continue;
                }

                // 关键：获取副设备位置
                Eigen::Vector3d secondary_pos_ecef;
                if (!getSecondaryPositionAtTime(dd_obs.timestamp, secondary_pos_ecef)) {
                    dd_rejected_no_secondary_pos++;
                    ROS_WARN_THROTTLE(2.0, "Cannot get secondary position at %.3f", 
                                     dd_obs.timestamp);
                    continue;
                }

                // 创建修正后的双差因子
                RealTimeDDObs rt_obs; // 创建构造函数所需的新对象
                
                // 从 dd_obs (DoubleDifferenceObs) 复制数据到 rt_obs (RealTimeDDObs)
                rt_obs.timestamp         = dd_obs.timestamp;
                rt_obs.sat1_id           = dd_obs.sat1_id;
                rt_obs.sat2_id           = dd_obs.sat2_id;
                rt_obs.sat1_pos_ecef     = dd_obs.sat1_pos_ecef;
                rt_obs.sat2_pos_ecef     = dd_obs.sat2_pos_ecef;
                
                // 关键：名称不匹配
                rt_obs.psr_dd_observed = dd_obs.psr_dd;         //
                rt_obs.quality_weight  = dd_obs.quality_score;  //
                
                // rt_obs.measurement_std 将使用 RealTimeDDObs 构造函数中
                // 定义的默认值 (10.0)，因为 DoubleDifferenceObs 
                // 中没有提供这个字段。
                // -----------------------------------------------------------------

                auto dd_factor = new GnssDDFactorRealtime(rt_obs, secondary_pos_ecef); // 使用新创建的 rt_obs
                ceres::LossFunction* dd_loss = new ceres::HuberLoss(1.0);
                
                problem.AddResidualBlock(dd_factor, dd_loss,
                                       para_Pose[frame_idx], 
                                       para_SpeedBias[frame_idx], 
                                       para_anc_ecef);
                dd_added++;
                
                ROS_INFO("Added DD: sats(%d,%d), frame=%d, quality=%.2f, "
                        "secondary_pos=[%.1f,%.1f,%.1f]", 
                        dd_obs.sat1_id, dd_obs.sat2_id, frame_idx, 
                        dd_obs.quality_score,
                        secondary_pos_ecef.x(), secondary_pos_ecef.y(), 
                        secondary_pos_ecef.z());
            }
            
            ROS_INFO("DD Summary: %d added, %d rejected (quality), %d rejected (no sec pos)", 
                    dd_added, dd_rejected_quality, dd_rejected_no_secondary_pos);
        }
    }
    TIME_END(dd, "Opt_DD_Factor");  // <-- 添加

    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }
    
    // ========== 添加：IMU因子计时 ==========
    TIME_START(imu_factor);
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0)
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]);
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    TIME_END(imu_factor, "Opt_IMU_Factor");
    
    TIME_START(gnss_factor);  // <-- 添加
    if (gnss_ready)
    {
        for(int i = 0; i <= WINDOW_SIZE; ++i)
        {
            // cerr << "size of gnss_meas_buf[" << i << "] is " << gnss_meas_buf[i].size() << endl;
            const std::vector<ObsPtr> &curr_obs = gnss_meas_buf[i];
            const std::vector<EphemBasePtr> &curr_ephem = gnss_ephem_buf[i];

            for (uint32_t j = 0; j < curr_obs.size(); ++j)
            {
                const uint32_t sys = satsys(curr_obs[j]->sat, NULL);
                const uint32_t sys_idx = gnss_comm::sys2idx.at(sys);

                int lower_idx = -1;
                const double obs_local_ts = time2sec(curr_obs[j]->time) - diff_t_gnss_local;
                if (Headers[i].stamp.toSec() > obs_local_ts)
                    lower_idx = (i==0? 0 : i-1);
                else
                    lower_idx = (i==WINDOW_SIZE? WINDOW_SIZE-1 : i);
                const double lower_ts = Headers[lower_idx].stamp.toSec();
                const double upper_ts = Headers[lower_idx+1].stamp.toSec();

                const double ts_ratio = (upper_ts-obs_local_ts) / (upper_ts-lower_ts);
                GnssPsrDoppFactor *gnss_factor = new GnssPsrDoppFactor(curr_obs[j], 
                    curr_ephem[j], latest_gnss_iono_params, ts_ratio);
                problem.AddResidualBlock(gnss_factor, NULL, para_Pose[lower_idx], 
                    para_SpeedBias[lower_idx], para_Pose[lower_idx+1], para_SpeedBias[lower_idx+1],
                    para_rcv_dt+i*4+sys_idx, para_rcv_ddt+i, para_yaw_enu_local, para_anc_ecef);
            }
        }

        // build relationship between rcv_dt and rcv_ddt
        for (size_t k = 0; k < 4; ++k)
        {
            for (uint32_t i = 0; i < WINDOW_SIZE; ++i)
            {
                const double gnss_dt = Headers[i+1].stamp.toSec() - Headers[i].stamp.toSec();
                DtDdtFactor *dt_ddt_factor = new DtDdtFactor(gnss_dt);
                problem.AddResidualBlock(dt_ddt_factor, NULL, para_rcv_dt+i*4+k, 
                    para_rcv_dt+(i+1)*4+k, para_rcv_ddt+i, para_rcv_ddt+i+1);
            }
        }

        // add rcv_ddt smooth factor
        for (int i = 0; i < WINDOW_SIZE; ++i)
        {
            DdtSmoothFactor *ddt_smooth_factor = new DdtSmoothFactor(GNSS_DDT_WEIGHT);
            problem.AddResidualBlock(ddt_smooth_factor, NULL, para_rcv_ddt+i, para_rcv_ddt+i+1);
        }
    }
    TIME_END(gnss_factor, "Opt_GNSS_Factor");  // <-- 添加
    
    TIME_START(visual);  // <-- 添加
    int f_m_cnt = 0;
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point;

        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j)
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point;
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, 
                        it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                        it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }
            else
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }
    TIME_END(visual, "Opt_Visual_Factor");  // <-- 添加

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    TIME_START(ceres);  // <-- 添加
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.max_num_iterations = NUM_ITERATIONS;
    options.use_nonmonotonic_steps = true;

    options.max_solver_time_in_seconds = (marginalization_flag == MARGIN_OLD) ? 
                                          SOLVER_TIME * 0.8 : SOLVER_TIME * 0.6; // 减少时间限制
    options.function_tolerance = 1e-6;  // 放宽收敛条件
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-8;

    // 添加数值稳定性选项
    options.check_gradients = false;  // 关闭梯度检查以提高速度
    options.gradient_check_relative_precision = 1e-4;

    ROS_DEBUG("Starting optimization");

    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    TIME_END(ceres, "Opt_Ceres_Solve");  // <-- 添加

    // 检查求解结果
    if (!summary.IsSolutionUsable()) {
        ROS_WARN("Ceres solver failed: %s", summary.FullReport().c_str());
        // 可以考虑回滚到上一状态或采用其他恢复策略
    }

    ROS_DEBUG("Solver completed: %s, cost: %e -> %e", 
            summary.termination_type == ceres::CONVERGENCE ? "CONVERGED" : "NO_CONVERGENCE",
            summary.initial_cost, summary.final_cost);
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());

    while(para_yaw_enu_local[0] > M_PI)   para_yaw_enu_local[0] -= 2.0*M_PI;
    while(para_yaw_enu_local[0] < -M_PI)  para_yaw_enu_local[0] += 2.0*M_PI;
    // std::cout << "yaw is " << para_yaw_enu_local[0]*180/M_PI << std::endl;

    double2vector();

    if (gnss_ready && solver_flag == NON_LINEAR && ENABLE_REAL_TIME_DD && rt_dd_processor_) {
        static int dd_write_count = 0;
        
        try {
            // 从rt_dd_processor获取DD观测
            double current_time_local = Headers[WINDOW_SIZE].stamp.toSec();
            double current_time_gnss = current_time_local + diff_t_gnss_local; // 🔧 修复：转换为GNSS时间
            auto dd_observations = rt_dd_processor_->getAvailableDDObservations(
                current_time_gnss, DD_TIME_SYNC_THRESHOLD);
            
            if (!dd_observations.empty()) {
                std::ofstream dd_output(GNSS_DD_RESULT_PATH, std::ios::app);
                
                if (dd_output.is_open()) {
                    int valid_written = 0;
                    
                    for (const auto& dd_obs : dd_observations) {
                        // 简单的时间匹配：使用最近一帧
                        int frame_idx = WINDOW_SIZE;
                        double time_diff = std::abs(Headers[frame_idx].stamp.toSec() - 
                                                    (dd_obs.timestamp - diff_t_gnss_local));
                        
                        // 只写入时间匹配良好的观测
                        if (time_diff < DD_TIME_SYNC_THRESHOLD) {
                            // 写入CSV：时间戳,帧索引,主agent,副agent,卫星1,卫星2,DD值,质量
                            dd_output << std::fixed << std::setprecision(9)
                                     << dd_obs.timestamp << ","
                                     << frame_idx << ","
                                     << dd_obs.master_agent_id << ","
                                     << dd_obs.secondary_agent_id << ","
                                     << dd_obs.sat1_id << ","
                                     << dd_obs.sat2_id << ","
                                     << dd_obs.psr_dd << ","
                                     << dd_obs.quality_score << std::endl;
                            
                            valid_written++;
                        }
                    }
                    
                    dd_output.flush();
                    dd_output.close();
                    
                    dd_write_count++;
                    if (valid_written > 0) {
                        ROS_INFO("DD Write #%d: %d observations written (from %zu available)", 
                                dd_write_count, valid_written, dd_observations.size());
                    }
                } else {
                    ROS_ERROR_THROTTLE(5.0, "Failed to open DD result file: %s", 
                                      GNSS_DD_RESULT_PATH.c_str());
                }
            } else {
                ROS_DEBUG_THROTTLE(2.0, "No DD observations available at time %.3f", current_time_gnss);
            }
        } catch (const std::exception& e) {
            ROS_ERROR_THROTTLE(5.0, "Exception in DD writing: %s", e.what());
        }
    }

    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD)
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double();



        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i);
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(
                last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(
                marginalization_factor, NULL, last_marginalization_parameter_blocks, drop_set);
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }
        else
        {
            std::vector<double> anchor_value;
            for (uint32_t k = 0; k < 7; ++k)
                anchor_value.push_back(para_Pose[0][k]);
            PoseAnchorFactor *pose_anchor_factor = new PoseAnchorFactor(anchor_value);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(pose_anchor_factor, 
                NULL, vector<double *>{para_Pose[0]}, vector<int>{0});
            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        if (gnss_ready)
        {
            for (uint32_t j = 0; j < gnss_meas_buf[0].size(); ++j)
            {
                const uint32_t sys = satsys(gnss_meas_buf[0][j]->sat, NULL);
                const uint32_t sys_idx = gnss_comm::sys2idx.at(sys);

                const double obs_local_ts = time2sec(gnss_meas_buf[0][j]->time) - diff_t_gnss_local;
                const double lower_ts = Headers[0].stamp.toSec();
                const double upper_ts = Headers[1].stamp.toSec();
                const double ts_ratio = (upper_ts-obs_local_ts) / (upper_ts-lower_ts);

                GnssPsrDoppFactor *gnss_factor = new GnssPsrDoppFactor(gnss_meas_buf[0][j], 
                    gnss_ephem_buf[0][j], latest_gnss_iono_params, ts_ratio);
                ResidualBlockInfo *psr_dopp_residual_block_info = new ResidualBlockInfo(gnss_factor, NULL,
                    vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], 
                        para_SpeedBias[1],para_rcv_dt+sys_idx, para_rcv_ddt, 
                        para_yaw_enu_local, para_anc_ecef},
                    vector<int>{0, 1, 4, 5});
                marginalization_info->addResidualBlockInfo(psr_dopp_residual_block_info);
            }

            const double gnss_dt = Headers[1].stamp.toSec() - Headers[0].stamp.toSec();
            for (size_t k = 0; k < 4; ++k)
            {
                DtDdtFactor *dt_ddt_factor = new DtDdtFactor(gnss_dt);
                ResidualBlockInfo *dt_ddt_residual_block_info = new ResidualBlockInfo(dt_ddt_factor, NULL,
                    vector<double *>{para_rcv_dt+k, para_rcv_dt+4+k, para_rcv_ddt, para_rcv_ddt+1}, 
                    vector<int>{0, 2});
                marginalization_info->addResidualBlockInfo(dt_ddt_residual_block_info);
            }

            // margin rcv_ddt smooth factor
            DdtSmoothFactor *ddt_smooth_factor = new DdtSmoothFactor(GNSS_DDT_WEIGHT);
            ResidualBlockInfo *ddt_smooth_residual_block_info = new ResidualBlockInfo(ddt_smooth_factor, NULL,
                    vector<double *>{para_rcv_ddt, para_rcv_ddt+1}, vector<int>{0});
            marginalization_info->addResidualBlockInfo(ddt_smooth_residual_block_info);
        }

        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0)
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, 
                            it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                            it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, 
                            loss_function, vector<double *>{para_Pose[imu_i], para_Pose[imu_j], 
                                para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, 
                            loss_function, vector<double *>{para_Pose[imu_i], para_Pose[imu_j], 
                                para_Ex_Pose[0], para_Feature[feature_index]},
                            vector<int>{0, 3});
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        TicToc t_pre_margin;
        marginalization_info->preMarginalize();
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize();
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
            for (uint32_t k = 0; k < 4; ++k)
                addr_shift[reinterpret_cast<long>(para_rcv_dt+i*4+k)] = para_rcv_dt+(i-1)*4+k;
            addr_shift[reinterpret_cast<long>(para_rcv_ddt+i)] = para_rcv_ddt+i-1;
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        addr_shift[reinterpret_cast<long>(para_yaw_enu_local)] = para_yaw_enu_local;
        addr_shift[reinterpret_cast<long>(para_anc_ecef)] = para_anc_ecef;
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i);
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue;
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                    for (uint32_t k = 0; k < 4; ++k)
                        addr_shift[reinterpret_cast<long>(para_rcv_dt+i*4+k)] = para_rcv_dt+(i-1)*4+k;
                    addr_shift[reinterpret_cast<long>(para_rcv_ddt+i)] = para_rcv_ddt+i-1;
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                    for (uint32_t k = 0; k < 4; ++k)
                        addr_shift[reinterpret_cast<long>(para_rcv_dt+i*4+k)] = para_rcv_dt+i*4+k;
                    addr_shift[reinterpret_cast<long>(para_rcv_ddt+i)] = para_rcv_ddt+i;
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            addr_shift[reinterpret_cast<long>(para_yaw_enu_local)] = para_yaw_enu_local;
            addr_shift[reinterpret_cast<long>(para_anc_ecef)] = para_anc_ecef;
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;
            
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

void Estimator::slideWindow()
{
    TIME_MODULE("Marginalization");  // <-- 添加这行

    TicToc t_margin;
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();
        back_R0 = Rs[0];
        back_P0 = Ps[0];
        if (frame_count == WINDOW_SIZE)
        {
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]);

                std::swap(pre_integrations[i], pre_integrations[i + 1]);

                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];
                Ps[i].swap(Ps[i + 1]);
                Vs[i].swap(Vs[i + 1]);
                Bas[i].swap(Bas[i + 1]);
                Bgs[i].swap(Bgs[i + 1]);

                // GNSS related
                gnss_meas_buf[i].swap(gnss_meas_buf[i+1]);
                gnss_ephem_buf[i].swap(gnss_ephem_buf[i+1]);
                for (uint32_t k = 0; k < 4; ++k)
                    para_rcv_dt[i*4+k] = para_rcv_dt[(i+1)*4+k];
                para_rcv_ddt[i] = para_rcv_ddt[i+1];
            }
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            // GNSS related
            gnss_meas_buf[WINDOW_SIZE].clear();
            gnss_ephem_buf[WINDOW_SIZE].clear();

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0;
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration;
                it_0->second.pre_integration = nullptr;
 
                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            slideWindowOld();
        }
    }
    else
    {
        if (frame_count == WINDOW_SIZE)
        {
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            // GNSS related
            gnss_meas_buf[frame_count-1] = gnss_meas_buf[frame_count];
            gnss_ephem_buf[frame_count-1] = gnss_ephem_buf[frame_count];
            for (uint32_t k = 0; k < 4; ++k)
                para_rcv_dt[(frame_count-1)*4+k] = para_rcv_dt[frame_count*4+k];
            para_rcv_ddt[frame_count-1] = para_rcv_ddt[frame_count];
            gnss_meas_buf[frame_count].clear();
            gnss_ephem_buf[frame_count].clear();

            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
void Estimator::slideWindowNew()
{
    sum_of_front++;
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
void Estimator::slideWindowOld()
{
    sum_of_back++;

    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1;
        Vector3d P0, P1;
        R0 = back_R0 * ric[0];
        R1 = Rs[0] * ric[0];
        P0 = back_P0 + back_R0 * tic[0];
        P1 = Ps[0] + Rs[0] * tic[0];
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
        f_manager.removeBack();
}

std::vector<EphemBasePtr> Estimator::getCurrentValidEphems(const std::vector<ObsPtr>& observations) {
    std::vector<EphemBasePtr> valid_ephems;
    
    for (const auto& obs : observations) {
        EphemBasePtr best_ephem = nullptr;
        if (sat2ephem.count(obs->sat) > 0) {
            double obs_time = time2sec(obs->time);
            const auto& ephem_list = sat2ephem.at(obs->sat);
            
            // 找最近的星历，增加时间有效性检查
            double min_time_diff = std::numeric_limits<double>::max();
            for (const auto& ephem : ephem_list) {
                if (ephem) {
                    double ephem_time = time2sec(ephem->toe);
                    double time_diff = std::abs(ephem_time - obs_time);
                    
                    // 检查星历有效期
                    double max_age = (satsys(obs->sat, NULL) == SYS_GLO) ? 1800.0 : 7200.0;
                    if (time_diff < min_time_diff && time_diff < max_age) {
                        min_time_diff = time_diff;
                        best_ephem = ephem;
                    }
                }
            }
        }
        
        valid_ephems.push_back(best_ephem);
    }
    
    return valid_ephems;
}

void Estimator::outputResults() 
{
    static int call_count = 0;
    call_count++;

    if (solver_flag == INITIAL && call_count % 10 != 0) {
        return;  // INITIAL阶段降低写入频率
    }
    
    // 始终写入VINS结果
    std::ofstream vins_file(VINS_RESULT_PATH, std::ios::app);
    if (vins_file.is_open()) {
        double timestamp = Headers[WINDOW_SIZE].stamp.toSec();
        Quaterniond q(Rs[WINDOW_SIZE]);
        vins_file << std::fixed << std::setprecision(9)
                  << timestamp << ","
                  << Ps[WINDOW_SIZE].x() << "," 
                  << Ps[WINDOW_SIZE].y() << ","
                  << Ps[WINDOW_SIZE].z() << ","
                  << q.w() << ","
                  << q.x() << ","
                  << q.y() << ","
                  << q.z() << ","
                  << Vs[WINDOW_SIZE].x() << ","
                  << Vs[WINDOW_SIZE].y() << ","
                  << Vs[WINDOW_SIZE].z() << std::endl;
        vins_file.close();
        
        // 定期打印确认
        if (call_count % 50 == 0) {
            ROS_INFO("Written %d VINS records to %s", call_count, VINS_RESULT_PATH.c_str());
        }
    } else {
        ROS_ERROR_THROTTLE(5.0, "Failed to open VINS result file: %s", VINS_RESULT_PATH.c_str());
    }
    
    // GNSS结果仅在ready时写入
    if (gnss_ready && GNSS_ENABLE) {
        std::ofstream gnss_file(GNSS_RESULT_PATH, std::ios::app);
        if (gnss_file.is_open()) {
            gnss_file << std::fixed << std::setprecision(15)
                      << Headers[WINDOW_SIZE].stamp.toSec() << ","
                      << ecef_pos.x() << ","
                      << ecef_pos.y() << ","
                      << ecef_pos.z() << ","
                      << enu_pos.x() << ","
                      << enu_pos.y() << ","
                      << enu_pos.z() << std::endl;
            gnss_file.close();
        }
    }
}

void Estimator::addSecondaryPosition(double timestamp, const Eigen::Vector3d& pos_ecef)
{
    std::lock_guard<std::mutex> lock(secondary_buffer_mutex_);
    secondary_position_buffer_[timestamp] = pos_ecef;
}

void Estimator::cleanSecondaryPositionBuffer(double current_time, double max_age)
{
    std::lock_guard<std::mutex> lock(secondary_buffer_mutex_);
    auto it = secondary_position_buffer_.begin();
    int cleaned_count = 0;
    while (it != secondary_position_buffer_.end()) {
        if (current_time - it->first > max_age) {
            it = secondary_position_buffer_.erase(it);
            cleaned_count++;
        } else {
            ++it;
        }
    }
    if (cleaned_count > 0) {
        ROS_INFO("Cleaned %d old secondary positions", cleaned_count);
    }
}

size_t Estimator::getSecondaryPositionBufferSize() const
{
    std::lock_guard<std::mutex> lock(secondary_buffer_mutex_);
    return secondary_position_buffer_.size();
}