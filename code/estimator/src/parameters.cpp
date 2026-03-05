#include "parameters.h"

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string FACTOR_GRAPH_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD;
double MAX_TIME_DIFF = 0.1;
bool GNSS_ENABLE;
std::string GNSS_EPHEM_TOPIC;
std::string GNSS_GLO_EPHEM_TOPIC;
std::string GNSS_MEAS_TOPIC;
std::string GNSS_IONO_PARAMS_TOPIC;
std::string GNSS_TP_INFO_TOPIC;
std::vector<double> GNSS_IONO_DEFAULT_PARAMS;
bool GNSS_LOCAL_ONLINE_SYNC;
std::string LOCAL_TRIGGER_INFO_TOPIC;
double GNSS_LOCAL_TIME_DIFF;
double GNSS_ELEVATION_THRES;
double GNSS_PSR_STD_THRES;
double GNSS_DOPP_STD_THRES;
uint32_t GNSS_TRACK_NUM_THRES;
double GNSS_DDT_WEIGHT;
std::string GNSS_RESULT_PATH;
double GNSS_DD_WEIGHT = 0.05;
std::string GNSS_DD_DATA_PATH;
std::string GNSS_DD_RESULT_PATH;

int SELF_AGENT_ID = 0;
int TARGET_AGENT_ID = 1;
bool ENABLE_REAL_TIME_DD = false;
double DD_TIME_SYNC_THRESHOLD = 0.05;
int DD_MAX_CONSTRAINTS_PER_OPTIMIZATION = 6;
double DD_QUALITY_THRESHOLD = 0.3;
double DD_MEASUREMENT_STD_DEFAULT = 20.0;
bool INTER_AGENT_COMM_ENABLE = false;
double INTER_AGENT_COMM_TIMEOUT = 0.1;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();
    }
    return ans;
}

void validateAndFixParameters() {
    ROS_INFO("========== Parameter Validation ==========");
    
    // 1. 检查双差参数
    if (GNSS_DD_WEIGHT <= 0 || GNSS_DD_WEIGHT > 100) {
        ROS_WARN("Invalid GNSS_DD_WEIGHT: %.3f, setting to default 1.0", GNSS_DD_WEIGHT);
        GNSS_DD_WEIGHT = 1.0;
    }
    
    if (DD_TIME_SYNC_THRESHOLD <= 0 || DD_TIME_SYNC_THRESHOLD > 1.0) {
        ROS_WARN("Invalid DD_TIME_SYNC_THRESHOLD: %.3f, setting to 0.05", DD_TIME_SYNC_THRESHOLD);
        DD_TIME_SYNC_THRESHOLD = 0.05;
    }
    
    if (DD_MAX_CONSTRAINTS_PER_OPTIMIZATION <= 0 || DD_MAX_CONSTRAINTS_PER_OPTIMIZATION > 50) {
        ROS_WARN("Invalid DD_MAX_CONSTRAINTS: %d, setting to 5", DD_MAX_CONSTRAINTS_PER_OPTIMIZATION);
        DD_MAX_CONSTRAINTS_PER_OPTIMIZATION = 1;
    }
    
    // 2. 检查GNSS相关参数
    if (GNSS_ENABLE) {
        if (GNSS_ELEVATION_THRES < 5.0 || GNSS_ELEVATION_THRES > 45.0) {
            ROS_WARN("Invalid elevation threshold: %.1f, setting to 15.0", GNSS_ELEVATION_THRES);
            GNSS_ELEVATION_THRES = 15.0;
        }
        
        if (GNSS_PSR_STD_THRES <= 0 || GNSS_PSR_STD_THRES > 100.0) {
            ROS_WARN("Invalid PSR std threshold: %.1f, setting to 30.0", GNSS_PSR_STD_THRES);
            GNSS_PSR_STD_THRES = 30.0;
        }
        
        if (MAX_TIME_DIFF <= 0 || MAX_TIME_DIFF > 0.5) {
            ROS_WARN("Invalid MAX_TIME_DIFF: %.3f, setting to 0.1", MAX_TIME_DIFF);
            MAX_TIME_DIFF = 0.1;
        }
    }
    
    // 3. 检查实时DD参数
    if (ENABLE_REAL_TIME_DD) {
        if (SELF_AGENT_ID < 0 || TARGET_AGENT_ID < 0 || SELF_AGENT_ID == TARGET_AGENT_ID) {
            ROS_ERROR("Invalid agent IDs: self=%d, target=%d", SELF_AGENT_ID, TARGET_AGENT_ID);
            ROS_ERROR("Setting safe defaults: self=0, target=1");
            SELF_AGENT_ID = 0;
            TARGET_AGENT_ID = 1;
        }
        
        if (DD_QUALITY_THRESHOLD <= 0 || DD_QUALITY_THRESHOLD > 1.0) {
            ROS_WARN("Invalid DD quality threshold: %.3f, setting to 0.3", DD_QUALITY_THRESHOLD);
            DD_QUALITY_THRESHOLD = 0.3;
        }
    }
    
    // 4. 检查基础SLAM参数
    if (WINDOW_SIZE < 5 || WINDOW_SIZE > 20) {
        ROS_ERROR("Invalid WINDOW_SIZE: %d (should be 5-20)", WINDOW_SIZE);
    }
    
    if (MIN_PARALLAX <= 0) {
        ROS_ERROR("Invalid MIN_PARALLAX: %.6f", MIN_PARALLAX);
    }
    
    // 5. 打印关键参数
    ROS_INFO("Key Parameters:");
    ROS_INFO("  GNSS_ENABLE: %s", GNSS_ENABLE ? "true" : "false");
    ROS_INFO("  ENABLE_REAL_TIME_DD: %s", ENABLE_REAL_TIME_DD ? "true" : "false");
    ROS_INFO("  GNSS_DD_WEIGHT: %.3f", GNSS_DD_WEIGHT);
    ROS_INFO("  DD_TIME_SYNC_THRESHOLD: %.3f", DD_TIME_SYNC_THRESHOLD);
    ROS_INFO("  SELF_AGENT_ID: %d, TARGET_AGENT_ID: %d", SELF_AGENT_ID, TARGET_AGENT_ID);
    ROS_INFO("  MAX_TIME_DIFF: %.3f", MAX_TIME_DIFF);
    ROS_INFO("========================================");
}

void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    config_file = readParam<std::string>(n, "config_file");
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    fsSettings["imu_topic"] >> IMU_TOPIC;

    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    std::string tmp_output_dir;
    fsSettings["output_dir"] >> tmp_output_dir;
    assert(!tmp_output_dir.empty() && "Output directory cannot be empty.\n");
    if (tmp_output_dir[0] == '~')
        tmp_output_dir.replace(0, 1, getenv("HOME"));
    char actual_output_dir[PATH_MAX+1];
    if(!realpath(tmp_output_dir.c_str(), actual_output_dir))
        std::cerr << "ERROR: Failed to obtain the real path of " << tmp_output_dir << '\n';
    std::string OUTPUT_DIR(actual_output_dir);
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_DIR.c_str());
    
    // 定义OUTPUT_PATH用于向后兼容
    std::string OUTPUT_PATH = OUTPUT_DIR;

    VINS_RESULT_PATH = OUTPUT_DIR + "/vins_result_no_loop.csv";
    std::ofstream fout1(VINS_RESULT_PATH, std::ios::out);
    if (fout1.is_open()) {
        fout1 << "timestamp,px,py,pz,qw,qx,qy,qz,vx,vy,vz\n";  // 添加CSV头
        fout1.close();
    }
    fout1.close();
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    FACTOR_GRAPH_RESULT_PATH = OUTPUT_DIR + "/factor_graph_result.txt";
    std::ofstream fout2(FACTOR_GRAPH_RESULT_PATH, std::ios::out);
    if (fout2.is_open()) {
        fout2 << "# timestamp feature_count gnss_ready frame_count\n";  // 添加注释头
        fout2.close();
    }
    fout2.close();

    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    G.z() = fsSettings["g_norm"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    fsSettings["gnss_dd_weight"] >> GNSS_DD_WEIGHT;
    if (GNSS_DD_WEIGHT == 0) {
        GNSS_DD_WEIGHT = 1.0; // 默认值
    }

    fsSettings["gnss_dd_result_path"] >> GNSS_DD_RESULT_PATH;
    if (GNSS_DD_RESULT_PATH.empty()) {
        GNSS_DD_RESULT_PATH = OUTPUT_DIR + "/gnss_dd_result.csv";
    }

    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_DIR + "/extrinsic_parameter.csv";

    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_DIR + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
        
    } 

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    int gnss_enable_value = fsSettings["gnss_enable"];
    GNSS_ENABLE = (gnss_enable_value == 0 ? false : true);

    if (GNSS_ENABLE)
    {
        fsSettings["gnss_ephem_topic"] >> GNSS_EPHEM_TOPIC;
        fsSettings["gnss_glo_ephem_topic"] >> GNSS_GLO_EPHEM_TOPIC;
        fsSettings["gnss_meas_topic"] >> GNSS_MEAS_TOPIC;
        fsSettings["gnss_iono_params_topic"] >> GNSS_IONO_PARAMS_TOPIC;
        fsSettings["gnss_dd_weight"] >> GNSS_DD_WEIGHT;
        cv::Mat cv_iono;
        fsSettings["gnss_iono_default_parameters"] >> cv_iono;
        Eigen::Matrix<double, 1, 8> eigen_iono;
        cv::cv2eigen(cv_iono, eigen_iono);
        for (uint32_t i = 0; i < 8; ++i)
            GNSS_IONO_DEFAULT_PARAMS.push_back(eigen_iono(0, i));
        
        fsSettings["gnss_tp_info_topic"] >> GNSS_TP_INFO_TOPIC;
        int gnss_local_online_sync_value = fsSettings["gnss_local_online_sync"];
        GNSS_LOCAL_ONLINE_SYNC = (gnss_local_online_sync_value == 0 ? false : true);
        if (!GNSS_LOCAL_ONLINE_SYNC) {
            // 如果不使用在线同步，直接设置时间差为有效
            time_diff_valid = true;
            ROS_INFO("Using fixed GNSS-local time difference: %f", GNSS_LOCAL_TIME_DIFF);
        }
        if (GNSS_LOCAL_ONLINE_SYNC)
            fsSettings["local_trigger_info_topic"] >> LOCAL_TRIGGER_INFO_TOPIC;
        else
            GNSS_LOCAL_TIME_DIFF = fsSettings["gnss_local_time_diff"];

        GNSS_ELEVATION_THRES = fsSettings["gnss_elevation_thres"];
        const double gnss_ddt_sigma = fsSettings["gnss_ddt_sigma"];
        GNSS_PSR_STD_THRES = fsSettings["gnss_psr_std_thres"];
        GNSS_DOPP_STD_THRES = fsSettings["gnss_dopp_std_thres"];
        const double track_thres = fsSettings["gnss_track_num_thres"];
        GNSS_TRACK_NUM_THRES = static_cast<uint32_t>(track_thres);
        GNSS_DDT_WEIGHT = 1.0 / gnss_ddt_sigma;
        GNSS_RESULT_PATH = OUTPUT_DIR + "/gnss_result.csv";
        GNSS_DD_RESULT_PATH = OUTPUT_DIR + "/gnss_dd_result.csv";
        std::ofstream gnss_output(GNSS_RESULT_PATH, std::ios::out);
        if (gnss_output.is_open()) {
            gnss_output << "timestamp,ecef_x,ecef_y,ecef_z,enu_e,enu_n,enu_u\n";  // 添加CSV头
            gnss_output.close();
        }
        
        std::ofstream dd_output(GNSS_DD_RESULT_PATH, std::ios::out);
        if (dd_output.is_open()) {
            dd_output << "timestamp,frame_idx,sat1_id,sat2_id,psr_dd_obs,psr_dd_pred,residual,weight\n";  // 添加CSV头
            dd_output.close();
        }
        
        ROS_INFO_STREAM("GNSS enabled");
        ROS_INFO_STREAM("GNSS DD enabled with weight: " << GNSS_DD_WEIGHT);
        MAX_TIME_DIFF = fsSettings["max_time_diff"];
        if (MAX_TIME_DIFF <= 0) MAX_TIME_DIFF = 0.1;
    }

    // 在此处添加双差相关参数
    n.param<double>("gnss_dd_weight", GNSS_DD_WEIGHT, 0.3);
    n.param<std::string>("gnss_dd_data_path", GNSS_DD_DATA_PATH, "");
    // n.param<std::string>("gnss_dd_result_path", GNSS_DD_RESULT_PATH, "");
    
    // 可以添加一个日志，显示加载的参数
    ROS_INFO("GNSS Double Difference params: weight=%.2f, data_path=%s, result_path=%s", 
             GNSS_DD_WEIGHT, GNSS_DD_DATA_PATH.c_str(), GNSS_DD_RESULT_PATH.c_str());

    n.param<int>("agent_id", SELF_AGENT_ID, 0);

    int enable_rt_dd = fsSettings["enable_real_time_dd"];
    ENABLE_REAL_TIME_DD = (enable_rt_dd != 0);

    if (ENABLE_REAL_TIME_DD) {
        DD_TIME_SYNC_THRESHOLD = fsSettings["dd_time_sync_threshold"];
        DD_MAX_CONSTRAINTS_PER_OPTIMIZATION = fsSettings["dd_max_constraints_per_optimization"];
        DD_QUALITY_THRESHOLD = fsSettings["dd_quality_threshold"];
        DD_MEASUREMENT_STD_DEFAULT = fsSettings["dd_measurement_std_default"];
        
        ROS_INFO("Real-time DD enabled for agent %d", SELF_AGENT_ID);
        ROS_INFO("DD time sync threshold: %.3f", DD_TIME_SYNC_THRESHOLD);
    }

    int comm_enable = fsSettings["inter_agent_comm_enable"];
    INTER_AGENT_COMM_ENABLE = (comm_enable != 0);
    INTER_AGENT_COMM_TIMEOUT = fsSettings["inter_agent_comm_timeout"];
    
    fsSettings.release();
    validateAndFixParameters();
}

