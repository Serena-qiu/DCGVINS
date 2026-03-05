#pragma once
#include <limits>           
#include <cmath>            
#include <typeinfo>
#include <mutex>
#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include "initial/gnss_vi_initializer.h"
#include <std_msgs/Header.h>
#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/gnss_psr_dopp_factor.hpp"
#include "factor/gnss_dt_ddt_factor.hpp"
#include "factor/gnss_dt_anchor_factor.hpp"
#include "factor/gnss_ddt_smooth_factor.hpp"
#include "factor/pos_vel_factor.hpp"
#include "factor/pose_anchor_factor.h"
#include <opencv2/core/eigen.hpp>
#include "utility/debug_logger.hpp"
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_spp.hpp>
#include "factor/gnss_dd_manager.hpp"
#include "real_time_dd_processor.hpp"
#include "gnss_bias_corrector.hpp"  // +++++ 新增：实时GNSS校正器 +++++

using namespace gnss_comm;

// 前置声明
class RealTimeDDProcessor;
class FaultToleranceManager; 
class TimeSynchronizer;
class DDConstraintCache;

class Estimator
{
  public:
    // 嵌套类定义（放在public部分前面，确保可见性）
    class DDConstraintCache {
    public:
        struct CachedConstraint {
            double timestamp;
            int frame_idx;
            void* factor;  // 使用void*避免循环依赖
            double quality;
            bool used;
        };
        
        void updateCache(double window_start, double window_end,
                        const std::vector<ObsData>& observations);
        std::vector<CachedConstraint*> getValidConstraints(double current_time);
        void clearOldCache(double current_time);
        
    private:
        std::map<double, std::vector<CachedConstraint>> cache_;
        std::mutex cache_mutex_;
    };

    class TimeSynchronizer {
    public:
        enum class SyncStatus { SUCCESS, FAILED, NO_DATA };
        struct SyncResult {
            bool is_synchronized;
            double time_offset;
            double sync_quality;
            
            SyncResult() : is_synchronized(false), time_offset(0.0), sync_quality(0.0) {}
        };
        
        TimeSynchronizer() = default;
        
        SyncResult synchronizeByCommonView(
            const std::vector<ObsPtr>& obs1,
            const std::vector<ObsPtr>& obs2,
            const std::vector<EphemBasePtr>& ephems);

    private:
        struct ClockFilter {
            double clock_offset;
            double clock_drift;
            Eigen::Matrix2d P;
            
            ClockFilter() : clock_offset(0.0), clock_drift(0.0) {
                P.setIdentity();
            }
            
            void predict(double dt);
            void update(double measured_offset, double measurement_var);
        };
        
        std::map<std::pair<int,int>, ClockFilter> clock_filters_;
    };

    class FaultToleranceManager {
    public:
        enum ConnectionState { CONNECTED, DEGRADED, DISCONNECTED };
        
        struct AgentStatus {
            int agent_id;
            ConnectionState state;
            double last_message_time;
            double weight_factor;
            int missed_messages;
        };
        
        FaultToleranceManager() = default;
        
        void updateAgentStatus(int agent_id, double current_time);
        double getConstraintWeight(int agent_id) const;
        void handleReconnection(int agent_id);
        ConnectionState getAgentState(int agent_id) const;
        std::vector<int> getConnectedAgents() const;
        void printStatus() const;
        
    private:
        std::map<int, AgentStatus> agent_statuses_;
        const double TIMEOUT_DEGRADED = 0.5;
        const double TIMEOUT_DISCONNECT = 2.0;
        const double WEIGHT_DECAY_RATE = 0.9;
    };
     

    Estimator();
    ~Estimator();  // 只声明，不定义

    void setParameter();
    void debugGNSSStatus();
    
    // interface
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    void processGNSS(const std::vector<ObsPtr> &gnss_mea);
    void inputEphem(EphemBasePtr ephem_ptr);
    void inputIonoParams(double ts, const std::vector<double> &iono_params);
    void inputGNSSTimeDiff(const double t_diff);
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);

    // internal
    void clearState();
    bool initialStructure();
    bool visualInitialAlign();
    
    // GNSS related
    bool GNSSVIAlign();
    int findMatchingDroneIndex(double timestamp);
    void updateGNSSStatistics();
    void outputResults(); 
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    void slideWindow();
    void solveOdometry();
    void slideWindowNew();
    void slideWindowOld();
    void optimization();
    void vector2double();
    void double2vector();
    bool failureDetection();
    void addGnssDDFactor(int frame_count, const ObsData &obs1, const ObsData &obs2);
    bool getSecondaryPositionAtTime(double timestamp, Eigen::Vector3d& pos_ecef) const;

    // +++++ 新增：GNSS实时校正相关接口 +++++
    
    /**
     * @brief 启用/禁用GNSS实时校正
     */
    void enableGNSSCorrection(bool enable);
    
    /**
     * @brief 配置GNSS校正参数
     */
    void configureGNSSCorrector(const GNSSBiasCorrector::CorrectionParams& params);
    
    /**
     * @brief 获取GNSS校正统计信息
     */
    GNSSBiasCorrector::CorrectionStatistics getGNSSCorrectionStats() const;
    
    /**
     * @brief 重置GNSS校正器
     */
    void resetGNSSCorrector();
    
    /**
     * @brief 输入RTK真值用于在线学习（可选）
     * @param rtk_pos_ecef RTK真值位置（ECEF坐标）
     * @param timestamp 时间戳
     */
    void inputRTKGroundTruth(const Eigen::Vector3d& rtk_pos_ecef, double timestamp);

    // 实时DD处理器
    void setRealTimeDDProcessor(RealTimeDDProcessor* processor) {
        rt_dd_processor_ = processor;
    }
    
    RealTimeDDProcessor* getRealTimeDDProcessor() const {
        return rt_dd_processor_;
    }
    
    /**
     * @brief [新函数] 向副设备位置缓存中添加一个新条目
     */
    void addSecondaryPosition(double timestamp, const Eigen::Vector3d& pos_ecef);

    /**
     * @brief [新函数] 清理早于 (current_time - max_age) 的缓存
     */
    void cleanSecondaryPositionBuffer(double current_time, double max_age = 10.0);

    /**
     * @brief [新函数] 获取缓存大小
     */
    size_t getSecondaryPositionBufferSize() const;
   
    // 获取当前有效星历
    std::vector<EphemBasePtr> getCurrentValidEphems(const std::vector<ObsPtr>& observations);
    
    GnssDDManager dd_manager;
    FeatureManager f_manager;
    
    enum SolverFlag
    {
        INITIAL,
        NON_LINEAR
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,
        MARGIN_SECOND_NEW = 1
    };

    SolverFlag solver_flag;
    MarginalizationFlag marginalization_flag;
    Vector3d g;

    Matrix3d ric[NUM_OF_CAM];
    Vector3d tic[NUM_OF_CAM];

    Vector3d Ps[(WINDOW_SIZE + 1)];
    Vector3d Vs[(WINDOW_SIZE + 1)];
    Matrix3d Rs[(WINDOW_SIZE + 1)];
    Vector3d Bas[(WINDOW_SIZE + 1)];
    Vector3d Bgs[(WINDOW_SIZE + 1)];
    double td;

    Matrix3d back_R0, last_R, last_R0;
    Vector3d back_P0, last_P, last_P0;
    std_msgs::Header Headers[(WINDOW_SIZE + 1)];

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)];
    Vector3d acc_0, gyr_0;

    vector<double> dt_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)];
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];

    // GNSS related
    bool gnss_ready;
    Eigen::Vector3d anc_ecef;
    Eigen::Matrix3d R_ecef_enu;
    double yaw_enu_local;
    std::vector<ObsPtr> gnss_meas_buf[(WINDOW_SIZE+1)];
    std::vector<EphemBasePtr> gnss_ephem_buf[(WINDOW_SIZE+1)];
    std::vector<double> latest_gnss_iono_params;
    std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem;
    std::map<uint32_t, std::map<double, size_t>> sat2time_index;
    std::map<uint32_t, uint32_t> sat_track_status;
    double para_anc_ecef[3];
    double para_yaw_enu_local[1];
    double para_rcv_dt[(WINDOW_SIZE+1)*4];
    double para_rcv_ddt[WINDOW_SIZE+1];
    
    // GNSS statistics
    double diff_t_gnss_local;
    Eigen::Matrix3d R_enu_local;
    Eigen::Vector3d ecef_pos, enu_pos, enu_vel, enu_ypr;

    int frame_count;
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid;

    MotionEstimator m_estimator;
    InitialEXRotation initial_ex_rotation;

    bool first_imu;
    bool is_valid, is_key;
    bool failure_occur;

    vector<Vector3d> point_cloud;
    vector<Vector3d> margin_cloud;
    vector<Vector3d> key_poses;
    double initial_timestamp;

    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS];
    double para_Feature[NUM_OF_F][SIZE_FEATURE];
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];
    double para_Td[1][1];

    MarginalizationInfo *last_marginalization_info;
    vector<double *> last_marginalization_parameter_blocks;

    map<double, ImageFrame> all_image_frame;
    IntegrationBase *tmp_pre_integration;

    bool first_optimization;
    
  private:
    RealTimeDDProcessor* rt_dd_processor_ = nullptr;
    DebugLogger debug_logger_;
    mutable std::mutex secondary_buffer_mutex_;
    std::map<double, Eigen::Vector3d> secondary_position_buffer_;  // 缓存副设备位置

    std::unique_ptr<FaultToleranceManager> fault_tolerance_mgr_;
    std::unique_ptr<TimeSynchronizer> time_synchronizer_;
    std::unique_ptr<DDConstraintCache> dd_cache_;
    
    // +++++ 新增：GNSS实时校正器 +++++
    std::unique_ptr<GNSSBiasCorrector> gnss_corrector_;
    bool enable_gnss_correction_ = true;  // 默认启用
    mutable std::mutex gnss_corrector_mutex_;
    
    // 存储原始GNSS位置用于对比
    struct GNSSRawData {
        double timestamp;
        Eigen::Vector3d raw_pos_ecef;
        Eigen::Vector3d corrected_pos_ecef;
    };
    std::deque<GNSSRawData> gnss_correction_log_;
    const size_t MAX_CORRECTION_LOG = 500;
    
    double computePredictedDDResidual(const RealTimeDDObs& rt_obs, int frame_idx);
    
    struct TimeMatchingStats {
        int total_attempts = 0;
        int successful_matches = 0;
        double avg_time_diff = 0.0;
        double max_time_diff = 0.0;
        std::vector<double> recent_diffs;
    } time_matching_stats_;
   
    bool enable_dd_time_debug_ = true;
};