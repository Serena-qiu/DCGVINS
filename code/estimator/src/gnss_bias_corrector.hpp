#ifndef GNSS_BIAS_CORRECTOR_HPP
#define GNSS_BIAS_CORRECTOR_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <gnss_comm/gnss_utility.hpp>
#include "utility/debug_logger.hpp"

using namespace gnss_comm;

/**
 * @brief 实时GNSS偏差校正器
 * 
 * 功能：
 * 1. 静态偏差估计与校正
 * 2. 时变UP方向偏差校正（分段多项式拟合）
 * 3. 增强卡尔曼滤波（针对UP方向）
 * 4. 移动平均平滑
 * 5. 离群点检测与处理
 */
class GNSSBiasCorrector {
public:
    struct CorrectionParams {
        // 静态偏差
        Eigen::Vector3d static_bias_enu;
        
        // 时变偏差参数
        bool enable_time_variant_correction;
        int polynomial_degree;
        int num_segments;
        
        // 卡尔曼滤波参数
        bool enable_kalman_filter;
        double process_variance;      // UP方向过程噪声
        double measurement_variance;  // UP方向测量噪声
        
        // 平滑参数
        bool enable_smoothing;
        int smoothing_window_size;
        
        // 离群点检测
        double outlier_threshold_sigma;
        
        CorrectionParams() 
            : static_bias_enu(Eigen::Vector3d::Zero()),
              enable_time_variant_correction(true),
              polynomial_degree(2),
              num_segments(3),
              enable_kalman_filter(true),
              process_variance(0.002),
              measurement_variance(0.3),
              enable_smoothing(true),
              smoothing_window_size(11),
              outlier_threshold_sigma(2.5) {}
    };
    
    struct CorrectionStatistics {
        // 校正前后RMS
        double rms_before_horizontal;
        double rms_before_vertical;
        double rms_after_horizontal;
        double rms_after_vertical;
        
        // 改善率
        double improvement_horizontal;
        double improvement_vertical;
        
        // 离群点统计
        int total_points;
        int outliers_detected;
        double outlier_rate;
        
        // 校正质量评估
        enum QualityLevel { EXCELLENT, GOOD, MODERATE, POOR };
        QualityLevel quality_level;
        
        CorrectionStatistics() 
            : rms_before_horizontal(0), rms_before_vertical(0),
              rms_after_horizontal(0), rms_after_vertical(0),
              improvement_horizontal(0), improvement_vertical(0),
              total_points(0), outliers_detected(0), outlier_rate(0),
              quality_level(POOR) {}
    };
    
    /**
     * @brief 构造函数
     */
    explicit GNSSBiasCorrector(const CorrectionParams& params = CorrectionParams());
    
    /**
     * @brief 设置ENU坐标系原点
     */
    void setENUOrigin(const Eigen::Vector3d& ecef_origin);
    
    /**
     * @brief 实时校正GNSS位置（主接口）
     * @param pos_ecef 输入ECEF位置
     * @param timestamp 时间戳
     * @return 校正后的ECEF位置
     */
    Eigen::Vector3d correctPosition(const Eigen::Vector3d& pos_ecef, double timestamp);
    
    /**
     * @brief 使用RTK真值更新校正模型（在线学习）
     * @param gnss_pos_ecef GNSS原始位置
     * @param rtk_pos_ecef RTK真值位置
     * @param timestamp 时间戳
     */
    void updateWithGroundTruth(const Eigen::Vector3d& gnss_pos_ecef,
                               const Eigen::Vector3d& rtk_pos_ecef,
                               double timestamp);
    
    /**
     * @brief 获取当前校正统计信息
     */
    CorrectionStatistics getStatistics() const;
    
    /**
     * @brief 重置校正器状态
     */
    void reset();
    
    /**
     * @brief 检查校正器是否已初始化
     */
    bool isInitialized() const { return is_initialized_; }
    
    /**
     * @brief 设置调试日志
     */
    void enableDebugLog(bool enable) { enable_debug_ = enable; }

private:
    // 1D卡尔曼滤波器（用于UP方向）
    class KalmanFilter1D {
    public:
        KalmanFilter1D(double process_var, double measurement_var)
            : process_variance_(process_var),
              measurement_variance_(measurement_var),
              state_(0.0),
              error_covariance_(1.0) {}
        
        double update(double measurement) {
            // 预测
            double predicted_state = state_;
            double predicted_cov = error_covariance_ + process_variance_;
            
            // 更新
            double kalman_gain = predicted_cov / (predicted_cov + measurement_variance_);
            state_ = predicted_state + kalman_gain * (measurement - predicted_state);
            error_covariance_ = (1.0 - kalman_gain) * predicted_cov;
            
            return state_;
        }
        
        void reset() {
            state_ = 0.0;
            error_covariance_ = 1.0;
        }
        
        double getState() const { return state_; }
        
    private:
        double process_variance_;
        double measurement_variance_;
        double state_;
        double error_covariance_;
    };
    
    // 时变偏差建模（分段多项式）
    struct TimeVariantBiasModel {
        struct Segment {
            double time_start;
            double time_end;
            std::vector<double> poly_coeffs;  // 从高次到低次
        };
        
        std::vector<Segment> segments;
        
        double evaluate(double time) const {
            for (const auto& seg : segments) {
                if (time >= seg.time_start && time <= seg.time_end) {
                    double result = 0.0;
                    double t_normalized = (time - seg.time_start) / 
                                         (seg.time_end - seg.time_start);
                    double t_power = 1.0;
                    
                    for (int i = seg.poly_coeffs.size() - 1; i >= 0; --i) {
                        result += seg.poly_coeffs[i] * t_power;
                        t_power *= t_normalized;
                    }
                    return result;
                }
            }
            return 0.0;
        }
    };
    
    // 观测数据缓存
    struct ObservationData {
        double timestamp;
        Eigen::Vector3d pos_enu;
        Eigen::Vector3d error_enu;  // 如果有真值
        bool has_ground_truth;
        bool is_outlier;
    };
    
    // 内部方法
    Eigen::Vector3d ecefToENU(const Eigen::Vector3d& pos_ecef) const;
    Eigen::Vector3d enuToECEF(const Eigen::Vector3d& pos_enu) const;
    
    void estimateStaticBias();
    void buildTimeVariantModel();
    void detectOutliers();
    double computeMovingAverage(const std::deque<double>& values) const;
    void updateStatistics();
    
    // 成员变量
    CorrectionParams params_;
    
    // ENU坐标系
    bool enu_origin_set_;
    Eigen::Vector3d ecef_origin_;
    Eigen::Matrix3d R_ecef_enu_;
    
    // 校正模型
    Eigen::Vector3d static_bias_enu_;
    TimeVariantBiasModel time_variant_model_up_;
    std::unique_ptr<KalmanFilter1D> kalman_filter_up_;
    
    // 数据缓冲
    std::deque<ObservationData> observation_buffer_;
    std::deque<double> up_smoothing_buffer_;
    const size_t MAX_BUFFER_SIZE = 1000;
    
    // 状态
    bool is_initialized_;
    bool static_bias_estimated_;
    double first_timestamp_;
    double last_timestamp_;
    
    // 统计信息
    mutable CorrectionStatistics statistics_;
    
    // 线程安全
    mutable std::mutex mutex_;
    
    // 调试
    bool enable_debug_;
    DebugLogger debug_logger_;
};

// ECEF到LLA转换（高精度）
inline Eigen::Vector3d ecef2lla(const Eigen::Vector3d& ecef) {
    const double a = 6378137.0;           // WGS84半长轴
    const double b = 6356752.314245179;   // WGS84半短轴
    const double e2 = (a*a - b*b) / (a*a);
    
    double x = ecef.x();
    double y = ecef.y();
    double z = ecef.z();
    
    double lon = std::atan2(y, x);
    double p = std::sqrt(x*x + y*y);
    double lat = std::atan2(z, p * (1.0 - e2));
    
    // 迭代求解纬度
    for (int i = 0; i < 10; ++i) {
        double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
        double lat_new = std::atan2(z + e2 * N * std::sin(lat), p);
        if (std::abs(lat_new - lat) < 1e-14) break;
        lat = lat_new;
    }
    
    double N = a / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    double alt = p / std::cos(lat) - N;
    
    return Eigen::Vector3d(lat, lon, alt);
}

// ENU旋转矩阵计算
inline Eigen::Matrix3d computeENURotation(double lat, double lon) {
    double sin_lat = std::sin(lat);
    double cos_lat = std::cos(lat);
    double sin_lon = std::sin(lon);
    double cos_lon = std::cos(lon);
    
    Eigen::Matrix3d R;
    R << -sin_lon,           cos_lon,          0,
         -sin_lat*cos_lon,  -sin_lat*sin_lon,  cos_lat,
          cos_lat*cos_lon,   cos_lat*sin_lon,  sin_lat;
    
    return R;
}

#endif // GNSS_BIAS_CORRECTOR_HPP