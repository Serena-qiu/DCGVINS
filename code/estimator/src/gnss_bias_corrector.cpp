#include "gnss_bias_corrector.hpp"
#include <algorithm>
#include <numeric>
#include <ros/ros.h>

GNSSBiasCorrector::GNSSBiasCorrector(const CorrectionParams& params)
    : params_(params),
      enu_origin_set_(false),
      ecef_origin_(Eigen::Vector3d::Zero()),
      R_ecef_enu_(Eigen::Matrix3d::Identity()),
      static_bias_enu_(Eigen::Vector3d::Zero()),
      is_initialized_(false),
      static_bias_estimated_(false),
      first_timestamp_(0.0),
      last_timestamp_(0.0),
      enable_debug_(false),
      debug_logger_("GNSSBiasCorrector")
{
    if (params_.enable_kalman_filter) {
        kalman_filter_up_ = std::make_unique<KalmanFilter1D>(
            params_.process_variance,
            params_.measurement_variance
        );
    }
}

void GNSSBiasCorrector::setENUOrigin(const Eigen::Vector3d& ecef_origin) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    ecef_origin_ = ecef_origin;
    
    // 计算ENU旋转矩阵
    Eigen::Vector3d lla = ecef2lla(ecef_origin);
    R_ecef_enu_ = computeENURotation(lla.x(), lla.y());
    
    enu_origin_set_ = true;
    
    if (enable_debug_) {
        ROS_INFO("[GNSSBiasCorrector] ENU origin set: ECEF=(%.2f, %.2f, %.2f), LLA=(%.8f, %.8f, %.2f)",
                 ecef_origin.x(), ecef_origin.y(), ecef_origin.z(),
                 lla.x() * 180.0 / M_PI, lla.y() * 180.0 / M_PI, lla.z());
    }
}

Eigen::Vector3d GNSSBiasCorrector::ecefToENU(const Eigen::Vector3d& pos_ecef) const {
    Eigen::Vector3d delta_ecef = pos_ecef - ecef_origin_;
    return R_ecef_enu_ * delta_ecef;
}

Eigen::Vector3d GNSSBiasCorrector::enuToECEF(const Eigen::Vector3d& pos_enu) const {
    Eigen::Vector3d delta_ecef = R_ecef_enu_.transpose() * pos_enu;
    return ecef_origin_ + delta_ecef;
}

Eigen::Vector3d GNSSBiasCorrector::correctPosition(
    const Eigen::Vector3d& pos_ecef, 
    double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!enu_origin_set_) {
        ROS_WARN_THROTTLE(1.0, "[GNSSBiasCorrector] ENU origin not set, returning original position");
        return pos_ecef;
    }
    
    // 转换到ENU
    Eigen::Vector3d pos_enu = ecefToENU(pos_ecef);
    
    // 1. 应用静态偏差校正
    Eigen::Vector3d corrected_enu = pos_enu;
    if (static_bias_estimated_) {
        corrected_enu -= static_bias_enu_;
    }
    
    // 2. 应用时变UP偏差校正
    if (params_.enable_time_variant_correction && !time_variant_model_up_.segments.empty()) {
        double time_variant_bias = time_variant_model_up_.evaluate(timestamp);
        corrected_enu.z() -= time_variant_bias;
    }
    
    // 3. 卡尔曼滤波（仅UP方向）
    if (params_.enable_kalman_filter && kalman_filter_up_) {
        corrected_enu.z() = kalman_filter_up_->update(corrected_enu.z());
    }
    
    // 4. 移动平均平滑（仅UP方向）
    if (params_.enable_smoothing) {
        up_smoothing_buffer_.push_back(corrected_enu.z());
        if (up_smoothing_buffer_.size() > params_.smoothing_window_size) {
            up_smoothing_buffer_.pop_front();
        }
        
        if (up_smoothing_buffer_.size() >= params_.smoothing_window_size) {
            corrected_enu.z() = computeMovingAverage(up_smoothing_buffer_);
        }
    }
    
    // 转换回ECEF
    return enuToECEF(corrected_enu);
}

void GNSSBiasCorrector::updateWithGroundTruth(
    const Eigen::Vector3d& gnss_pos_ecef,
    const Eigen::Vector3d& rtk_pos_ecef,
    double timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!enu_origin_set_) {
        ROS_WARN_ONCE("[GNSSBiasCorrector] Cannot update without ENU origin");
        return;
    }
    
    // 转换到ENU
    Eigen::Vector3d gnss_enu = ecefToENU(gnss_pos_ecef);
    Eigen::Vector3d rtk_enu = ecefToENU(rtk_pos_ecef);
    Eigen::Vector3d error_enu = gnss_enu - rtk_enu;
    
    // 保存观测
    ObservationData obs;
    obs.timestamp = timestamp;
    obs.pos_enu = gnss_enu;
    obs.error_enu = error_enu;
    obs.has_ground_truth = true;
    obs.is_outlier = false;
    
    observation_buffer_.push_back(obs);
    
    // 限制缓冲区大小
    if (observation_buffer_.size() > MAX_BUFFER_SIZE) {
        observation_buffer_.pop_front();
    }
    
    // 记录时间范围
    if (!is_initialized_) {
        first_timestamp_ = timestamp;
        is_initialized_ = true;
    }
    last_timestamp_ = timestamp;
    
    // 检测离群点
    if (observation_buffer_.size() >= 50) {
        detectOutliers();
    }
    
    // 估计静态偏差（需要足够数据）
    if (!static_bias_estimated_ && observation_buffer_.size() >= 100) {
        estimateStaticBias();
        static_bias_estimated_ = true;
        
        if (enable_debug_) {
            ROS_INFO("[GNSSBiasCorrector] Static bias estimated: E=%.3f, N=%.3f, U=%.3f",
                     static_bias_enu_.x(), static_bias_enu_.y(), static_bias_enu_.z());
        }
    }
    
    // 建立时变模型（需要更多数据和已估计静态偏差）
    if (static_bias_estimated_ && 
        params_.enable_time_variant_correction &&
        observation_buffer_.size() >= 300) {
        buildTimeVariantModel();
    }
    
    // 更新统计信息
    if (observation_buffer_.size() % 50 == 0) {
        updateStatistics();
    }
}

void GNSSBiasCorrector::estimateStaticBias() {
    if (observation_buffer_.empty()) return;
    
    Eigen::Vector3d sum_error = Eigen::Vector3d::Zero();
    int count = 0;
    
    for (const auto& obs : observation_buffer_) {
        if (obs.has_ground_truth && !obs.is_outlier) {
            sum_error += obs.error_enu;
            count++;
        }
    }
    
    if (count > 0) {
        static_bias_enu_ = sum_error / count;
        params_.static_bias_enu = static_bias_enu_;
    }
}

void GNSSBiasCorrector::buildTimeVariantModel() {
    // 收集校正后的UP误差
    std::vector<double> times;
    std::vector<double> up_errors;
    
    for (const auto& obs : observation_buffer_) {
        if (obs.has_ground_truth && !obs.is_outlier) {
            times.push_back(obs.timestamp);
            // 减去静态偏差后的残差
            double residual_up = obs.error_enu.z() - static_bias_enu_.z();
            up_errors.push_back(residual_up);
        }
    }
    
    if (times.size() < 50) return;
    
    // 分段拟合
    time_variant_model_up_.segments.clear();
    
    double time_span = last_timestamp_ - first_timestamp_;
    double segment_duration = time_span / params_.num_segments;
    
    for (int seg = 0; seg < params_.num_segments; ++seg) {
        double seg_start = first_timestamp_ + seg * segment_duration;
        double seg_end = (seg == params_.num_segments - 1) ? 
                         last_timestamp_ : seg_start + segment_duration;
        
        // 提取该段数据
        std::vector<double> seg_times;
        std::vector<double> seg_errors;
        
        for (size_t i = 0; i < times.size(); ++i) {
            if (times[i] >= seg_start && times[i] <= seg_end) {
                // 归一化时间到[0,1]
                double t_norm = (times[i] - seg_start) / (seg_end - seg_start);
                seg_times.push_back(t_norm);
                seg_errors.push_back(up_errors[i]);
            }
        }
        
        if (seg_times.size() < params_.polynomial_degree + 2) {
            continue;  // 数据不足
        }
        
        // 多项式拟合（使用最小二乘法）
        int degree = std::min(params_.polynomial_degree, 
                             static_cast<int>(seg_times.size()) - 1);
        
        Eigen::MatrixXd A(seg_times.size(), degree + 1);
        Eigen::VectorXd b(seg_times.size());
        
        for (size_t i = 0; i < seg_times.size(); ++i) {
            double t = seg_times[i];
            double t_power = 1.0;
            for (int j = 0; j <= degree; ++j) {
                A(i, j) = t_power;
                t_power *= t;
            }
            b(i) = seg_errors[i];
        }
        
        // 求解
        Eigen::VectorXd coeffs = A.colPivHouseholderQr().solve(b);
        
        // 保存段信息
        TimeVariantBiasModel::Segment segment;
        segment.time_start = seg_start;
        segment.time_end = seg_end;
        segment.poly_coeffs.resize(degree + 1);
        for (int j = 0; j <= degree; ++j) {
            segment.poly_coeffs[j] = coeffs(j);
        }
        
        time_variant_model_up_.segments.push_back(segment);
    }
    
    if (enable_debug_) {
        ROS_INFO("[GNSSBiasCorrector] Time-variant model built with %zu segments",
                 time_variant_model_up_.segments.size());
    }
}

void GNSSBiasCorrector::detectOutliers() {
    if (observation_buffer_.size() < 30) return;
    
    // 计算UP误差的中位数和MAD
    std::vector<double> up_errors;
    for (const auto& obs : observation_buffer_) {
        if (obs.has_ground_truth) {
            up_errors.push_back(obs.error_enu.z());
        }
    }
    
    if (up_errors.empty()) return;
    
    std::vector<double> sorted_errors = up_errors;
    std::nth_element(sorted_errors.begin(), 
                    sorted_errors.begin() + sorted_errors.size() / 2,
                    sorted_errors.end());
    double median = sorted_errors[sorted_errors.size() / 2];
    
    // 计算MAD
    std::vector<double> abs_deviations;
    for (double err : up_errors) {
        abs_deviations.push_back(std::abs(err - median));
    }
    std::nth_element(abs_deviations.begin(),
                    abs_deviations.begin() + abs_deviations.size() / 2,
                    abs_deviations.end());
    double mad = abs_deviations[abs_deviations.size() / 2];
    
    // 标记离群点
    double threshold = params_.outlier_threshold_sigma * 1.4826 * mad;  // 1.4826是MAD到标准差的转换因子
    
    int outlier_count = 0;
    for (auto& obs : observation_buffer_) {
        if (obs.has_ground_truth) {
            double modified_z_score = 0.6745 * (obs.error_enu.z() - median) / (mad + 1e-10);
            obs.is_outlier = (std::abs(modified_z_score) > params_.outlier_threshold_sigma);
            if (obs.is_outlier) outlier_count++;
        }
    }
    
    if (enable_debug_ && outlier_count > 0) {
        ROS_INFO("[GNSSBiasCorrector] Detected %d outliers (%.1f%%)",
                 outlier_count, 100.0 * outlier_count / up_errors.size());
    }
}

double GNSSBiasCorrector::computeMovingAverage(const std::deque<double>& values) const {
    if (values.empty()) return 0.0;
    return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
}

void GNSSBiasCorrector::updateStatistics() {
    statistics_.total_points = observation_buffer_.size();
    statistics_.outliers_detected = 0;
    
    if (observation_buffer_.empty()) return;
    
    double sum_e_before = 0, sum_n_before = 0, sum_u_before = 0;
    double sum_e_after = 0, sum_n_after = 0, sum_u_after = 0;
    int count = 0;
    
    for (const auto& obs : observation_buffer_) {
        if (!obs.has_ground_truth) continue;
        
        if (obs.is_outlier) {
            statistics_.outliers_detected++;
            continue;
        }
        
        // 校正前误差
        double err_e_before = obs.error_enu.x();
        double err_n_before = obs.error_enu.y();
        double err_u_before = obs.error_enu.z();
        
        sum_e_before += err_e_before * err_e_before;
        sum_n_before += err_n_before * err_n_before;
        sum_u_before += err_u_before * err_u_before;
        
        // 校正后误差（估算）
        double err_e_after = err_e_before - static_bias_enu_.x();
        double err_n_after = err_n_before - static_bias_enu_.y();
        double err_u_after = err_u_before - static_bias_enu_.z();
        
        // 减去时变偏差
        if (!time_variant_model_up_.segments.empty()) {
            err_u_after -= time_variant_model_up_.evaluate(obs.timestamp);
        }
        
        sum_e_after += err_e_after * err_e_after;
        sum_n_after += err_n_after * err_n_after;
        sum_u_after += err_u_after * err_u_after;
        
        count++;
    }
    
    if (count > 0) {
        statistics_.rms_before_horizontal = std::sqrt((sum_e_before + sum_n_before) / count);
        statistics_.rms_before_vertical = std::sqrt(sum_u_before / count);
        statistics_.rms_after_horizontal = std::sqrt((sum_e_after + sum_n_after) / count);
        statistics_.rms_after_vertical = std::sqrt(sum_u_after / count);
        
        statistics_.improvement_horizontal = 
            (statistics_.rms_before_horizontal - statistics_.rms_after_horizontal) / 
            (statistics_.rms_before_horizontal + 1e-10) * 100.0;
        statistics_.improvement_vertical = 
            (statistics_.rms_before_vertical - statistics_.rms_after_vertical) / 
            (statistics_.rms_before_vertical + 1e-10) * 100.0;
        
        statistics_.outlier_rate = 100.0 * statistics_.outliers_detected / 
                                  static_cast<double>(statistics_.total_points);
        
        // 质量评估
        if (statistics_.rms_after_vertical < 1.5) {
            statistics_.quality_level = CorrectionStatistics::EXCELLENT;
        } else if (statistics_.rms_after_vertical < 2.0) {
            statistics_.quality_level = CorrectionStatistics::GOOD;
        } else if (statistics_.rms_after_vertical < 3.0) {
            statistics_.quality_level = CorrectionStatistics::MODERATE;
        } else {
            statistics_.quality_level = CorrectionStatistics::POOR;
        }
    }
}

GNSSBiasCorrector::CorrectionStatistics GNSSBiasCorrector::getStatistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return statistics_;
}

void GNSSBiasCorrector::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    
    observation_buffer_.clear();
    up_smoothing_buffer_.clear();
    time_variant_model_up_.segments.clear();
    
    static_bias_enu_.setZero();
    static_bias_estimated_ = false;
    is_initialized_ = false;
    
    if (kalman_filter_up_) {
        kalman_filter_up_->reset();
    }
    
    statistics_ = CorrectionStatistics();
    
    ROS_INFO("[GNSSBiasCorrector] Reset completed");
}