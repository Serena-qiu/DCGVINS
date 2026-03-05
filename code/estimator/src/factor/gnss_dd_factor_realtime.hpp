#ifndef GNSS_DD_FACTOR_REALTIME_H_
#define GNSS_DD_FACTOR_REALTIME_H_

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>

using namespace gnss_comm;

/* 
 * 🔴 修正后的实时双差因子
 * 
 * 双差定义：DD = (主_星1 - 副_星1) - (主_星2 - 副_星2)
 *               = (主_星1 - 主_星2) - (副_星1 - 副_星2)
 *               = 主设备单差 - 副设备单差
 * 
 * Parameters:
 * [0]: 主设备位置姿态 (7维: tx, ty, tz, qx, qy, qz, qw)
 * [1]: 主设备速度bias (9维: vx, vy, vz, bax, bay, baz, bgx, bgy, bgz) 
 * [2]: 参考ECEF坐标 (3维: ref_x, ref_y, ref_z)
 */
class GnssDDFactorRealtime : public ceres::SizedCostFunction<1, 7, 9, 3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GnssDDFactorRealtime() = delete;
    
    // 构造函数需要副设备位置
    explicit GnssDDFactorRealtime(const RealTimeDDObs& dd_obs, 
                                   const Eigen::Vector3d& secondary_pos_ecef)
        : dd_obs_(dd_obs), secondary_pos_ecef_(secondary_pos_ecef)
    {
        // 设置权重
        double std_psr = std::max(dd_obs.measurement_std, 1.0);
        weight_ = dd_obs.quality_weight / (std_psr * std_psr);
        
        // 验证卫星位置
        if (dd_obs_.sat1_pos_ecef.norm() < 1e6 || dd_obs_.sat2_pos_ecef.norm() < 1e6) {
            ROS_WARN("Invalid satellite positions in DD factor, sats(%d,%d)", 
                     dd_obs.sat1_id, dd_obs.sat2_id);
            weight_ = 0.0;
        }
        
        // 验证副设备位置
        if (secondary_pos_ecef_.norm() < 1e6 || secondary_pos_ecef_.norm() > 7e6) {
            ROS_WARN("Invalid secondary position: %.1f m, using zero weight", 
                     secondary_pos_ecef_.norm());
            weight_ = 0.0;
        }
        
        ROS_DEBUG("RT DD Factor: sats(%d,%d), psr_dd=%.3f, secondary_pos=[%.1f,%.1f,%.1f], weight=%.6f", 
                  dd_obs.sat1_id, dd_obs.sat2_id, dd_obs.psr_dd_observed,
                  secondary_pos_ecef_.x(), secondary_pos_ecef_.y(), secondary_pos_ecef_.z(),
                  weight_);
    }
    
    virtual bool Evaluate(double const *const *parameters, double *residuals, 
                         double **jacobians) const override
    {
        // 检查权重
        if (weight_ <= 0.0) {
            residuals[0] = 0.0;
            if (jacobians) {
                if (jacobians[0]) memset(jacobians[0], 0, sizeof(double) * 1 * 7);
                if (jacobians[1]) memset(jacobians[1], 0, sizeof(double) * 1 * 9);
                if (jacobians[2]) memset(jacobians[2], 0, sizeof(double) * 1 * 3);
            }
            return true;
        }
        
        // ========== 1. 提取主设备状态（优化变量） ==========
        Eigen::Vector3d P_main(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Q_main(parameters[0][6], parameters[0][3], 
                                  parameters[0][4], parameters[0][5]);
        Q_main.normalize();
        
        // 参考ECEF坐标
        Eigen::Vector3d ref_ecef(parameters[2][0], parameters[2][1], parameters[2][2]);
        
        // ========== 2. 转换主设备位置到ECEF ==========
        Eigen::Matrix3d R_ecef_enu = ecef2rotation(ref_ecef);
        Eigen::Vector3d P_ecef_main = ref_ecef + R_ecef_enu * P_main;
        
        // ========== 3. 计算主设备单差 ==========
        Eigen::Vector3d vec_main_sat1 = dd_obs_.sat1_pos_ecef - P_ecef_main;
        Eigen::Vector3d vec_main_sat2 = dd_obs_.sat2_pos_ecef - P_ecef_main;
        
        double range_main_sat1 = vec_main_sat1.norm();
        double range_main_sat2 = vec_main_sat2.norm();
        double single_diff_main = range_main_sat1 - range_main_sat2;
        
        // ========== 4. 计算副设备单差（用SPP位置） ==========
        Eigen::Vector3d vec_secondary_sat1 = dd_obs_.sat1_pos_ecef - secondary_pos_ecef_;
        Eigen::Vector3d vec_secondary_sat2 = dd_obs_.sat2_pos_ecef - secondary_pos_ecef_;
        
        double range_secondary_sat1 = vec_secondary_sat1.norm();
        double range_secondary_sat2 = vec_secondary_sat2.norm();
        double single_diff_secondary = range_secondary_sat1 - range_secondary_sat2;
        
        // ========== 5. 计算预测双差 ==========
        double predicted_dd = single_diff_main - single_diff_secondary;
        
        // ========== 6. 计算残差 ==========
        residuals[0] = (dd_obs_.psr_dd_observed - predicted_dd) * weight_;
        
        // 调试信息
        if (std::abs(residuals[0]) > 100.0 * weight_) {
            ROS_WARN_THROTTLE(5.0, 
                "Large DD residual: %.2fm for sats(%d,%d)\n"
                "  Observed DD: %.2f\n"
                "  Predicted DD: %.2f (main_sd=%.2f, sec_sd=%.2f)\n"
                "  Main pos: [%.1f, %.1f, %.1f]\n"
                "  Secondary pos: [%.1f, %.1f, %.1f]",
                residuals[0]/weight_, dd_obs_.sat1_id, dd_obs_.sat2_id,
                dd_obs_.psr_dd_observed, predicted_dd, 
                single_diff_main, single_diff_secondary,
                P_ecef_main.x(), P_ecef_main.y(), P_ecef_main.z(),
                secondary_pos_ecef_.x(), secondary_pos_ecef_.y(), secondary_pos_ecef_.z());
        }
        
        // ========== 7. 计算Jacobian矩阵 ==========
        if (jacobians)
        {
            // 单位方向向量
            Eigen::Vector3d u_main_sat1 = vec_main_sat1.normalized();
            Eigen::Vector3d u_main_sat2 = vec_main_sat2.normalized();
            
            // 🔴 关键：双差梯度 = -(u主_星1 - u主_星2)
            // 注意：副设备位置固定，其梯度为0
            Eigen::Vector3d dd_gradient = -(u_main_sat1 - u_main_sat2);
            
            // Jacobian w.r.t 主设备位置姿态
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_pose(jacobians[0]);
                J_pose.setZero();
                
                // 位置部分：∂residual/∂P_main
                Eigen::Vector3d dres_dP_local = R_ecef_enu.transpose() * dd_gradient * weight_;
                J_pose.block<1, 3>(0, 0) = dres_dP_local.transpose();
                
                // 姿态部分对伪距影响很小，设为0
                J_pose.block<1, 4>(0, 3).setZero();
            }
            
            // Jacobian w.r.t 速度和bias（对伪距无影响）
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 9, Eigen::RowMajor>> J_speed_bias(jacobians[1]);
                J_speed_bias.setZero();
            }
            
            // Jacobian w.r.t 参考ECEF坐标
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_ref(jacobians[2]);
                J_ref = dd_gradient.transpose() * weight_;
            }
        }
        
        return true;
    }
    
    // 辅助函数：获取观测值
    double getObservedDD() const { return dd_obs_.psr_dd_observed; }
    double getWeight() const { return weight_; }
    
    // 计算预测双差（调试用）
    double getPredictedDD(double const *const *parameters) const
    {
        Eigen::Vector3d P_main(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d ref_ecef(parameters[2][0], parameters[2][1], parameters[2][2]);
        
        Eigen::Matrix3d R_ecef_enu = ecef2rotation(ref_ecef);
        Eigen::Vector3d P_ecef_main = ref_ecef + R_ecef_enu * P_main;
        
        // 主设备单差
        double range_main_sat1 = (dd_obs_.sat1_pos_ecef - P_ecef_main).norm();
        double range_main_sat2 = (dd_obs_.sat2_pos_ecef - P_ecef_main).norm();
        double single_diff_main = range_main_sat1 - range_main_sat2;
        
        // 副设备单差
        double range_sec_sat1 = (dd_obs_.sat1_pos_ecef - secondary_pos_ecef_).norm();
        double range_sec_sat2 = (dd_obs_.sat2_pos_ecef - secondary_pos_ecef_).norm();
        double single_diff_sec = range_sec_sat1 - range_sec_sat2;
        
        return single_diff_main - single_diff_sec;
    }
    
private:
    RealTimeDDObs dd_obs_;
    Eigen::Vector3d secondary_pos_ecef_;  // 副设备ECEF位置（来自SPP）
    double weight_;
};

#endif // GNSS_DD_FACTOR_REALTIME_H_