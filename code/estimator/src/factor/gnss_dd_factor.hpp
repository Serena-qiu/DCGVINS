// gnss_dd_factor.hpp
#ifndef GNSS_DD_FACTOR_H_
#define GNSS_DD_FACTOR_H_

#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>

using namespace gnss_comm;

#ifndef LIGHT_SPEED
#define LIGHT_SPEED 299792458.0
#endif

// 双差观测数据结构
struct ObsData {
    double timestamp;
    int sat_id1, sat_id2;
    double psr1, psr2;           // 原始观测或单差
    double freq1, freq2;
    
    // 新增：预计算的双差值
    double psr_double_diff;      // PSR_DoubleDiff
    
    ObsData() : timestamp(0), sat_id1(0), sat_id2(0), 
                psr1(0), psr2(0), freq1(1575.42e6), freq2(1575.42e6),
                psr_double_diff(0) {}
};

/* 
 * 预计算双差因子 - 约束主设备状态
 * 使用已计算好的双差值作为观测
 * Parameters[0]: 主设备位置姿态 (7维)
 * Parameters[1]: 主设备速度bias (9维) 
 * Parameters[2]: 参考ECEF坐标 (3维)
 */
class GnssDDFactor : public ceres::SizedCostFunction<1, 7, 9, 3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    GnssDDFactor() = delete;
    
    // 使用预计算双差数据的构造函数
    GnssDDFactor(const ObsData& obs_data, double quality_coeff = 1.0)
        : obs_data_(obs_data), quality_coeff_(quality_coeff)
    {
        // 直接使用CSV中预计算的双差值
        psr_dd_observed_ = obs_data.psr_double_diff;
        
        // 生成卫星位置用于约束计算
        generateSatellitePositions();
        
        // 设置权重
        setupWeights(quality_coeff);
        
        ROS_DEBUG("DD Factor created: sats(%d,%d), psr_dd=%.3f", 
                  obs_data.sat_id1, obs_data.sat_id2, psr_dd_observed_);
    }
    
    // 兼容原有接口的构造函数
    GnssDDFactor(const ObsData& obs1, const ObsData& obs2, double quality_coeff = 1.0)
        : GnssDDFactor(obs1, quality_coeff) 
    {
        // obs2参数暂时忽略，因为双差值已经预计算
    }
    
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
    {
        // 提取主设备状态
        Eigen::Vector3d P_main(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Q_main(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
        
        // 速度和bias（如果需要的话）
        Eigen::Vector3d V_main(parameters[1][0], parameters[1][1], parameters[1][2]);
        
        // 参考ECEF坐标
        Eigen::Vector3d ref_ecef(parameters[2][0], parameters[2][1], parameters[2][2]);
        
        // 转换主设备位置到ECEF坐标
        Eigen::Matrix3d R_ecef_enu = ecef2rotation(ref_ecef);
        Eigen::Vector3d P_ecef_main = R_ecef_enu.transpose() * P_main + ref_ecef;
        
        // 计算主设备到两颗卫星的距离
        double range_main_sat1 = (sat1_pos_ - P_ecef_main).norm();
        double range_main_sat2 = (sat2_pos_ - P_ecef_main).norm();
        
        // 计算主设备的卫星间单差
        double single_diff_main = range_main_sat1 - range_main_sat2;

        residuals[0] = (psr_dd_observed_ - single_diff_main) * psr_weight_;
       
        
        // 计算雅可比矩阵
        if (jacobians)
        {
            // 计算卫星方向向量
            Eigen::Vector3d dir_sat1 = (sat1_pos_ - P_ecef_main).normalized();
            Eigen::Vector3d dir_sat2 = (sat2_pos_ - P_ecef_main).normalized();
            Eigen::Vector3d dd_gradient = dir_sat1 - dir_sat2;
            
            // Jacobian w.r.t 主设备位置姿态
            if (jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> J_pose(jacobians[0]);
                J_pose.setZero();
                
                // 位置部分的Jacobian
                Eigen::Vector3d dp_dlocal = R_ecef_enu.transpose() * dd_gradient;
                J_pose.block<1, 3>(0, 0) = -dp_dlocal.transpose() * psr_weight_;
                
                // 姿态部分通常影响较小，设为0
            }
            
            // Jacobian w.r.t 速度和bias
            if (jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 9, Eigen::RowMajor>> J_speed_bias(jacobians[1]);
                J_speed_bias.setZero();
            }
            
            // Jacobian w.r.t 参考ECEF坐标
            if (jacobians[2])
            {
                Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> J_ref(jacobians[2]);
                J_ref.setZero();
                // 参考点变化的影响
                J_ref.block<1, 3>(0, 0) = -dd_gradient.transpose() * psr_weight_ * 0.1;
            }
        }
        
        return true;
    }

    double getObservedDD() const { 
        return psr_dd_observed_; 
    }
    
    double getWeight() const { 
        return psr_weight_; 
    }
    
    double getPredictedDD(double const *const *parameters) const {
        // 提取主设备状态
        Eigen::Vector3d P_main(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Vector3d ref_ecef(parameters[2][0], parameters[2][1], parameters[2][2]);
        
        // 转换主设备位置到ECEF坐标
        Eigen::Matrix3d R_ecef_enu = ecef2rotation(ref_ecef);
        Eigen::Vector3d P_ecef_main = R_ecef_enu.transpose() * P_main + ref_ecef;
        
        // 计算主设备到两颗卫星的距离
        double range_main_sat1 = (sat1_pos_ - P_ecef_main).norm();
        double range_main_sat2 = (sat2_pos_ - P_ecef_main).norm();
        
        return range_main_sat1 - range_main_sat2;
    }
    
private:
    void generateSatellitePositions()
    {
        // 基于卫星ID生成典型的卫星位置（用于约束计算）
        const double GPS_ORBIT_RADIUS = 26570000.0; // GPS轨道半径
        
        // 为不同卫星ID生成不同的位置
        double azimuth1 = (obs_data_.sat_id1 * 15.0) * M_PI / 180.0;  // 每颗卫星相隔15度
        double elevation1 = (30.0 + obs_data_.sat_id1 * 2.0) * M_PI / 180.0;
        
        double azimuth2 = (obs_data_.sat_id2 * 15.0) * M_PI / 180.0;
        double elevation2 = (30.0 + obs_data_.sat_id2 * 2.0) * M_PI / 180.0;
        
        // 卫星1位置
        sat1_pos_.x() = GPS_ORBIT_RADIUS * cos(elevation1) * cos(azimuth1);
        sat1_pos_.y() = GPS_ORBIT_RADIUS * cos(elevation1) * sin(azimuth1);
        sat1_pos_.z() = GPS_ORBIT_RADIUS * sin(elevation1);
        
        // 卫星2位置
        sat2_pos_.x() = GPS_ORBIT_RADIUS * cos(elevation2) * cos(azimuth2);
        sat2_pos_.y() = GPS_ORBIT_RADIUS * cos(elevation2) * sin(azimuth2);
        sat2_pos_.z() = GPS_ORBIT_RADIUS * sin(elevation2);
    }
    
    void setupWeights(double quality_coeff)
    {
        // 基于双差值的大小评估质量
        double psr_quality = assessMeasurementQuality(std::abs(psr_dd_observed_), 100.0); // 100m阈值
        
        // 设置权重
        psr_weight_ = quality_coeff * psr_quality / (20.0 * 20.0); // 假设20m标准差
        
        // 限制权重范围
        psr_weight_ = std::min(std::max(psr_weight_, 0.01), 1.0);
    }
    
    static double assessMeasurementQuality(double value, double threshold)
    {
        // 根据测量值大小评估质量
        if (value < threshold * 0.1) return 1.0;      // 很好
        else if (value < threshold * 0.3) return 0.8; // 好
        else if (value < threshold * 0.6) return 0.5; // 中等
        else if (value < threshold) return 0.3;       // 差
        else return 0.1;                              // 很差
    }
    
    // 成员变量
    ObsData obs_data_;
    double quality_coeff_;
    
    // 观测值
    double psr_dd_observed_;
    
    // 权重和参数
    double psr_weight_;
    
    // 卫星位置
    Eigen::Vector3d sat1_pos_, sat2_pos_;
};

#endif // GNSS_DD_FACTOR_H_