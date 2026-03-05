#include "time_synchronizer.hpp"
#include <algorithm>
#include <cmath>

#ifndef LIGHT_SPEED
#define LIGHT_SPEED 299792458.0  // 光速常量 m/s
#endif

// Use full scope resolution for the return type
TimeSynchronizer::SyncResult TimeSynchronizer::synchronizeByCommonView(
    const std::vector<ObsPtr>& obs1,
    const std::vector<ObsPtr>& obs2,
    const std::vector<EphemBasePtr>& ephems) 
{
    SyncResult result;  // 使用正确的结构体
    result.is_synchronized = false;
    
    // Find common visible satellites
    std::vector<int> common_sats;
    for (const auto& o1 : obs1) {
        for (const auto& o2 : obs2) {
            if (o1->sat == o2->sat) {
                common_sats.push_back(o1->sat);
            }
        }
    }
    
    if (common_sats.size() < 4) {
        return result;
    }
    
    // Calculate clock difference for each common-view satellite
    std::vector<double> clock_diffs;
    
    for (int sat_id : common_sats) {
        // Get observations from both receivers for the same satellite
        ObsPtr obs_rcv1 = nullptr, obs_rcv2 = nullptr;
        
        for (const auto& o : obs1) {
            if (static_cast<int>(o->sat) == sat_id) obs_rcv1 = o;
        }
        for (const auto& o : obs2) {
            if (static_cast<int>(o->sat) == sat_id) obs_rcv2 = o;
        }
        
        if (!obs_rcv1 || !obs_rcv2) continue;
        
        // Use pseudorange single difference to estimate clock difference
        int freq_idx = 0;
        if (obs_rcv1->psr.size() > static_cast<size_t>(freq_idx) && 
            obs_rcv2->psr.size() > static_cast<size_t>(freq_idx)) {
            // Single difference eliminates satellite clock error and atmospheric delays
            double single_diff = obs_rcv1->psr[freq_idx] - obs_rcv2->psr[freq_idx];
            
            // Convert to time difference (divide by speed of light)
            double time_diff = single_diff / LIGHT_SPEED;
            clock_diffs.push_back(time_diff);
        }
    }
    
    if (clock_diffs.empty()) {
        return result;
    }
    
    // Use median as robust estimate
    std::sort(clock_diffs.begin(), clock_diffs.end());
    result.time_offset = clock_diffs[clock_diffs.size() / 2];
    
    // Calculate sync quality (based on standard deviation)
    double sum = 0, sum_sq = 0;
    for (double diff : clock_diffs) {
        sum += diff;
        sum_sq += diff * diff;
    }
    double mean = sum / clock_diffs.size();
    double std_dev = std::sqrt(sum_sq / clock_diffs.size() - mean * mean);
    
    result.sync_quality = 1.0 / (1.0 + std_dev * 1e9);  // Nanosecond precision
    result.is_synchronized = (std_dev < 1e-7);  // 100ns threshold
    
    return result;
}

void TimeSynchronizer::ClockFilter::predict(double dt) {
    // 时钟预测模型
    clock_offset += clock_drift * dt;
    
    // 增加预测不确定性
    P(0,0) += 0.01 * dt;  // 位置方差增长
    P(1,1) += 0.001 * dt; // 速度方差增长
}

void TimeSynchronizer::ClockFilter::update(double measured_offset, double measurement_var) {
    // 卡尔曼滤波更新
    double innovation = measured_offset - clock_offset;
    double S = P(0,0) + measurement_var;
    
    if (S > 1e-10) {  // 避免除零
        double K1 = P(0,0) / S;
        double K2 = P(0,1) / S;
        
        clock_offset += K1 * innovation;
        clock_drift += K2 * innovation;
        
        // 更新协方差矩阵
        P(0,0) *= (1.0 - K1);
        P(0,1) *= (1.0 - K1);
        P(1,0) -= K2 * P(0,0);
        P(1,1) -= K2 * P(0,1);
    }
}