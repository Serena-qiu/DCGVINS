#ifndef EPHEMERIS_MANAGER_H_
#define EPHEMERIS_MANAGER_H_

#include <gnss_comm/gnss_utility.hpp>
#include <map>
#include <deque>
#include <mutex>
#include <ros/ros.h>

using namespace gnss_comm;  // 或者在下面使用完整命名空间

class EphemerisManager {
public:
    EphemerisManager(int agent_id) : agent_id_(agent_id) {}
    bool hasAnyEphemeris() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return !sat2ephem_.empty();
    }

    int getEphemerisCount() const {
        std::lock_guard<std::mutex> lock(mutex_);
        int count = 0;
        for (const auto& sat_pair : sat2ephem_) {
            count += sat_pair.second.size();
        }
        return count;
    }
    // 添加新星历
    void addEphemeris(const gnss_comm::EphemBasePtr& ephem) {
        std::lock_guard<std::mutex> lock(mutex_);
        
        int sat_id = ephem->sat;
        double toe = gnss_comm::time2sec(ephem->toe);
        
        // 检查是否是新星历或更新
        if (sat2time_index_[sat_id].count(toe) == 0) {
            sat2ephem_[sat_id].push_back(ephem);
            sat2time_index_[sat_id][toe] = sat2ephem_[sat_id].size() - 1;
            
            // 限制历史星历数量（每颗卫星最多保留10个）
            if (sat2ephem_[sat_id].size() > 10) {
                sat2ephem_[sat_id].pop_front();
                rebuildTimeIndex(sat_id);
            }
            
            ROS_DEBUG("Agent %d: New ephemeris for sat %d at toe %.1f", 
                     agent_id_, sat_id, toe);
        }
    }
    
    // 获取最佳星历
    gnss_comm::EphemBasePtr getBestEphemeris(int sat_id, double obs_time) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return getBestEphemerisUnsafe(sat_id, obs_time);
    }
    
    // 获取所有可用星历
    std::map<int, gnss_comm::EphemBasePtr> getAllValidEphemeris(double obs_time) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::map<int, gnss_comm::EphemBasePtr> valid_ephems;
        
        for (const auto& sat_pair : sat2ephem_) {
            int sat_id = sat_pair.first;
            auto ephem = getBestEphemerisUnsafe(sat_id, obs_time);  // 使用不加锁的版本
            if (ephem) {
                valid_ephems[sat_id] = ephem;
            }
        }
        
        return valid_ephems;
    }
    
    // 统计信息
    void printStatistics() const {
        std::lock_guard<std::mutex> lock(mutex_);
        ROS_INFO("Agent %d Ephemeris Statistics:", agent_id_);
        for (const auto& sat_pair : sat2ephem_) {
            ROS_INFO("  Sat %d: %zu ephemeris entries", 
                    sat_pair.first, sat_pair.second.size());
        }
    }
    
private:
    int agent_id_;
    mutable std::mutex mutex_;
    
    // 星历存储：卫星ID -> 星历列表（按时间排序）
    std::map<int, std::deque<gnss_comm::EphemBasePtr>> sat2ephem_;
    // 时间索引：卫星ID -> (toe时间 -> 列表索引)
    std::map<int, std::map<double, size_t>> sat2time_index_;
    
    // 内部不加锁的版本，用于避免重入锁死锁
    gnss_comm::EphemBasePtr getBestEphemerisUnsafe(int sat_id, double obs_time) const {
        if (sat2ephem_.count(sat_id) == 0) {
            return nullptr;
        }
        
        const auto& ephem_list = sat2ephem_.at(sat_id);
        gnss_comm::EphemBasePtr best_ephem = nullptr;
        double min_dt = 7200.0;  // 2小时最大有效期
        
        for (const auto& ephem : ephem_list) {
            double toe = gnss_comm::time2sec(ephem->toe);
            double dt = std::abs(toe - obs_time);
            
            // GPS/Galileo/BDS星历有效期：2小时
            // GLONASS星历有效期：30分钟
            double max_age = (satsys(sat_id, NULL) == SYS_GLO) ? 1800.0 : 7200.0;
            
            if (dt < min_dt && dt < max_age) {
                min_dt = dt;
                best_ephem = ephem;
            }
        }
        
        if (!best_ephem) {
            ROS_WARN_THROTTLE(5.0, "Agent %d: No valid ephemeris for sat %d at time %.1f", 
                            agent_id_, sat_id, obs_time);
        }
        
        return best_ephem;
    }
    
    void rebuildTimeIndex(int sat_id) {
        sat2time_index_[sat_id].clear();
        const auto& ephem_list = sat2ephem_[sat_id];
        for (size_t i = 0; i < ephem_list.size(); ++i) {
            double toe = gnss_comm::time2sec(ephem_list[i]->toe);
            sat2time_index_[sat_id][toe] = i;
        }
    }
};

#endif // EPHEMERIS_MANAGER_H_