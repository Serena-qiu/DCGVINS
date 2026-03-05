#ifndef REAL_TIME_DD_PROCESSOR_H_
#define REAL_TIME_DD_PROCESSOR_H_

#include <map>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <ros/ros.h>
#include "ephemeris_manager.hpp"  // 新增的星历管理器、
#include <Eigen/Core>

using namespace gnss_comm;

struct RealTimeDDObs {
    double timestamp;
    int sat1_id;
    int sat2_id;
    double psr_dd_observed;
    double quality_weight;
    Eigen::Vector3d sat1_pos_ecef;
    Eigen::Vector3d sat2_pos_ecef;
    double measurement_std;
    
    RealTimeDDObs() : timestamp(0.0), sat1_id(0), sat2_id(0), 
                      psr_dd_observed(0.0), quality_weight(1.0),
                      measurement_std(10.0) {}
};

class DDProcessorException : public std::exception {
public:
    explicit DDProcessorException(const std::string& message) : message_(message) {}
    const char* what() const noexcept override { return message_.c_str(); }
private:
    std::string message_;
};

// 数据结构定义
struct AgentGnssData {
    int agent_id;
    double timestamp;
    std::vector<ObsPtr> observations;
    std::vector<EphemBasePtr> ephemeris;
    Eigen::Vector3d approximate_position;

    AgentGnssData() : agent_id(0), timestamp(0.0), 
                      approximate_position(Eigen::Vector3d::Zero()) {}
};

struct DoubleDifferenceObs {
    double timestamp;
    int master_agent_id;
    int secondary_agent_id;
    int sat1_id, sat2_id;
    double psr_dd;
    double cp_dd;
    double quality_score;
    Eigen::Vector3d sat1_pos_ecef;
    Eigen::Vector3d sat2_pos_ecef;

    DoubleDifferenceObs() : timestamp(0), master_agent_id(0), secondary_agent_id(0),
                           sat1_id(0), sat2_id(0), psr_dd(0), cp_dd(0), quality_score(1.0) {}
};

class RealTimeDDProcessor
{
public:
    // 构造函数（只声明一次）
    explicit RealTimeDDProcessor(double time_sync_threshold = 0.05);
    ~RealTimeDDProcessor() = default;

    // 核心接口
    void inputAgentData(const AgentGnssData& agent_data);
    std::vector<DoubleDifferenceObs> processAvailableData();
    std::vector<DoubleDifferenceObs> getAvailableDDObservations(
        double current_time, double time_window = 0.1);
    
    // 实时处理接口
    void processRealtimeObservations(
        int agent_id,
        const std::vector<ObsPtr>& observations,
        const std::vector<EphemBasePtr>& ephemeris,
        double timestamp);
    
    // 星历管理
    void initializeAgent(int agent_id);
    void addEphemeris(int agent_id, const EphemBasePtr& ephem);
    
    // 管理接口
    void setCommDelay(int agent1, int agent2, double delay_ms);
    void cleanupOldData(double current_time, double max_age = 2.0);
    void getStatistics(int& total_observations, int& valid_dd_count, 
                      double& avg_quality) const;
    // 新增：获取特定agent的星历
    EphemBasePtr getEphemerisForAgent(int agent_id, int sat_id, double obs_time) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (agent_ephem_managers_.count(agent_id) == 0) {
            ROS_WARN_ONCE("No ephemeris manager for agent %d", agent_id);
            return nullptr;
        }
        
        return agent_ephem_managers_[agent_id]->getBestEphemeris(sat_id, obs_time);
    }
    // 获取agent的所有有效星历
    std::map<int, EphemBasePtr> getAllEphemerisForAgent(int agent_id, double obs_time) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        if (agent_ephem_managers_.count(agent_id) == 0) {
            return std::map<int, EphemBasePtr>();
        }
        
        return agent_ephem_managers_[agent_id]->getAllValidEphemeris(obs_time);
    }
    
    // 获取星历统计信息
    void printEphemerisStatistics() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        ROS_INFO("=== Ephemeris Statistics ===");
        for (const auto& pair : agent_ephem_managers_) {
            int agent_id = pair.first;
            auto ephems = pair.second->getAllValidEphemeris(ros::Time::now().toSec());
            ROS_INFO("Agent %d: %zu satellites with valid ephemeris", 
                    agent_id, ephems.size());
        }
    }

private:
    // 时间对齐的观测缓存（添加缺失的结构）
    struct TimeAlignedObs {
        double timestamp;
        std::map<int, std::vector<ObsPtr>> agent_observations;
        std::map<int, std::vector<EphemBasePtr>> agent_ephemeris;
        bool isComplete() const { return agent_observations.size() >= 2; }
    };

    // 数据成员
    std::map<int, std::deque<AgentGnssData>> agent_data_buffer_;
    std::deque<TimeAlignedObs> time_aligned_buffer_;  // 添加缺失的成员
    std::vector<DoubleDifferenceObs> computed_dd_observations_;  // 添加缺失的成员
    
    // 星历管理器
    std::map<int, std::shared_ptr<EphemerisManager>> agent_ephem_managers_;
    
    // 线程安全
    mutable std::mutex data_mutex_;
    mutable std::mutex dd_mutex_; // 添加缺失的互斥锁

    // 参数
    double time_sync_threshold_;
    double max_time_diff_;  // 添加缺失的成员
    std::map<std::pair<int,int>, double> comm_delays_;

    // 统计
    mutable int total_dd_generated_;
    mutable double last_cleanup_time_;

    // 内部方法
    bool checkTimeSynchronization(const AgentGnssData& data1, 
                                 const AgentGnssData& data2) const;
    
    bool findTimeMatchedObservations(const AgentGnssData& data1, 
                                    const AgentGnssData& data2,
                                    std::vector<std::pair<ObsPtr, ObsPtr>>& matched_obs) const;
    
    void computeDoubleDifferencesForSlot(const TimeAlignedObs& slot);  // 添加缺失的函数
    
    bool computeSingleDD(const std::vector<ObsPtr>& obs1_list, 
                        const std::vector<ObsPtr>& obs2_list,
                        int ref_sat, int target_sat, 
                        DoubleDifferenceObs& dd_obs);  // 添加缺失的函数
    
    int selectReferenceSatellite(const std::set<int>& common_sats, 
                                const std::vector<ObsPtr>& obs_list);  // 添加缺失的函数
    
    // 修正拼写错误
    bool computeDoubleDifference(const ObsPtr& obs1_agent1, const ObsPtr& obs1_agent2,
                                const ObsPtr& obs2_agent1, const ObsPtr& obs2_agent2,
                                const EphemBasePtr& ephem1, const EphemBasePtr& ephem2,  // 修正拼写
                                DoubleDifferenceObs& dd_obs) const;
    
    // 使用星历的双差计算
    bool computeDoubleDifferenceWithEphemeris(
        const ObsPtr& obs_ref_agent1, const ObsPtr& obs_ref_agent2,
        const ObsPtr& obs_tar_agent1, const ObsPtr& obs_tar_agent2,
        int agent1_id, int agent2_id,
        DoubleDifferenceObs& dd_obs);
    
    std::vector<std::pair<int,int>> selectOptimalSatellitePairs(
        const std::vector<ObsPtr>& obs1, 
        const std::vector<ObsPtr>& obs2) const;
    
    double assessDDQuality(const DoubleDifferenceObs& dd_obs) const;
};

#endif // REAL_TIME_DD_PROCESSOR_H_