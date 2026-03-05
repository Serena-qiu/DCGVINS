#include "real_time_dd_processor.hpp"
#include <algorithm>
#include <cmath>
#include <set>
#include <mutex>

RealTimeDDProcessor::RealTimeDDProcessor(double time_sync_threshold)
    : time_sync_threshold_(time_sync_threshold), 
      max_time_diff_(time_sync_threshold),
      total_dd_generated_(0), 
      last_cleanup_time_(0.0)
{
    ROS_INFO("Real-time DD processor initialized with sync threshold: %.3f s", time_sync_threshold);
}

void RealTimeDDProcessor::initializeAgent(int agent_id) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (agent_ephem_managers_.count(agent_id) == 0) {
        agent_ephem_managers_[agent_id] = 
            std::make_shared<EphemerisManager>(agent_id);
        ROS_INFO("Initialized ephemeris manager for agent %d", agent_id);
    }
}

void RealTimeDDProcessor::addEphemeris(int agent_id, const EphemBasePtr& ephem) 
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (agent_ephem_managers_.count(agent_id) == 0) {
        // 在锁内初始化，但不重复获取锁
        if (agent_ephem_managers_.count(agent_id) == 0) {
            agent_ephem_managers_[agent_id] = 
                std::make_shared<EphemerisManager>(agent_id);
            ROS_INFO("Initialized ephemeris manager for agent %d", agent_id);
        }
    }
    agent_ephem_managers_[agent_id]->addEphemeris(ephem);
    
    ROS_DEBUG_THROTTLE(5.0, "Agent %d: Added ephemeris for sat %d", 
                       agent_id, ephem->sat);
}

// 🔧 核心修复：完全重写processRealtimeObservations函数
void RealTimeDDProcessor::processRealtimeObservations(
    int agent_id,
    const std::vector<ObsPtr>& observations,
    const std::vector<EphemBasePtr>& ephemeris,
    double timestamp)
{
    ROS_DEBUG("Processing realtime observations for agent %d at time %.6f", 
              agent_id, timestamp);
    
    // 🔧 第一步：加锁，添加数据到缓冲区
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 查找或创建时间对齐的观测槽
        TimeAlignedObs* aligned_obs = nullptr;
        
        const double time_threshold = 0.5;
        
        for (auto& obs_slot : time_aligned_buffer_) {
            if (std::abs(obs_slot.timestamp - timestamp) < time_threshold) {
                aligned_obs = &obs_slot;
                break;
            }
        }
        
        if (!aligned_obs) {
            TimeAlignedObs new_slot;
            new_slot.timestamp = timestamp;
            time_aligned_buffer_.push_back(new_slot);
            aligned_obs = &time_aligned_buffer_.back();
            ROS_DEBUG("Created new slot for timestamp %.6f", timestamp);
        }
        
        // 添加当前agent的观测数据
        aligned_obs->agent_observations[agent_id] = observations;
        aligned_obs->agent_ephemeris[agent_id] = ephemeris;
        
        ROS_DEBUG("Added agent %d data to slot %.6f (total agents: %zu)", 
                  agent_id, timestamp, aligned_obs->agent_observations.size());
    }
    
    // 🔧 第二步：查找并移除可处理的槽（加锁-复制-移除-解锁）
    TimeAlignedObs slot_to_process;
    bool found_ready_slot = false;
    
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 查找第一个完整的槽（至少有2个agent的数据）
        auto slot_it = std::find_if(time_aligned_buffer_.begin(), 
                                   time_aligned_buffer_.end(),
                                   [](const TimeAlignedObs& obs) {
                                       return obs.agent_observations.size() >= 2;
                                   });
        
        if (slot_it != time_aligned_buffer_.end()) {
            // 复制槽数据
            slot_to_process = *slot_it;
            found_ready_slot = true;
            
            // 立即从缓冲区移除
            time_aligned_buffer_.erase(slot_it);
            
            ROS_DEBUG("Found complete slot at time %.6f, removed from buffer", 
                      slot_to_process.timestamp);
        }
        
        // 定期清理：移除过期的不完整槽
        double current_time = timestamp;  // 🔧 关键修复：使用GNSS时间戳而不是ROS系统时间
        auto old_size = time_aligned_buffer_.size();
        
        time_aligned_buffer_.erase(
            std::remove_if(time_aligned_buffer_.begin(),
                          time_aligned_buffer_.end(),
                          [current_time](const TimeAlignedObs& obs) {
                              // 移除超过5秒且不完整的槽
                              return (current_time - obs.timestamp) > 5.0 && 
                                     obs.agent_observations.size() < 2;
                          }),
            time_aligned_buffer_.end());
        
        if (time_aligned_buffer_.size() != old_size) {
            ROS_INFO("Cleaned up %zu incomplete/old slots from buffer", 
                     old_size - time_aligned_buffer_.size());
        }
    }
    
    // 🔧 第三步：在锁外处理复制出来的数据
    if (found_ready_slot) {
        try {
            computeDoubleDifferencesForSlot(slot_to_process);
            ROS_DEBUG("Successfully processed slot at time %.6f", slot_to_process.timestamp);
            
        } catch (const std::exception& e) {
            ROS_ERROR("DD computation for slot %.6f failed: %s", 
                      slot_to_process.timestamp, e.what());
        }
    }
    
    // 🔧 第四步：定期清理旧数据（降低频率，避免性能问题）
    static ros::Time last_cleanup = ros::Time::now();
    ros::Time current_ros_time = ros::Time::now();
    
    if ((current_ros_time - last_cleanup).toSec() > 10.0) {
        cleanupOldData(timestamp, 30.0);  // 🔧 修复：使用GNSS时间戳进行清理
        last_cleanup = current_ros_time;
        ROS_DEBUG("Performed periodic cleanup");
    }
}

void RealTimeDDProcessor::computeDoubleDifferencesForSlot(const TimeAlignedObs& slot)
{
    ROS_INFO("=== Starting DD computation for slot at time %.3f ===", slot.timestamp);
    
    // 如果只有一个agent的数据，直接返回
    if (slot.agent_observations.size() < 2) {
        ROS_WARN("Only %zu agents available, need 2, skipping DD", 
                 slot.agent_observations.size());
        return;
    }
    
    auto it = slot.agent_observations.begin();
    int agent1_id = it->first;
    const auto& obs1_list = it->second;
    
    ++it;
    int agent2_id = it->first;
    const auto& obs2_list = it->second;
    
    ROS_INFO("Computing DD: Agent %d has %zu obs, Agent %d has %zu obs", 
             agent1_id, obs1_list.size(), agent2_id, obs2_list.size());
    
    // 找共同卫星
    std::set<int> sats1, sats2, common_sats;
    for (const auto& obs : obs1_list) {
        if (obs && obs->sat > 0) sats1.insert(obs->sat);
    }
    for (const auto& obs : obs2_list) {
        if (obs && obs->sat > 0) sats2.insert(obs->sat);
    }
    
    ROS_INFO("Agent %d sats: %zu, Agent %d sats: %zu", 
             agent1_id, sats1.size(), agent2_id, sats2.size());
    
    std::set_intersection(sats1.begin(), sats1.end(),
                         sats2.begin(), sats2.end(),
                         std::inserter(common_sats, common_sats.begin()));
    
    ROS_INFO("Found %zu common satellites", common_sats.size());
    
    // 打印共同卫星列表
    if (!common_sats.empty()) {
        std::stringstream ss;
        for (int sat : common_sats) ss << sat << " ";
        ROS_INFO("Common satellites: %s", ss.str().c_str());
    }
    
    // 检查最小要求
    if (common_sats.size() < 2) {
        ROS_WARN("Insufficient common sats (%zu < 2), skipping DD", 
                common_sats.size());
        return;
    }
    
    // 🔧 关键修复：星历检查使用锁保护
    std::set<int> sats_with_valid_ephem;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        // 确保星历管理器已初始化
        if (agent_ephem_managers_.count(agent1_id) == 0) {
            ROS_INFO("Creating ephemeris manager for agent %d", agent1_id);
            agent_ephem_managers_[agent1_id] = 
                std::make_shared<EphemerisManager>(agent1_id);
        }
        if (agent_ephem_managers_.count(agent2_id) == 0) {
            ROS_INFO("Creating ephemeris manager for agent %d", agent2_id);
            agent_ephem_managers_[agent2_id] = 
                std::make_shared<EphemerisManager>(agent2_id);
        }
        
        // 检查星历可用性
        ROS_INFO("Checking ephemeris availability...");
        for (int sat_id : common_sats) {
            auto ephem = agent_ephem_managers_[agent1_id]->getBestEphemeris(
                sat_id, slot.timestamp);
            if (ephem) {
                sats_with_valid_ephem.insert(sat_id);
            }
        }
    }
    
    ROS_INFO("After ephemeris check: %zu satellites have valid ephemeris", 
             sats_with_valid_ephem.size());
    
    if (sats_with_valid_ephem.size() < 2) {
        ROS_WARN("Insufficient satellites with ephemeris (%zu < 2), skipping DD", 
                sats_with_valid_ephem.size());
        return;
    }
    
    // 选择参考卫星
    ROS_INFO("Selecting reference satellite...");
    int ref_sat = selectReferenceSatellite(sats_with_valid_ephem, obs1_list);
    ROS_INFO("Selected reference satellite: %d", ref_sat);
    
    // 计算双差
    ROS_INFO("Starting DD computation for %zu satellite pairs...", 
             sats_with_valid_ephem.size() - 1);
    
    int successful_dd = 0;
    int failed_dd = 0;
    
    for (int target_sat : sats_with_valid_ephem) {
        if (target_sat == ref_sat) continue;
        
        try {
            // --- 开始：手动查找4个观测值 ---
            ObsPtr obs_ref_agent1 = nullptr, obs_ref_agent2 = nullptr;
            ObsPtr obs_target_agent1 = nullptr, obs_target_agent2 = nullptr;
            
            for (const auto& obs : obs1_list) {
                if (obs && static_cast<int>(obs->sat) == ref_sat) obs_ref_agent1 = obs;
                if (obs && static_cast<int>(obs->sat) == target_sat) obs_target_agent1 = obs;
            }
            for (const auto& obs : obs2_list) {
                if (obs && static_cast<int>(obs->sat) == ref_sat) obs_ref_agent2 = obs;
                if (obs && static_cast<int>(obs->sat) == target_sat) obs_target_agent2 = obs;
            }
            
            if (!obs_ref_agent1 || !obs_ref_agent2 || !obs_target_agent1 || !obs_target_agent2) {
                ROS_WARN("Missing observations for DD pair (%d, %d)", ref_sat, target_sat);
                failed_dd++;
                continue;
            }
            // --- 结束：查找观测值 ---

            DoubleDifferenceObs dd_obs;
            dd_obs.timestamp = slot.timestamp;
            dd_obs.master_agent_id = agent1_id;
            dd_obs.secondary_agent_id = agent2_id;
            
            // 🔧 关键修复：调用正确的函数来计算卫星位置
            if (computeDoubleDifferenceWithEphemeris(
                    obs_ref_agent1, obs_ref_agent2,
                    obs_target_agent1, obs_target_agent2,
                    agent1_id, agent2_id, dd_obs)) 
            {
                // 增加日志以确认卫星位置已计算
                ROS_INFO("DD SUCCESS: sats(%d,%d), psr_dd=%.3f, sat1_pos_norm=%.1f, sat2_pos_norm=%.1f", 
                         ref_sat, target_sat, dd_obs.psr_dd, 
                         dd_obs.sat1_pos_ecef.norm(), dd_obs.sat2_pos_ecef.norm());
                
                // 🔧 线程安全地添加到结果缓冲区
                {
                    std::lock_guard<std::mutex> lock(dd_mutex_);
                    computed_dd_observations_.push_back(dd_obs);
                    total_dd_generated_++;
                }
                successful_dd++;
            } else {
                ROS_WARN("DD FAILED: sats(%d,%d) (computation failed)", ref_sat, target_sat);
                failed_dd++;
            }
            
        } catch (const std::exception& e) {
            ROS_ERROR("DD computation exception for sats(%d,%d): %s", 
                     ref_sat, target_sat, e.what());
            failed_dd++;
        }
    }
    
    ROS_INFO("=== DD computation complete: %d successful, %d failed ===", 
             successful_dd, failed_dd);
}

bool RealTimeDDProcessor::computeSingleDD(
    const std::vector<ObsPtr>& obs1_list, 
    const std::vector<ObsPtr>& obs2_list,
    int ref_sat, int target_sat, 
    DoubleDifferenceObs& dd_obs)
{
    // 找到对应的观测数据
    ObsPtr obs_ref_agent1 = nullptr, obs_ref_agent2 = nullptr;
    ObsPtr obs_target_agent1 = nullptr, obs_target_agent2 = nullptr;
    
    // Agent 1 的观测
    for (const auto& obs : obs1_list) {
        if (static_cast<int>(obs->sat) == ref_sat) obs_ref_agent1 = obs;
        if (static_cast<int>(obs->sat) == target_sat) obs_target_agent1 = obs;
    }
    
    // Agent 2 的观测
    for (const auto& obs : obs2_list) {
        if (static_cast<int>(obs->sat) == ref_sat) obs_ref_agent2 = obs;
        if (static_cast<int>(obs->sat) == target_sat) obs_target_agent2 = obs;
    }
    
    if (!obs_ref_agent1 || !obs_ref_agent2 || !obs_target_agent1 || !obs_target_agent2) {
        throw DDProcessorException("Missing observations for DD computation");
    }
    
    // 获取L1频率索引
    int freq_idx = -1;
    L1_freq(obs_ref_agent1, &freq_idx);
    if (freq_idx < 0) {
        throw DDProcessorException("No L1 frequency found in observations");
    }
    
    try {
        // 计算双差（钟差在双差中被消除）
        double psr_ref_sd = obs_ref_agent1->psr[freq_idx] - obs_ref_agent2->psr[freq_idx];
        double psr_target_sd = obs_target_agent1->psr[freq_idx] - obs_target_agent2->psr[freq_idx];
        dd_obs.psr_dd = psr_ref_sd - psr_target_sd;
        
        dd_obs.sat1_id = ref_sat;
        dd_obs.sat2_id = target_sat;
        dd_obs.quality_score = assessDDQuality(dd_obs);
        
        return true;
        
    } catch (const std::exception& e) {
        throw DDProcessorException("DD computation failed: " + std::string(e.what()));
    }
}

int RealTimeDDProcessor::selectReferenceSatellite(
    const std::set<int>& common_sats, 
    const std::vector<ObsPtr>& obs_list)
{
    // 选择信号最强的卫星作为参考
    int best_sat = *common_sats.begin();
    double best_snr = 0;
    
    for (int sat_id : common_sats) {
        // 跳过BDS卫星作为参考星
        if (sat_id > 100) continue;
        for (const auto& obs : obs_list) {
            if (static_cast<int>(obs->sat) == sat_id) {
                // 使用伪距标准差的倒数作为信号质量指标
                if (!obs->psr_std.empty()) {
                    double snr = 1.0 / obs->psr_std[0];
                    if (snr > best_snr) {
                        best_snr = snr;
                        best_sat = sat_id;
                    }
                }
                break;
            }
        }
    }
    
    return best_sat;
}

std::vector<DoubleDifferenceObs> RealTimeDDProcessor::getAvailableDDObservations(
    double current_time, double time_window)
{
    std::lock_guard<std::mutex> lock(dd_mutex_);
    
    std::vector<DoubleDifferenceObs> available_obs;
    
    for (const auto& dd_obs : computed_dd_observations_) {
        if (std::abs(dd_obs.timestamp - current_time) <= time_window) {
            available_obs.push_back(dd_obs);
        }
    }
    
    // 清理过期的观测
    computed_dd_observations_.erase(
        std::remove_if(computed_dd_observations_.begin(), 
                      computed_dd_observations_.end(),
                      [current_time, time_window](const DoubleDifferenceObs& obs) {
                          return std::abs(obs.timestamp - current_time) > time_window * 2;
                      }),
        computed_dd_observations_.end());
    
    return available_obs;
}

void RealTimeDDProcessor::inputAgentData(const AgentGnssData& agent_data)
{
    ROS_INFO("Receiving Agent %d data: %zu started, time=%.6f", 
             agent_data.agent_id, 
             agent_data.observations.size(), 
             agent_data.timestamp);
    processRealtimeObservations(
        agent_data.agent_id,
        agent_data.observations,
        agent_data.ephemeris,
        agent_data.timestamp);
}

std::vector<DoubleDifferenceObs> RealTimeDDProcessor::processAvailableData()
{
    return getAvailableDDObservations(ros::Time::now().toSec());
}

bool RealTimeDDProcessor::checkTimeSynchronization(
    const AgentGnssData& data1, 
    const AgentGnssData& data2) const
{
    double time_diff = std::abs(data1.timestamp - data2.timestamp);
    return time_diff <= time_sync_threshold_;
}

bool RealTimeDDProcessor::computeDoubleDifferenceWithEphemeris(
    const ObsPtr& obs_ref_agent1, const ObsPtr& obs_ref_agent2,
    const ObsPtr& obs_tar_agent1, const ObsPtr& obs_tar_agent2,
    int agent1_id, int agent2_id,
    DoubleDifferenceObs& dd_obs) 
{
    std::lock_guard<std::mutex> lock(data_mutex_); // 保护星历访问
    
    // 1. 获取星历 (我们只需要主agent的星历，因为之前已经做了共享)
    auto ephem_ref = agent_ephem_managers_[agent1_id]->getBestEphemeris(
        obs_ref_agent1->sat, time2sec(obs_ref_agent1->time));
    auto ephem_tar = agent_ephem_managers_[agent1_id]->getBestEphemeris(
        obs_tar_agent1->sat, time2sec(obs_tar_agent1->time));
    
    // 2. 检查星历有效性
    if (!ephem_ref || !ephem_tar) {
        ROS_WARN("Agent %d missing ephemeris for sats (%d,%d)", 
                agent1_id, obs_ref_agent1->sat, obs_tar_agent1->sat);
        return false;
    }
    
    // 3. 计算卫星位置
    Eigen::Vector3d sat_ref_pos, sat_tar_pos;
    
    // 计算参考卫星位置
    if (satsys(obs_ref_agent1->sat, NULL) == SYS_GLO) {
        sat_ref_pos = geph2pos(obs_ref_agent1->time, 
            std::dynamic_pointer_cast<GloEphem>(ephem_ref), NULL);
    } else {
        sat_ref_pos = eph2pos(obs_ref_agent1->time, 
            std::dynamic_pointer_cast<Ephem>(ephem_ref), NULL);
    }
    
    // 计算目标卫星位置
    if (satsys(obs_tar_agent1->sat, NULL) == SYS_GLO) {
        sat_tar_pos = geph2pos(obs_tar_agent1->time, 
            std::dynamic_pointer_cast<GloEphem>(ephem_tar), NULL);
    } else {
        sat_tar_pos = eph2pos(obs_tar_agent1->time, 
            std::dynamic_pointer_cast<Ephem>(ephem_tar), NULL);
    }

    // 检查计算结果是否有效
    if (sat_ref_pos.norm() < 1e6 || sat_tar_pos.norm() < 1e6) {
        ROS_WARN("Failed to compute valid satellite positions for sats (%d,%d)", 
                obs_ref_agent1->sat, obs_tar_agent1->sat);
        return false;
    }
    
    // 4. 将计算出的卫星位置存入结果结构体
    dd_obs.sat1_pos_ecef = sat_ref_pos;
    dd_obs.sat2_pos_ecef = sat_tar_pos;
    
    // 5. 计算双差
    int freq_idx = -1;
    L1_freq(obs_ref_agent1, &freq_idx);
    if (freq_idx < 0) return false;
    
    double psr_ref_sd = obs_ref_agent1->psr[freq_idx] - obs_ref_agent2->psr[freq_idx];
    double psr_tar_sd = obs_tar_agent1->psr[freq_idx] - obs_tar_agent2->psr[freq_idx];
    dd_obs.psr_dd = psr_ref_sd - psr_tar_sd;
    
    // 6. 填充其余信息
    dd_obs.sat1_id = obs_ref_agent1->sat;
    dd_obs.sat2_id = obs_tar_agent1->sat;
    dd_obs.quality_score = assessDDQuality(dd_obs);
    
    return true;
}

double RealTimeDDProcessor::assessDDQuality(const DoubleDifferenceObs& dd_obs) const
{
    double psr_dd_magnitude = std::abs(dd_obs.psr_dd);
    
    if (psr_dd_magnitude < 10.0) return 1.0;
    else if (psr_dd_magnitude < 30.0) return 0.8;
    else if (psr_dd_magnitude < 50.0) return 0.6;
    else if (psr_dd_magnitude < 100.0) return 0.4;
    else return 0.2;
}

void RealTimeDDProcessor::cleanupOldData(double current_time, double max_age)
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    // 清理时间对齐缓存
    auto old_buffer_size = time_aligned_buffer_.size();
    time_aligned_buffer_.erase(
        std::remove_if(time_aligned_buffer_.begin(),
                      time_aligned_buffer_.end(),
                      [current_time, max_age](const TimeAlignedObs& obs) {
                          return (current_time - obs.timestamp) > max_age;
                      }),
        time_aligned_buffer_.end());
    
    if (time_aligned_buffer_.size() != old_buffer_size) {
        ROS_INFO("Cleaned up %zu old entries from time-aligned buffer", 
                old_buffer_size - time_aligned_buffer_.size());
    }
    
    // 清理agent数据缓存
    for (auto& agent_buffer : agent_data_buffer_) {
        auto& buffer = agent_buffer.second;
        while (!buffer.empty() && (current_time - buffer.front().timestamp) > max_age) {
            buffer.pop_front();
        }
    }
    
    last_cleanup_time_ = current_time;
}

void RealTimeDDProcessor::getStatistics(int& total_observations, int& valid_dd_count, 
                                       double& avg_quality) const
{
    std::lock_guard<std::mutex> lock(dd_mutex_);
    
    total_observations = 0;
    for (const auto& agent_buffer : agent_data_buffer_) {
        for (const auto& data : agent_buffer.second) {
            total_observations += data.observations.size();
        }
    }
    
    valid_dd_count = total_dd_generated_;
    
    if (!computed_dd_observations_.empty()) {
        double sum_quality = 0;
        for (const auto& dd : computed_dd_observations_) {
            sum_quality += dd.quality_score;
        }
        avg_quality = sum_quality / computed_dd_observations_.size();
    } else {
        avg_quality = 0.0;
    }
}