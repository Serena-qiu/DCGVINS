#include "fault_tolerance_manager.hpp"
#include <algorithm>

FaultToleranceManager::FaultToleranceManager() {}

void FaultToleranceManager::updateAgentStatus(int agent_id, double current_time) 
{
    if (agent_statuses_.find(agent_id) == agent_statuses_.end()) {
        AgentStatus status;
        status.agent_id = agent_id;
        status.state = CONNECTED;
        status.last_message_time = current_time;
        status.weight_factor = 1.0;
        status.missed_messages = 0;
        agent_statuses_[agent_id] = status;
        
        ROS_INFO("New agent %d connected at time %.2f", agent_id, current_time);
        return;
    }
    
    auto& status = agent_statuses_[agent_id];
    double time_since_last = current_time - status.last_message_time;
    
    // 收到新消息，更新时间戳
    if (time_since_last < 0.01) {  // 新消息
        status.last_message_time = current_time;
    }
    
    if (time_since_last < TIMEOUT_DEGRADED) {
        // 正常连接状态
        if (status.state != CONNECTED) {
            ROS_INFO("Agent %d reconnected", agent_id);
        }
        status.state = CONNECTED;
        status.weight_factor = std::min(1.0, status.weight_factor * 1.1);
        status.missed_messages = 0;
    } 
    else if (time_since_last < TIMEOUT_DISCONNECT) {
        // 降级状态
        if (status.state == CONNECTED) {
            ROS_WARN("Agent %d connection degrading", agent_id);
        }
        status.state = DEGRADED;
        status.weight_factor = std::max(0.1, status.weight_factor * WEIGHT_DECAY_RATE);
        status.missed_messages++;
        
        ROS_WARN_THROTTLE(1.0, "Agent %d degraded: %.1fs since last msg, weight=%.2f", 
                         agent_id, time_since_last, status.weight_factor);
    } 
    else {
        // 断开连接
        if (status.state != DISCONNECTED) {
            ROS_ERROR("Agent %d disconnected", agent_id);
        }
        status.state = DISCONNECTED;
        status.weight_factor = 0.0;
    }
}

double FaultToleranceManager::getConstraintWeight(int agent_id) const 
{
    auto it = agent_statuses_.find(agent_id);
    if (it != agent_statuses_.end()) {
        return it->second.weight_factor;
    }
    return 1.0;  // 默认权重
}

void FaultToleranceManager::handleReconnection(int agent_id) 
{
    if (agent_statuses_.find(agent_id) != agent_statuses_.end()) {
        auto& status = agent_statuses_[agent_id];
        
        ROS_INFO("Agent %d reconnection handled, restoring full weight", agent_id);
        
        status.weight_factor = 1.0;
        status.state = CONNECTED;
        status.missed_messages = 0;
        status.last_message_time = ros::Time::now().toSec();
    }
}

FaultToleranceManager::ConnectionState FaultToleranceManager::getAgentState(int agent_id) const
{
    auto it = agent_statuses_.find(agent_id);
    if (it != agent_statuses_.end()) {
        return it->second.state;
    }
    return DISCONNECTED;
}

std::vector<int> FaultToleranceManager::getConnectedAgents() const 
{
    std::vector<int> connected_agents;
    for (const auto& pair : agent_statuses_) {
        if (pair.second.state == CONNECTED) {
            connected_agents.push_back(pair.first);
        }
    }
    return connected_agents;
}

void FaultToleranceManager::printStatus() const 
{
    ROS_INFO("=== Agent Connection Status ===");
    for (const auto& pair : agent_statuses_) {
        const auto& status = pair.second;
        std::string state_str = (status.state == CONNECTED) ? "CONNECTED" :
                                (status.state == DEGRADED) ? "DEGRADED" : "DISCONNECTED";
        ROS_INFO("Agent %d: %s (weight=%.2f, missed=%d)", 
                status.agent_id, state_str.c_str(), 
                status.weight_factor, status.missed_messages);
    }
    ROS_INFO("==============================");
}