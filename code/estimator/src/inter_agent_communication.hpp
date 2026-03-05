#ifndef INTER_AGENT_COMMUNICATION_H_
#define INTER_AGENT_COMMUNICATION_H_

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <gnss_comm/gnss_ros.hpp>
#include "real_time_dd_processor.hpp"
#include <functional>
#include <vector>

using namespace gnss_comm;

class InterAgentCommunication
{
public:
    explicit InterAgentCommunication(ros::NodeHandle& nh, int self_agent_id);
    ~InterAgentCommunication() = default;
    
    // 发送本agent的GNSS数据给其他agents
    void broadcastGnssData(const std::vector<ObsPtr>& gnss_observations,
                          const std::vector<EphemBasePtr>& ephemeris,
                          const Eigen::Vector3d& current_position_estimate);
    
    // 设置接收到其他agent数据时的回调
    void setDataReceivedCallback(std::function<void(const AgentGnssData&)> callback);
    
    // 发送位置估计（用于协作定位质量评估）
    void sendPositionEstimate(const Eigen::Vector3d& position, 
                             const Eigen::Matrix3d& covariance);
    
    // 获取通信统计信息
    void getCommStats(int& messages_sent, int& messages_received, 
                     double& last_received_time) const;

private:
    ros::NodeHandle nh_;
    int self_agent_id_;
    
    // ROS发布器和订阅器
    ros::Publisher gnss_data_pub_;
    ros::Publisher position_pub_;
    ros::Subscriber gnss_data_sub_;
    ros::Subscriber position_sub_;
    
    // 回调函数
    std::function<void(const AgentGnssData&)> data_received_callback_;
    
    // 统计信息
    mutable int messages_sent_;
    mutable int messages_received_;
    mutable double last_received_time_;
    
    // ROS回调函数
    void gnssDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    // 数据序列化/反序列化
    std::vector<double> serializeGnssData(const std::vector<ObsPtr>& observations,
                                         const std::vector<EphemBasePtr>& ephemeris);
    bool deserializeGnssData(const std::vector<double>& data, AgentGnssData& agent_data);
    
    // 验证接收到的数据
    bool validateReceivedData(const AgentGnssData& data) const;
};

// ========== 实现部分 ==========

InterAgentCommunication::InterAgentCommunication(ros::NodeHandle& nh, int self_agent_id)
    : nh_(nh), self_agent_id_(self_agent_id), messages_sent_(0), 
      messages_received_(0), last_received_time_(0.0)
{
    // 设置发布器
    std::string gnss_out_topic = "/cooperative_gnss/agent_" + std::to_string(self_agent_id_) + "/gnss_data";
    std::string pos_out_topic = "/cooperative_gnss/agent_" + std::to_string(self_agent_id_) + "/position";
    
    gnss_data_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(gnss_out_topic, 10);
    position_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(pos_out_topic, 10);
    
    // 订阅对方agent的数据
    int target_agent_id = (self_agent_id_ == 0) ? 1 : 0;
    std::string gnss_sub_topic = "/cooperative_gnss/agent_" + std::to_string(target_agent_id) + "/gnss_data";
    std::string pos_sub_topic = "/cooperative_gnss/agent_" + std::to_string(target_agent_id) + "/position";
    
    gnss_data_sub_ = nh_.subscribe<std_msgs::Float64MultiArray>(
        gnss_sub_topic, 100, &InterAgentCommunication::gnssDataCallback, this);
    position_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        pos_sub_topic, 100, &InterAgentCommunication::positionCallback, this);
    
    ROS_INFO("Inter-agent communication initialized for agent %d", self_agent_id_);
    ROS_INFO("  Publishing GNSS data on: %s", gnss_out_topic.c_str());
    ROS_INFO("  Publishing position on: %s", pos_out_topic.c_str());
    ROS_INFO("  Subscribing to: %s", gnss_sub_topic.c_str());
    ROS_INFO("  Subscribing to: %s", pos_sub_topic.c_str());
}

void InterAgentCommunication::broadcastGnssData(const std::vector<ObsPtr>& gnss_observations,
                                               const std::vector<EphemBasePtr>& ephemeris,
                                               const Eigen::Vector3d& current_position_estimate)
{
    if (gnss_observations.empty()) {
        return;
    }
    
    try {
        std_msgs::Float64MultiArray msg;
        msg.data = serializeGnssData(gnss_observations, ephemeris);
        
        // 添加header信息
        msg.layout.dim.resize(3);
        msg.layout.dim[0].label = "agent_id";
        msg.layout.dim[0].size = 1;
        msg.layout.dim[0].stride = 1;
        msg.layout.data_offset = self_agent_id_;
        
        msg.layout.dim[1].label = "timestamp";
        msg.layout.dim[1].size = 1;
        msg.layout.dim[1].stride = 1;
        
        msg.layout.dim[2].label = "obs_count";
        msg.layout.dim[2].size = gnss_observations.size();
        msg.layout.dim[2].stride = 5;
        
        gnss_data_pub_.publish(msg);
        messages_sent_++;
        
        ROS_DEBUG("Broadcasted GNSS data: %zu observations from agent %d", 
                  gnss_observations.size(), self_agent_id_);
        
    } catch (const std::exception& e) {
        ROS_WARN("Failed to broadcast GNSS data: %s", e.what());
    }
}

std::vector<double> InterAgentCommunication::serializeGnssData(
    const std::vector<ObsPtr>& observations,
    const std::vector<EphemBasePtr>& ephemeris)
{
    std::vector<double> serialized;
    
    if (observations.empty()) {
        return serialized;
    }
    
    serialized.push_back(static_cast<double>(observations.size()));
    double base_time = time2sec(observations[0]->time);
    serialized.push_back(base_time);
    
    for (size_t i = 0; i < observations.size(); i++) {
        const auto& obs = observations[i];
        
        serialized.push_back(static_cast<double>(obs->sat));
        
        int freq_idx = -1;
        L1_freq(obs, &freq_idx);
        if (freq_idx >= 0 && freq_idx < static_cast<int>(obs->psr.size())) {
            serialized.push_back(obs->psr[freq_idx]);
            serialized.push_back(obs->dopp[freq_idx]);
            if (!obs->cp.empty() && freq_idx < static_cast<int>(obs->cp.size())) {
                serialized.push_back(obs->cp[freq_idx]);
            } else {
                serialized.push_back(0.0);
            }
        } else {
            serialized.push_back(0.0);
            serialized.push_back(0.0);
            serialized.push_back(0.0);
        }
    }
    
    return serialized;
}

void InterAgentCommunication::gnssDataCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    try {
        int sender_agent_id = static_cast<int>(msg->layout.data_offset);
        
        if (sender_agent_id == self_agent_id_) {
            return;
        }
        
        AgentGnssData agent_data;
        if (deserializeGnssData(msg->data, agent_data)) {
            agent_data.agent_id = sender_agent_id;
            
            if (validateReceivedData(agent_data)) {
                messages_received_++;
                last_received_time_ = ros::Time::now().toSec();
                
                if (data_received_callback_) {
                    data_received_callback_(agent_data);
                }
                
                ROS_DEBUG("Received valid data from agent %d: %zu observations at %.6f",
                         sender_agent_id, agent_data.observations.size(), agent_data.timestamp);
            } else {
                ROS_WARN("Invalid data received from agent %d", sender_agent_id);
            }
        }
        
    } catch (const std::exception& e) {
        ROS_WARN("Failed to process received GNSS data: %s", e.what());
    }
}

bool InterAgentCommunication::deserializeGnssData(const std::vector<double>& data, AgentGnssData& agent_data)
{
    if (data.size() < 2) {
        return false;
    }
    
    try {
        size_t idx = 0;
        int obs_count = static_cast<int>(data[idx++]);
        double base_time = data[idx++];
        
        if (obs_count <= 0 || obs_count > 100) {
            return false;
        }
        
        agent_data.timestamp = base_time;
        agent_data.observations.clear();
        agent_data.ephemeris.clear();
        
        for (int i = 0; i < obs_count && idx + 3 < data.size(); i++) {
            // 修复：使用正确的类型名称
            ObsPtr obs = std::make_shared<Obs>();
            
            obs->sat = static_cast<int>(data[idx++]);
            double psr = data[idx++];
            double dopp = data[idx++]; 
            double cp = data[idx++];
            
            obs->time = sec2time(base_time);
            
            // 修复：使用正确的频率常量
            obs->freqs.push_back(FREQ1);
            obs->psr.push_back(psr);
            obs->dopp.push_back(dopp);
            obs->cp.push_back(cp);
            
            obs->psr_std.push_back(2.0);
            obs->dopp_std.push_back(0.5);
            obs->cp_std.push_back(0.02);
            
            agent_data.observations.push_back(obs);
            agent_data.ephemeris.push_back(nullptr);
        }
        
        return !agent_data.observations.empty();
        
    } catch (const std::exception& e) {
        ROS_WARN("Deserialization failed: %s", e.what());
        return false;
    }
}

bool InterAgentCommunication::validateReceivedData(const AgentGnssData& data) const
{
    if (data.observations.empty() || data.timestamp <= 0.0) {
        return false;
    }
    
    double current_time = ros::Time::now().toSec();
    double time_diff = std::abs(current_time - data.timestamp);
    if (time_diff > 60.0) {
        ROS_WARN("Received data with large time difference: %.1f seconds", time_diff);
        return false;
    }
    
    for (const auto& obs : data.observations) {
        if (!obs || obs->psr.empty()) {
            return false;
        }
        
        if (obs->psr[0] < 1e7 || obs->psr[0] > 3e8) {
            return false;
        }
    }
    
    return true;
}

void InterAgentCommunication::positionCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_DEBUG("Received position update from other agent");
}

void InterAgentCommunication::sendPositionEstimate(const Eigen::Vector3d& position, 
                                                  const Eigen::Matrix3d& covariance)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "ecef";
    
    pose_msg.pose.position.x = position[0];
    pose_msg.pose.position.y = position[1];
    pose_msg.pose.position.z = position[2];
    
    pose_msg.pose.orientation.w = 1.0;
    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    
    position_pub_.publish(pose_msg);
}

void InterAgentCommunication::setDataReceivedCallback(std::function<void(const AgentGnssData&)> callback)
{
    data_received_callback_ = callback;
}

void InterAgentCommunication::getCommStats(int& messages_sent, int& messages_received, 
                                          double& last_received_time) const
{
    messages_sent = messages_sent_;
    messages_received = messages_received_;
    last_received_time = last_received_time_;
}

#endif // INTER_AGENT_COMMUNICATION_H_