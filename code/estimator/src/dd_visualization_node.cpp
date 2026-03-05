// dd_visualization_node.cpp
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseArray.h>

class DDVisualizationNode {
public:
    DDVisualizationNode(ros::NodeHandle& nh) : nh_(nh) {
        // 发布器
        dd_residual_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/dd_visualization/residuals", 10);
        
        dd_statistics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>(
            "/dd_visualization/statistics", 10);
        
        trajectory_comparison_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/dd_visualization/trajectories", 10);
        
        // 订阅器
        dd_result_sub_ = nh_.subscribe(
            "/dd_processor/results", 10, 
            &DDVisualizationNode::ddResultCallback, this);
        
        pose_sub_ = nh_.subscribe(
            "/gvins/camera_pose", 10,
            &DDVisualizationNode::poseCallback, this);
        
        // 定时器
        vis_timer_ = nh_.createTimer(
            ros::Duration(0.1), 
            &DDVisualizationNode::visualizationTimerCallback, this);
    }
    
private:
    void visualizeDDResiduals() {
        visualization_msgs::MarkerArray markers;
        int id = 0;
        
        for (const auto& dd_data : recent_dd_data_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            
            // 残差大小映射到颜色
            double residual = dd_data.residual;
            marker.color.r = std::min(1.0, std::abs(residual) / 10.0);
            marker.color.g = std::max(0.0, 1.0 - std::abs(residual) / 10.0);
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            
            // 残差大小映射到高度
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = std::abs(residual);
            
            // 位置基于卫星对
            marker.pose.position.x = dd_data.sat1_id * 2.0;
            marker.pose.position.y = dd_data.sat2_id * 2.0;
            marker.pose.position.z = marker.scale.z / 2.0;
            
            markers.markers.push_back(marker);
        }
        
        dd_residual_pub_.publish(markers);
    }
    
    void publishStatistics() {
        std_msgs::Float64MultiArray stats_msg;
        
        // 计算统计信息
        double avg_residual = 0, max_residual = 0, rms_residual = 0;
        int valid_count = 0;
        
        for (const auto& dd : recent_dd_data_) {
            avg_residual += std::abs(dd.residual);
            max_residual = std::max(max_residual, std::abs(dd.residual));
            rms_residual += dd.residual * dd.residual;
            valid_count++;
        }
        
        if (valid_count > 0) {
            avg_residual /= valid_count;
            rms_residual = std::sqrt(rms_residual / valid_count);
        }
        
        stats_msg.data.push_back(avg_residual);
        stats_msg.data.push_back(max_residual);
        stats_msg.data.push_back(rms_residual);
        stats_msg.data.push_back(valid_count);
        
        dd_statistics_pub_.publish(stats_msg);
    }
    
    void compareTrajectories() {
        visualization_msgs::MarkerArray markers;
        
        // 显示有DD约束的轨迹（绿色）
        visualization_msgs::Marker dd_traj;
        dd_traj.header.frame_id = "world";
        dd_traj.header.stamp = ros::Time::now();
        dd_traj.id = 0;
        dd_traj.type = visualization_msgs::Marker::LINE_STRIP;
        dd_traj.scale.x = 0.1;
        dd_traj.color.r = 0.0;
        dd_traj.color.g = 1.0;
        dd_traj.color.b = 0.0;
        dd_traj.color.a = 1.0;
        
        for (const auto& pose : trajectory_with_dd_) {
            dd_traj.points.push_back(pose);
        }
        markers.markers.push_back(dd_traj);
        
        // 显示无DD约束的轨迹（红色）
        visualization_msgs::Marker no_dd_traj;
        no_dd_traj.header = dd_traj.header;
        no_dd_traj.id = 1;
        no_dd_traj.type = visualization_msgs::Marker::LINE_STRIP;
        no_dd_traj.scale.x = 0.1;
        no_dd_traj.color.r = 1.0;
        no_dd_traj.color.g = 0.0;
        no_dd_traj.color.b = 0.0;
        no_dd_traj.color.a = 0.5;
        
        for (const auto& pose : trajectory_without_dd_) {
            no_dd_traj.points.push_back(pose);
        }
        markers.markers.push_back(no_dd_traj);
        
        trajectory_comparison_pub_.publish(markers);
    }
    
    ros::NodeHandle nh_;
    ros::Publisher dd_residual_pub_;
    ros::Publisher dd_statistics_pub_;
    ros::Publisher trajectory_comparison_pub_;
    ros::Subscriber dd_result_sub_;
    ros::Subscriber pose_sub_;
    ros::Timer vis_timer_;
    
    struct DDData {
        double timestamp;
        int sat1_id, sat2_id;
        double residual;
        double weight;
    };
    
    std::deque<DDData> recent_dd_data_;
    std::vector<geometry_msgs::Point> trajectory_with_dd_;
    std::vector<geometry_msgs::Point> trajectory_without_dd_;
};