#include "debug_logger.hpp"
#include <ros/ros.h>
#include <iomanip>
#include <sstream>

DebugLogger::DebugLogger(const std::string& output_dir) 
    : output_dir_(output_dir), is_enabled_(true) 
{
    // 创建调试目录
    std::string cmd = "mkdir -p " + output_dir_;
    (void)system(cmd.c_str());
    
    // 打开主日志文件
    std::string filename = output_dir_ + "/dd_debug_" + 
                          std::to_string(ros::Time::now().toSec()) + ".csv";
    main_log_.open(filename);
    
    if (main_log_.is_open()) {
        main_log_ << "timestamp,event_type,sat1_id,sat2_id,value,quality,status,details\n";
        ROS_INFO("Debug logger initialized: %s", filename.c_str());
    } else {
        ROS_ERROR("Failed to open debug log: %s", filename.c_str());
        is_enabled_ = false;
    }
}

void DebugLogger::logDDConstraint(const DoubleDifferenceObs& dd_obs, 
                                  const std::string& status,
                                  const std::string& details) 
{
    if (!is_enabled_) return;
    
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    main_log_ << std::fixed << std::setprecision(6)
             << dd_obs.timestamp << ","
             << "DD_CONSTRAINT,"
             << dd_obs.sat1_id << ","
             << dd_obs.sat2_id << ","
             << dd_obs.psr_dd << ","
             << dd_obs.quality_score << ","
             << status << ","
             << details << "\n";
    
    main_log_.flush();
}

void DebugLogger::logTimeSync(double gnss_time, double local_time, 
                              double diff, bool success) 
{
    if (!is_enabled_) return;
    
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    main_log_ << std::fixed << std::setprecision(6)
             << ros::Time::now().toSec() << ","
             << "TIME_SYNC,"
             << "0,0,"  // 无卫星ID
             << diff << ","
             << (success ? 1.0 : 0.0) << ","
             << (success ? "SUCCESS" : "FAILED") << ","
             << "gnss:" << gnss_time << " local:" << local_time << "\n";
    
    main_log_.flush();
}