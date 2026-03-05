// 文件位置：~/lab/2025.9/DC-GVINS/estimator/src/test_ephemeris_node.cpp

#include <ros/ros.h>
#include <gnss_comm/gnss_ros.hpp>
#include <map>
#include <set>

class EphemerisMonitor {
public:
    EphemerisMonitor() : nh_("~") {
        // 订阅主设备和副设备的星历
        ephem_sub_ = nh_.subscribe("/ublox_driver/ephem", 100, 
                                   &EphemerisMonitor::ephemCallback, this);
        glo_ephem_sub_ = nh_.subscribe("/ublox_driver/glo_ephem", 100,
                                       &EphemerisMonitor::gloEphemCallback, this);
        meas_sub_ = nh_.subscribe("/ublox_driver/range_meas", 100,
                                  &EphemerisMonitor::measCallback, this);
        
        // 定时器显示统计
        timer_ = nh_.createTimer(ros::Duration(2.0), 
                                &EphemerisMonitor::timerCallback, this);
        
        ROS_INFO("Ephemeris monitor started");
    }
    
private:
    void ephemCallback(const gnss_comm::GnssEphemMsgConstPtr& msg) {
        auto ephem = gnss_comm::msg2ephem(msg);
        if (ephem) {
            ephemeris_count_[ephem->sat]++;
            total_ephem_++;
            last_ephem_time_ = ros::Time::now();
        }
    }
    
    void gloEphemCallback(const gnss_comm::GnssGloEphemMsgConstPtr& msg) {
        auto glo_ephem = gnss_comm::msg2glo_ephem(msg);
        if (glo_ephem) {
            ephemeris_count_[glo_ephem->sat]++;
            total_glo_ephem_++;
            last_ephem_time_ = ros::Time::now();
        }
    }
    
    void measCallback(const gnss_comm::GnssMeasMsgConstPtr& msg) {
        auto observations = gnss_comm::msg2meas(msg);
        total_meas_++;
        
        // 记录观测到的卫星
        for (const auto& obs : observations) {
            observed_sats_.insert(obs->sat);
        }
        last_meas_time_ = ros::Time::now();
    }
    
    void timerCallback(const ros::TimerEvent&) {
        ROS_INFO("========== Ephemeris Monitor Report ==========");
        ROS_INFO("Total ephemeris received: %d (GPS/GAL/BDS)", total_ephem_);
        ROS_INFO("Total GLONASS ephemeris: %d", total_glo_ephem_);
        ROS_INFO("Total measurements: %d", total_meas_);
        ROS_INFO("Unique satellites with ephemeris: %zu", ephemeris_count_.size());
        ROS_INFO("Unique satellites observed: %zu", observed_sats_.size());
        
        // 检查哪些观测到的卫星缺少星历
        std::set<int> missing_ephem;
        for (int sat : observed_sats_) {
            if (ephemeris_count_.find(sat) == ephemeris_count_.end()) {
                missing_ephem.insert(sat);
            }
        }
        
        if (!missing_ephem.empty()) {
            ROS_WARN("Satellites observed but missing ephemeris: %zu", missing_ephem.size());
            std::stringstream ss;
            for (int sat : missing_ephem) {
                ss << sat << " ";
            }
            ROS_WARN("Missing: %s", ss.str().c_str());
        }
        
        // 显示时间差
        double dt_ephem = (ros::Time::now() - last_ephem_time_).toSec();
        double dt_meas = (ros::Time::now() - last_meas_time_).toSec();
        
        if (dt_ephem > 2.0) {
            ROS_WARN("No ephemeris received for %.1f seconds", dt_ephem);
        }
        if (dt_meas > 2.0) {
            ROS_WARN("No measurements received for %.1f seconds", dt_meas);
        }
        
        ROS_INFO("===========================================");
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber ephem_sub_, glo_ephem_sub_, meas_sub_;
    ros::Timer timer_;
    
    std::map<int, int> ephemeris_count_;  // sat_id -> count
    std::set<int> observed_sats_;
    int total_ephem_ = 0;
    int total_glo_ephem_ = 0;
    int total_meas_ = 0;
    ros::Time last_ephem_time_;
    ros::Time last_meas_time_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_ephemeris");
    EphemerisMonitor monitor;
    ros::spin();
    return 0;
}