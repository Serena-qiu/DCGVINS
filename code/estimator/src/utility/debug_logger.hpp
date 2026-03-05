#ifndef DEBUG_LOGGER_HPP_
#define DEBUG_LOGGER_HPP_

#include <string>
#include <fstream>
#include <mutex>
#include "../real_time_dd_processor.hpp"

class DebugLogger {
public:
    explicit DebugLogger(const std::string& output_dir = "/tmp/dc_gvins_debug");
    ~DebugLogger() { if (main_log_.is_open()) main_log_.close(); }
    
    void logDDConstraint(const DoubleDifferenceObs& dd_obs, 
                        const std::string& status,
                        const std::string& details = "");
    
    void logTimeSync(double gnss_time, double local_time, 
                    double diff, bool success);
    
    void logOptimizationStats(int dd_constraints_added, 
                             int total_attempted,
                             double avg_quality);
    
    void setEnabled(bool enabled) { is_enabled_ = enabled; }
    
private:
    std::string output_dir_;
    std::ofstream main_log_;
    std::mutex log_mutex_;
    bool is_enabled_;
};

#endif