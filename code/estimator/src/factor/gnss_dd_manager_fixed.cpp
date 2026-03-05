#include <ros/ros.h>
#include "gnss_dd_factor.hpp"
#include "gnss_dd_manager.hpp"
#include <fstream>
#include <sstream>

GnssDDManager::GnssDDManager(const int max_history_size) : max_history_size_(max_history_size) {}

bool GnssDDManager::loadObservations(const std::string& data_file, const std::string& results_file) {
    observations.clear();
    
    std::ifstream data_stream(data_file);
    if (!data_stream.is_open()) {
        ROS_ERROR("Cannot open DD data file: %s", data_file.c_str());
        return false;
    }

    std::string line;
    int line_count = 0;
    int valid_count = 0;
    bool header_skipped = false;
    
    ROS_INFO("Loading DD data from: %s", data_file.c_str());
    
    while (std::getline(data_stream, line)) {
        line_count++;
        
        if (line.empty() || line[0] == '#') continue;
        
        // è·³è¿‡CSVå¤´éƒ¨
        if (!header_skipped && line.find("Timestamp") != std::string::npos) {
            header_skipped = true;
            continue;
        }
        
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, item, ',')) {
            tokens.push_back(item);
        }
        
        if (tokens.size() != 14) continue;
        
        try {
            ObsData obs;
            obs.timestamp = std::stod(tokens[0]);
            obs.sat_id1 = std::stoi(tokens[2]);
            obs.sat_id2 = std::stoi(tokens[3]);
            
            // è®¡ç®—å•å·® (Rcv1 - Rcv2)
            obs.psr1 = std::stod(tokens[4]) - std::stod(tokens[5]);  // PSR1å•å·®
            obs.psr2 = std::stod(tokens[8]) - std::stod(tokens[9]);  // PSR2å•å·®
            obs.cp1 = std::stod(tokens[6]) - std::stod(tokens[7]);   // CP1å•å·®
            obs.cp2 = std::stod(tokens[10]) - std::stod(tokens[11]); // CP2å•å·®
            obs.freq1 = std::stod(tokens[12]);
            obs.freq2 = std::stod(tokens[13]);
            
            // ç®€å•éªŒè¯
            if (obs.timestamp > 1e9 && obs.sat_id1 > 0 && obs.sat_id2 > 0 && obs.sat_id1 != obs.sat_id2) {
                observations.push_back(obs);
                valid_count++;
            }
            
        } catch (...) {
            // å¿½ç•¥è§£æžé”™è¯¯
        }
    }
    
    data_stream.close();
    
    ROS_INFO("Loaded %d DD observations from %d lines", valid_count, line_count);
    if (!observations.empty()) {
        ROS_INFO("Time range: [%.6f, %.6f]", observations.front().timestamp, observations.back().timestamp);
    }
    
    return valid_count > 0;
}

void GnssDDManager::addDDFactorToGraph(ceres::Problem& problem, 
                                       const ObsData& obs1, const ObsData& obs2,
                                       double* Pi, double* Pj, 
                                       double* ref_ecef, double quality_coeff) {
    auto dd_factor = new GnssDDFactor(obs1, obs2, quality_coeff);
    problem.AddResidualBlock(dd_factor, nullptr, Pi, Pj, ref_ecef);
}