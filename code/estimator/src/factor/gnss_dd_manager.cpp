// gnss_dd_manager.cpp - ä¿®å¤CSVè§£æžé—®é¢˜

#include <ros/ros.h>
#include "gnss_dd_factor.hpp"
#include "gnss_dd_manager.hpp"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <iomanip>

GnssDDManager::GnssDDManager(const int max_history_size) : max_history_size_(max_history_size) {
}

bool GnssDDManager::loadObservations(const std::string& data_file, const std::string& results_file) {
    observations.clear();
    
    // å†³å®šä½¿ç”¨å“ªä¸ªæ–‡ä»¶
    std::string primary_file = results_file.empty() ? data_file : results_file;
    
    // æ£€æŸ¥æ–‡ä»¶æ˜¯å¦å­˜åœ¨
    std::ifstream test_file(primary_file);
    if (!test_file.good()) {
        ROS_ERROR("Cannot open DD file: %s", primary_file.c_str());
        return false;
    }
    test_file.close();
    
    // é‡æ–°æ‰“å¼€æ–‡ä»¶è¿›è¡Œè¯»å–
    std::ifstream file(primary_file);
    if (!file.is_open()) {
        ROS_ERROR("Cannot open DD file: %s", primary_file.c_str());
        return false;
    }

    std::string line;
    int line_count = 0;
    int valid_count = 0;
    int parse_errors = 0;
    int rejected_quality = 0;
    int rejected_satid = 0;
    int rejected_timestamp = 0;
    
    // åˆ¤æ–­æ–‡ä»¶ç±»åž‹
    bool is_results_file = primary_file.find("results") != std::string::npos ||
                          primary_file.find("DoubleDiff") != std::string::npos;
    
    ROS_INFO("Loading DD observations from: %s (type: %s)", 
             primary_file.c_str(), is_results_file ? "results" : "raw");
    
    // è¯»å–ç¬¬ä¸€è¡Œæ¥åˆ¤æ–­æ–‡ä»¶æ ¼å¼
    std::string first_line;
    if (std::getline(file, first_line)) {
        line_count++;
        
        // å¦‚æžœæ˜¯è¡¨å¤´ï¼Œæ˜¾ç¤ºå¹¶è·³è¿‡
        if (first_line.find("Timestamp") != std::string::npos) {
            ROS_INFO("Detected CSV header: %s", first_line.substr(0, 80).c_str());
            
            // æ ¹æ®è¡¨å¤´åˆ¤æ–­æ–‡ä»¶æ ¼å¼
            if (first_line.find("PSR_DoubleDiff") != std::string::npos) {
                is_results_file = true;
                ROS_INFO("Detected results file format (contains PSR_DoubleDiff)");
            } else {
                is_results_file = false;
                ROS_INFO("Detected raw data file format");
            }
        } else {
            // ä¸æ˜¯è¡¨å¤´ï¼Œéœ€è¦å¤„ç†è¿™è¡Œæ•°æ®ï¼Œæ‰€ä»¥å›žåˆ°æ–‡ä»¶å¼€å§‹
            file.clear();
            file.seekg(0);
            line_count = 0;
        }
    }
    
    // æ˜¾ç¤ºå‰å‡ è¡Œç”¨äºŽè°ƒè¯•
    std::streampos current_pos = file.tellg();
    int preview_count = 0;
    ROS_INFO("Preview of data lines:");
    while (std::getline(file, line) && preview_count < 3) {
        if (!line.empty() && line[0] != '#' && line.find("Timestamp") == std::string::npos) {
            ROS_INFO("  Line %d: %s", preview_count + 1, line.substr(0, 100).c_str());
            preview_count++;
        }
    }
    
    // å›žåˆ°ä¹‹å‰çš„ä½ç½®ç»§ç»­å¤„ç†
    file.clear();
    file.seekg(current_pos);
    
    // å¤„ç†æ•°æ®
    while (std::getline(file, line)) {
        line_count++;
        
        if (line.empty() || line[0] == '#') continue;
        
        // è·³è¿‡è¡¨å¤´
        if (line.find("Timestamp") != std::string::npos) {
            continue;
        }
        
        std::stringstream ss(line);
        std::string item;
        std::vector<std::string> tokens;
        
        while (std::getline(ss, item, ',')) {
            // åŽ»é™¤é¦–å°¾ç©ºæ ¼
            size_t start = item.find_first_not_of(" \t\r\n");
            size_t end = item.find_last_not_of(" \t\r\n");
            if (start != std::string::npos && end != std::string::npos) {
                item = item.substr(start, end - start + 1);
            }
            tokens.push_back(item);
        }
        
        // è°ƒè¯•ï¼šæ˜¾ç¤ºç¬¬ä¸€è¡Œæ•°æ®çš„åˆ—æ•°
        if (valid_count == 0 && tokens.size() > 0) {
            ROS_INFO("First data line has %zu columns", tokens.size());
            for (size_t i = 0; i < std::min(tokens.size(), size_t(10)); i++) {
                ROS_INFO("  Column[%zu]: '%s'", i, tokens[i].c_str());
            }
        }
        
        try {
            // æ ¹æ®æ–‡ä»¶ç±»åž‹å¤„ç†æ•°æ®
            if (is_results_file && tokens.size() >= 5) {
                // å¤„ç†ç»“æžœæ–‡ä»¶æ ¼å¼
                ObsData obs;
                
                // åŸºæœ¬å­—æ®µ
                obs.timestamp = std::stod(tokens[0]);
                // tokens[1] æ˜¯ System (GPS)
                obs.sat_id1 = std::stoi(tokens[2]);
                obs.sat_id2 = std::stoi(tokens[3]);
                
                // é¢„è®¡ç®—çš„åŒå·®å€¼
                obs.psr_double_diff = std::stod(tokens[4]);        // PSR_DoubleDiff
                
                // é¢‘çŽ‡ä¿¡æ¯
                if (tokens.size() > 17) {
                    obs.freq1 = std::stod(tokens[17]) * 1e6; // MHzè½¬Hz
                    obs.freq2 = std::stod(tokens[18]) * 1e6;
                } else {
                    obs.freq1 = 1575.42e6; // GPS L1é»˜è®¤é¢‘çŽ‡
                    obs.freq2 = 1575.42e6;
                }
                
                // éªŒè¯æ•°æ®æœ‰æ•ˆæ€§
                bool is_valid = true;
                
                // æ—¶é—´æˆ³æ£€æŸ¥
                if (obs.timestamp <= 1e9 || obs.timestamp > 2e9) {
                    rejected_timestamp++;
                    is_valid = false;
                }
                
                // å«æ˜ŸIDæ£€æŸ¥
                if (obs.sat_id1 <= 0 || obs.sat_id2 <= 0 || obs.sat_id1 == obs.sat_id2) {
                    rejected_satid++;
                    is_valid = false;
                }
                
                // åŒå·®å€¼è´¨é‡æ£€æŸ¥
                if (std::abs(obs.psr_double_diff) > 5000.0) {
                    rejected_quality++;
                    if (rejected_quality <= 5) {
                        ROS_WARN("Rejected extreme DD values: psr_dd=%.1f", 
                                 obs.psr_double_diff);
                    }
                    is_valid = false;
                }
                
                if (is_valid) {
                    observations.push_back(obs);
                    valid_count++;
                    
                    // æ˜¾ç¤ºå‰å‡ ä¸ªæœ‰æ•ˆæ•°æ®
                    if (valid_count <= 5) {
                        ROS_INFO("DD[%d]: t=%.6f, sats=(%d,%d), psr_dd=%.3f", 
                                 valid_count, obs.timestamp, obs.sat_id1, obs.sat_id2, 
                                 obs.psr_double_diff);
                    }
                }
                
            } else if (!is_results_file && tokens.size() >= 12) {
                // å¤„ç†åŽŸå§‹æ•°æ®æ–‡ä»¶æ ¼å¼ï¼ˆéœ€è¦è®¡ç®—åŒå·®ï¼‰
                ObsData obs;
                obs.timestamp = std::stod(tokens[0]);
                obs.sat_id1 = std::stoi(tokens[2]);
                obs.sat_id2 = std::stoi(tokens[3]);
                
                // åŽŸå§‹è§‚æµ‹æ•°æ®
                double psr1_rcv1 = std::stod(tokens[4]);  // PSR1_Rcv1
                double psr1_rcv2 = std::stod(tokens[5]);  // PSR1_Rcv2
                double psr2_rcv1 = std::stod(tokens[8]);  // PSR2_Rcv1
                double psr2_rcv2 = std::stod(tokens[9]);  // PSR2_Rcv2
                obs.freq1 = std::stod(tokens[12]) * 1e6; // MHzè½¬Hz
                obs.freq2 = std::stod(tokens[13]) * 1e6;
                
                // è®¡ç®—åŒå·®
                double psr_sd1 = psr1_rcv1 - psr1_rcv2;  // å«æ˜Ÿ1çš„å•å·®
                double psr_sd2 = psr2_rcv1 - psr2_rcv2;  // å«æ˜Ÿ2çš„å•å·®
                obs.psr_double_diff = psr_sd1 - psr_sd2; // åŒå·®
                
                // éªŒè¯å¹¶æ·»åŠ 
                if (obs.timestamp > 1e9 && obs.sat_id1 > 0 && obs.sat_id2 > 0 && 
                    obs.sat_id1 != obs.sat_id2 &&
                    std::abs(obs.psr_double_diff) < 5000.0) {
                    
                    observations.push_back(obs);
                    valid_count++;
                    
                    if (valid_count <= 5) {
                        ROS_INFO("DD[%d]: t=%.6f, sats=(%d,%d), psr_dd=%.3f (computed)", 
                                 valid_count, obs.timestamp, obs.sat_id1, obs.sat_id2, 
                                 obs.psr_double_diff);
                    }
                }
            } else {
                // åˆ—æ•°ä¸åŒ¹é…
                if (line_count <= 10) {
                    ROS_WARN("Line %d: unexpected column count: %zu (expected >= %d)", 
                             line_count, tokens.size(), is_results_file ? 5 : 12);
                }
            }
            
        } catch (const std::exception& e) {
            parse_errors++;
            if (parse_errors <= 10) {
                ROS_WARN("Parse error at line %d: %s", line_count, e.what());
                ROS_WARN("  Line content: %s", line.substr(0, 100).c_str());
            }
        }
        
        // æ˜¾ç¤ºè¿›åº¦
        if (line_count % 10000 == 0) {
            ROS_INFO("Processing line %d, valid observations: %d", line_count, valid_count);
        }
        
        // é™åˆ¶åŠ è½½æ•°é‡ç”¨äºŽæµ‹è¯•
        if (valid_count >= 63000) {
            ROS_INFO("Reached maximum load limit (63000), stopping...");
            break;
        }
    }
    
    file.close();
    
    // æ˜¾ç¤ºè¯¦ç»†ç»Ÿè®¡
    ROS_INFO("=====================================");
    ROS_INFO("DD Data Loading Summary:");
    ROS_INFO("  Total lines processed: %d", line_count);
    ROS_INFO("  Valid observations loaded: %d", valid_count);
    ROS_INFO("  Parse errors: %d", parse_errors);
    ROS_INFO("  Rejected (timestamp): %d", rejected_timestamp);
    ROS_INFO("  Rejected (satellite ID): %d", rejected_satid);
    ROS_INFO("  Rejected (quality): %d", rejected_quality);
    ROS_INFO("=====================================");
    
    if (!observations.empty()) {
        double duration = observations.back().timestamp - observations.front().timestamp;
        ROS_INFO("Time range: [%.6f, %.6f] (%.1f sec)", 
                 observations.front().timestamp, observations.back().timestamp, duration);
        
        // æ˜¾ç¤ºç»Ÿè®¡ä¿¡æ¯
        double avg_psr_dd = 0;
        double max_psr_dd = 0;
        for (const auto& obs : observations) {
            double psr_abs = std::abs(obs.psr_double_diff);
            avg_psr_dd += psr_abs;
            max_psr_dd = std::max(max_psr_dd, psr_abs);
        }
        avg_psr_dd /= observations.size();
        
        ROS_INFO("DD statistics:");
        ROS_INFO("  Average |PSR_DD|: %.3f m", avg_psr_dd);
        ROS_INFO("  Max |PSR_DD|: %.3f m", max_psr_dd);
    }
    
    return valid_count > 0;
}

void GnssDDManager::addDDFactorToGraph(ceres::Problem& problem, 
                                       const ObsData& obs1, const ObsData& obs2,
                                       double* Pi, double* Pj, 
                                       double* ref_ecef, double quality_coeff) {
    auto dd_factor = new GnssDDFactor(obs1, quality_coeff);
    problem.AddResidualBlock(dd_factor, nullptr, Pi, Pj, ref_ecef);
}