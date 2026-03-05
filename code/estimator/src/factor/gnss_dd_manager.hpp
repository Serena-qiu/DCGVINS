#ifndef GNSS_DD_MANAGER_H_
#define GNSS_DD_MANAGER_H_

#include <map>
#include <vector>
#include <memory>
#include <queue>
#include <Eigen/Dense>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include "gnss_dd_factor.hpp"
#include "gnss_dd_quality.hpp"
#include <string>
#include <ceres/ceres.h>

#ifndef OBS_DATA_DEFINED
#define OBS_DATA_DEFINED
struct ObsData2 {
    double timestamp;
    int sat_id1, sat_id2;
    double psr1, psr2;
    double freq1, freq2;
};

struct DDResult {
    double timestamp;
    double psr_dd;
    double quality;
};
#endif // OBS_DATA_DEFINED

class GnssDDManager
{
public:
    struct DDMeasurement {
        double timestamp;
        int drone_i_id;
        int drone_j_id;
        int sat1_id;
        int sat2_id;
        uint32_t system;  // GPS, GLO, etc.
        double psr_dd;
        double quality_score;
    };
    
    struct DronePairData {
        int drone_i_id;
        int drone_j_id;
        std::map<std::pair<int, int>, GnssDDQualityChecker::SatellitePairMetrics> satellite_pairs;
        double overall_quality_score;
        int measurement_count;
    };

    // æ•°æ®æˆå‘˜
    std::vector<ObsData> observations;
    std::vector<DDResult> results;
    
    // æž„é€ å‡½æ•° - ç§»é™¤æ­§ä¹‰
    explicit GnssDDManager(const int max_history_size = 1000);

    // åŠ è½½è§‚æµ‹æ•°æ®
    bool loadObservations(const std::string& data_file, const std::string& results_file);
    
    // æ·»åŠ åŒå·®å› å­åˆ°ä¼˜åŒ–å›¾
    void addDDFactorToGraph(ceres::Problem& problem, 
                            const ObsData& obs1, const ObsData& obs2,
                            double* Pi, double* Pj, 
                            double* ref_ecef, double quality_coeff);

    // Add observations from multiple drones and create DD factors
    void processObservations(const std::map<int, std::vector<ObsPtr>>& drone_observations,
                           const std::map<int, EphemBasePtr>& ephemeris_data,
                           std::vector<std::shared_ptr<GnssDDFactor>>& dd_factors);
    
    // Get quality metrics for drone pair
    bool getDronePairQuality(int drone_i_id, int drone_j_id, 
                           DronePairData& pair_data) const;
    
    // Update historical quality database
    void updateQualityDatabase(const DDMeasurement& measurement);
    
    // Select best satellite pairs for double differencing
    std::vector<std::pair<int, int>> selectBestSatellitePairs(
        const std::vector<ObsPtr>& obs_i,
        const std::vector<ObsPtr>& obs_j,
        const int max_pairs = 5);
        
private:
    // è´¨é‡ç®¡ç†
    GnssDDQualityChecker quality_checker_;
    
    // æ— äººæœºå¯¹çš„åŽ†å²è´¨é‡æ•°æ®
    std::map<std::pair<int, int>, DronePairData> drone_pair_history_;
    
    // æµ‹é‡åŽ†å²é˜Ÿåˆ—ï¼ˆç”¨äºŽæ»‘åŠ¨çª—å£ç»Ÿè®¡ï¼‰
    std::queue<DDMeasurement> measurement_history_;
    int max_history_size_;
    
    // æŸ¥æ‰¾å…±åŒå¯è§å«æ˜Ÿ
    void findCommonSatellites(const std::vector<ObsPtr>& obs_i,
                            const std::vector<ObsPtr>& obs_j,
                            std::vector<int>& common_sats);
    
    // è®¡ç®—åŒå·®æµ‹é‡å€¼
    bool computeDoubleDifference(const ObsPtr& obs1_i, const ObsPtr& obs1_j,
                                const ObsPtr& obs2_i, const ObsPtr& obs2_j,
                                double& psr_dd);
    
    // è¯„ä¼°å«æ˜Ÿå¯¹è´¨é‡
    double evaluateSatellitePairQuality(int sat1_id, int sat2_id,
                                      const DronePairData& drone_pair);
    
    // æ¸…ç†è¿‡æœŸçš„åŽ†å²æ•°æ®
    void cleanupOldMeasurements();
    
    // æ›´æ–°æ— äººæœºå¯¹çš„ç»Ÿè®¡ä¿¡æ¯
    void updateDronePairStatistics(int drone_i_id, int drone_j_id,
                                 const DDMeasurement& measurement);
};

#endif // GNSS_DD_MANAGER_H_