#ifndef GNSS_DD_QUALITY_H_
#define GNSS_DD_QUALITY_H_

#include <vector>
#include <map>
#include <gnss_comm/gnss_utility.hpp>

using namespace gnss_comm;

class GnssDDQualityChecker {
public:
    // Add the missing DDQualityMetrics structure
    struct DDQualityMetrics {
        double mean_psr_dd;
        double psr_std;
        int sample_count;
        double quality_score;
        bool is_good_quality;
        
        DDQualityMetrics() : mean_psr_dd(0), psr_std(0), sample_count(0), 
                             quality_score(0), is_good_quality(false) {}
    };
    
    // Add the missing SatellitePairMetrics structure
    struct SatellitePairMetrics {
        DDQualityMetrics metrics;
        double variance_sum_;
        
        SatellitePairMetrics() : variance_sum_(0) {}
    };
    
    static constexpr int MIN_SAMPLES_FOR_ASSESSMENT = 10;
    static constexpr double VERY_POOR_QUALITY_STD_THRESHOLD = 50.0;
    
    GnssDDQualityChecker() = default;
    
    void analyzeDDQuality(const std::vector<double>& psr_dd_values,
                         DDQualityMetrics& metrics);
    
    bool isSatellitePairSuitable(const SatellitePairMetrics& pair_metrics);
    
    void updateQualityMetrics(SatellitePairMetrics& metrics,
                             const double new_psr_dd);
    
    double getQualityWeight(const DDQualityMetrics& metrics);
    
private:
    double calculateStandardDeviation(const std::vector<double>& values);
    double evaluateQualityScore(const DDQualityMetrics& metrics);
    void updateVarianceWelford(double& mean, double& var_sum, 
                              int& count, double new_value);
};

#endif // GNSS_DD_QUALITY_H_