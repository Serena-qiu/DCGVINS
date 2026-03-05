#include "gnss_dd_quality.hpp"
#include <cmath>
#include <numeric>

void GnssDDQualityChecker::analyzeDDQuality(const std::vector<double>& psr_dd_values,
                                          DDQualityMetrics& metrics)
{
    if (psr_dd_values.empty())
    {
        metrics.is_good_quality = false;
        metrics.quality_score = 0.0;
        return;
    }
    
    // Calculate means
    metrics.mean_psr_dd = std::accumulate(psr_dd_values.begin(), psr_dd_values.end(), 0.0) / psr_dd_values.size();
    
    // Calculate standard deviations
    metrics.psr_std = calculateStandardDeviation(psr_dd_values);
    
    metrics.sample_count = psr_dd_values.size();
    
    // Evaluate quality based on statistics (reflecting your data analysis)
    metrics.quality_score = evaluateQualityScore(metrics);
    metrics.is_good_quality = metrics.quality_score > 0.7;
}

bool GnssDDQualityChecker::isSatellitePairSuitable(const SatellitePairMetrics& pair_metrics)
{
    // Based on your statistical analysis, all satellite pairs showed poor quality
    // However, we still need to differentiate relative quality
    
    if (pair_metrics.metrics.sample_count < MIN_SAMPLES_FOR_ASSESSMENT)
    {
        // Not enough data to assess - assume medium quality
        return true;
    }
    
    // Check if the pair has reasonable standard deviation
    bool has_reasonable_std = (pair_metrics.metrics.psr_std < VERY_POOR_QUALITY_STD_THRESHOLD);
    
    return has_reasonable_std;
}

void GnssDDQualityChecker::updateQualityMetrics(SatellitePairMetrics& metrics,
                                              const double new_psr_dd)
{
    // 使用Welford算法进行增量更新
    metrics.metrics.sample_count++;
    double delta = new_psr_dd - metrics.metrics.mean_psr_dd;
    metrics.metrics.mean_psr_dd += delta / metrics.metrics.sample_count;
    double delta2 = new_psr_dd - metrics.metrics.mean_psr_dd;
    metrics.variance_sum_ += delta * delta2;
    
    // 计算标准差
    if (metrics.metrics.sample_count > 1) {
        metrics.metrics.psr_std = std::sqrt(metrics.variance_sum_ / (metrics.metrics.sample_count - 1));
    } else {
        metrics.metrics.psr_std = 0.0;
    }
    
    metrics.metrics.quality_score = evaluateQualityScore(metrics.metrics);
    metrics.metrics.is_good_quality = metrics.metrics.quality_score > 0.7;
}
double GnssDDQualityChecker::getQualityWeight(const DDQualityMetrics& metrics)
{
    // Based on your statistics, most satellite pairs show poor quality
    // We need to adapt our weighting scheme to reflect this reality
    
    if (metrics.sample_count < MIN_SAMPLES_FOR_ASSESSMENT)
    {
        // Default weight for insufficient data
        return 0.5;
    }
    
    // Create a relative weighting scheme based on your actual data
    // Since most std values are 10-30m, we'll adjust expectations
    double weight = 1.0;
    
    // Adjust weight based on standard deviation
    if (metrics.psr_std < 10.0)
    {
        weight = 1.0;  // Relatively good for urban canyon
    }
    else if (metrics.psr_std < 20.0)
    {
        weight = 0.7;  // Medium quality
    }
    else if (metrics.psr_std < 30.0)
    {
        weight = 0.4;  // Poor quality but still usable
    }
    else
    {
        weight = 0.2;  // Very poor quality
    }
    
    // Scale by overall quality score
    weight *= metrics.quality_score;
    
    return weight;
}

double GnssDDQualityChecker::calculateStandardDeviation(const std::vector<double>& values)
{
    if (values.size() < 2) return 0.0;
    
    double mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    double sq_sum = std::inner_product(values.begin(), values.end(), values.begin(), 0.0);
    return std::sqrt(sq_sum / values.size() - mean * mean);
}

double GnssDDQualityChecker::evaluateQualityScore(const DDQualityMetrics& metrics)
{
    // Based on your data analysis, we need to adapt the quality scoring
    // Most satellite pairs in your data show std > 10m
    
    double score = 1.0;
    
    // Adjust expectations based on actual data
    if (metrics.psr_std < 5.0)
    {
        score = 1.0;  // Excellent (rare in urban canyons)
    }
    else if (metrics.psr_std < 15.0)
    {
        score = 0.8;  // Good for urban environment
    }
    else if (metrics.psr_std < 25.0)
    {
        score = 0.6;  // Acceptable for urban environment
    }
    else if (metrics.psr_std < 35.0)
    {
        score = 0.4;  // Poor but potentially usable
    }
    else
    {
        score = 0.2;  // Very poor
    }
    
    // Adjust for consistency
    double sample_factor = std::min(1.0, 
        static_cast<double>(metrics.sample_count) / (3.0 * MIN_SAMPLES_FOR_ASSESSMENT));
    
    score *= sample_factor;
    
    return score;
}

void GnssDDQualityChecker::updateVarianceWelford(double& mean, double& var_sum, 
                                                 int& count, double new_value) {
    count++;
    double delta = new_value - mean;
    mean += delta / count;
    double delta2 = new_value - mean;
    var_sum += delta * delta2;
}