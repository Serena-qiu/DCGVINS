#ifndef TIME_SYNCHRONIZER_H_
#define TIME_SYNCHRONIZER_H_

#include <vector>
#include <map>
#include <Eigen/Dense>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_constant.hpp>

using namespace gnss_comm;

class TimeSynchronizer {
public:
    struct SyncResult {
        bool is_synchronized;
        double time_offset;
        double sync_quality;
        
        SyncResult() : is_synchronized(false), time_offset(0.0), sync_quality(0.0) {}
    };
    
    TimeSynchronizer() = default;
    
    SyncResult synchronizeByCommonView(
        const std::vector<ObsPtr>& obs1,
        const std::vector<ObsPtr>& obs2,
        const std::vector<EphemBasePtr>& ephems);
        
private:
    struct ClockFilter {
        double clock_offset;
        double clock_drift;
        Eigen::Matrix2d P;
        
        ClockFilter() : clock_offset(0.0), clock_drift(0.0) {
            P.setIdentity();
        }
        
        void predict(double dt);
        void update(double measured_offset, double measurement_var);
    };
    
    std::map<std::pair<int,int>, ClockFilter> clock_filters_;
};

#endif // TIME_SYNCHRONIZER_H_