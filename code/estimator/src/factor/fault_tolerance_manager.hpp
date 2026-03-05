#ifndef FAULT_TOLERANCE_MANAGER_H_
#define FAULT_TOLERANCE_MANAGER_H_
#include <map>
#include <ros/ros.h>
#include <vector>

class FaultToleranceManager {
public:
    enum ConnectionState {
        CONNECTED,
        DEGRADED,
        DISCONNECTED
    };
    
    struct AgentStatus {
        int agent_id;
        ConnectionState state;
        double last_message_time;
        double weight_factor;
        int missed_messages;
    };
    
    FaultToleranceManager();
    void updateAgentStatus(int agent_id, double current_time);
    double getConstraintWeight(int agent_id) const;
    void handleReconnection(int agent_id);

    ConnectionState getAgentState(int agent_id) const;
    std::vector<int> getConnectedAgents() const;
    void printStatus() const;
    
private:
    std::map<int, AgentStatus> agent_statuses_;
    const double TIMEOUT_DEGRADED = 0.5;
    const double TIMEOUT_DISCONNECT = 2.0;
    const double WEIGHT_DECAY_RATE = 0.9;
};

#endif // FAULT_TOLERANCE_MANAGER_H_