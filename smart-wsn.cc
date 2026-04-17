#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lr-wpan-module.h"
#include "ns3/internet-module.h"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("PbomSimulation");

// --- Global Constants from the Paper ---
const double AREA_SIZE = 800.0;
const uint32_t NUM_CHS = 6;
const double CH_COMM_RADIUS = 80.0;
const double MDC_MAX_SPEED = 30.0;
const double MDC_MIN_SPEED = 0.5;

// Buffer and Forecasting Parameters
const double BUFFER_CAPACITY = 2000.0;
const double ALPHA = 0.3; // Smoothing factor
const double THETA_SAFE = 0.7; // Safe fill fraction for offloading
const double DELTA_SAFETY_MARGIN = 30.0; // seconds
const double DT = 1.0; // Observation interval for forecasting

// Priority Weights (w1 + w2 + w3 = 1)
const double W1 = 0.5; // Urgency (TTO)
const double W2 = 0.3; // Criticality (Fill fraction)
const double W3 = 0.2; // Proximity (Distance)
const double P_CRIT = 0.6; // Priority threshold for reordering

// --- Energy Model Constants ---
const double INITIAL_ENERGY = 5000.0; // Joules
const double E_RX_PER_PACKET = 0.01; // J (receiving from member nodes)
const double E_TX_PER_PACKET = 0.02; // J (transmitting to MDC)
const double E_OFFLOAD_PER_PACKET = 0.05; // J (higher cost for inter-CH transmission)

// --- State Structures ---
struct ClusterHead {
    uint32_t id;
    Ptr<Node> node;
    Vector position;
    
    // Buffer metrics
    double occupancy; // q_i
    double lastOccupancy;
    double fillRate;  // f_i
    double tto;       // time-to-overflow
    
    // Traffic generation rate (heterogeneous as per paper)
    double dataArrivalRate; // packets/sec
    
    // Energy state
    double residualEnergy; // Joules
};

struct MobileDataCollector {
    Ptr<Node> node;
    Ptr<ConstantVelocityMobilityModel> mobility;
    uint32_t currentTargetId;
    std::vector<uint32_t> visitQueue;
    double currentSpeed;
};

// Global State
std::vector<ClusterHead> chList(NUM_CHS);
MobileDataCollector mdc;
uint32_t overflowEvents = 0;
uint32_t collectedPackets = 0;
EventId nextMdcUpdateEvent; 
std::ofstream metricsFile;
std::string simMode = "pbom"; // Added global mode variable

// --- Function Prototypes ---
void GenerateTraffic();
void UpdateForecasting();
void EvaluateMdcPriority();
void TriggerCooperativeOffloading(uint32_t sourceChId);
void CollectData();
void UpdateMdcMobility();
void RecordMetrics();
double CalculateDistance2D(Vector a, Vector b);

// --- Core Logic Implementation ---

double CalculateDistance2D(Vector a, Vector b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// Logs simulation data to CSV for graphing
void RecordMetrics() {
    double totalEnergy = 0.0;
    for (const auto& ch : chList) {
        totalEnergy += std::max(0.0, ch.residualEnergy);
    }
    double avgEnergy = totalEnergy / NUM_CHS;

    if (metricsFile.is_open()) {
        metricsFile << Simulator::Now().GetSeconds() << "," 
                    << overflowEvents << "," 
                    << collectedPackets << ","
                    << avgEnergy << "\n";
    }
    Simulator::Schedule(Seconds(10.0), &RecordMetrics);
}

// Simulates heterogeneous sensor data arriving at the CH buffers
void GenerateTraffic() {
    for (uint32_t i = 0; i < NUM_CHS; ++i) {
        double packetsArrived = chList[i].dataArrivalRate * DT;
        chList[i].occupancy += packetsArrived;
        
        // Energy consumption for receiving data
        chList[i].residualEnergy -= (packetsArrived * E_RX_PER_PACKET);
        
        // Check for actual overflow
        if (chList[i].occupancy >= BUFFER_CAPACITY) {
            overflowEvents++;
            NS_LOG_INFO("Time " << Simulator::Now().GetSeconds() << "s: OVERFLOW at CH " << i << "!");
            chList[i].occupancy = BUFFER_CAPACITY; // Cap at max
        }
    }
    Simulator::Schedule(Seconds(DT), &GenerateTraffic);
}

// Component 1: CH-Level Buffer Occupancy Forecasting (Eq. 2 & 3)
void UpdateForecasting() {
    for (uint32_t i = 0; i < NUM_CHS; ++i) {
        double deltaQ = chList[i].occupancy - chList[i].lastOccupancy;
        
        // Eq 2: Exponential smoothing of fill rate
        chList[i].fillRate = ALPHA * (deltaQ / DT) + (1.0 - ALPHA) * chList[i].fillRate;
        chList[i].lastOccupancy = chList[i].occupancy;
        
        // Eq 3: Time-to-overflow (TTO)
        if (chList[i].fillRate > 0) {
            chList[i].tto = (BUFFER_CAPACITY - chList[i].occupancy) / chList[i].fillRate;
        } else {
            chList[i].tto = std::numeric_limits<double>::infinity();
        }

        // Eq 4: Dynamic threshold check
        Vector mdcPos = mdc.mobility->GetPosition();
        double dist = CalculateDistance2D(mdcPos, chList[i].position);
        double tTravel = dist / MDC_MAX_SPEED;
        double tService = chList[i].occupancy / 50.0; // Assuming 50 pkt/s collection rate
        double tauWarn = tTravel + tService + DELTA_SAFETY_MARGIN;

        if (chList[i].tto < tauWarn) {
            // Check if MDC can even make it in time. If not, trigger offloading.
            if (chList[i].tto < tTravel && simMode == "pbom") {
                TriggerCooperativeOffloading(i);
            } else if (simMode == "pbom") {
                // Broadcast early warning to MDC
                Simulator::ScheduleNow(&EvaluateMdcPriority);
            }
        }
    }
    Simulator::Schedule(Seconds(DT), &UpdateForecasting);
}

// Component 2: Priority-Aware Dynamic Scheduling (Eq. 5 & Algorithm 1)
void EvaluateMdcPriority() {
    Vector mdcPos = mdc.mobility->GetPosition();
    int bestChId = -1;
    double highestPriority = 0.0;

    for (uint32_t i = 0; i < NUM_CHS; ++i) {
        if (chList[i].tto == std::numeric_limits<double>::infinity() || chList[i].tto <= 0) continue;

        double dRemain = CalculateDistance2D(mdcPos, chList[i].position);
        if (dRemain < 1.0) dRemain = 1.0; // Avoid division by zero
        
        // Eq 5: Priority Score
        double priority = W1 * (1.0 / chList[i].tto) + 
                          W2 * (chList[i].occupancy / BUFFER_CAPACITY) + 
                          W3 * (100.0 / dRemain); // scaled distance for balance
        
        if (priority > P_CRIT && priority > highestPriority) {
            highestPriority = priority;
            bestChId = i;
        }
    }

    // Dynamic Reordering
    if (bestChId != -1 && bestChId != (int)mdc.currentTargetId) {
        NS_LOG_INFO("Time " << Simulator::Now().GetSeconds() << "s: MDC rerouting to high-priority CH " << bestChId << " (Score: " << highestPriority << ")");
        
        // Remove from current queue and push to front
        auto it = std::find(mdc.visitQueue.begin(), mdc.visitQueue.end(), bestChId);
        if (it != mdc.visitQueue.end()) mdc.visitQueue.erase(it);
        
        mdc.visitQueue.insert(mdc.visitQueue.begin(), bestChId);
        mdc.currentTargetId = bestChId;
        
        // Trigger trajectory update, cancel old loop
        if (nextMdcUpdateEvent.IsRunning()) {
            Simulator::Cancel(nextMdcUpdateEvent);
        }
        Simulator::ScheduleNow(&UpdateMdcMobility);
    }
}

// Steers the MDC dynamically towards the current target CH
void UpdateMdcMobility() {
    if (mdc.visitQueue.empty()) return;

    mdc.currentTargetId = mdc.visitQueue.front();
    Vector mdcPos = mdc.mobility->GetPosition();
    Vector targetPos = chList[mdc.currentTargetId].position;

    double dx = targetPos.x - mdcPos.x;
    double dy = targetPos.y - mdcPos.y;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist <= 5.0) { // Catch edges safely
        // Arrived at CH, collect data and move to next
        mdc.mobility->SetVelocity(Vector(0, 0, 0));
        Simulator::ScheduleNow(&CollectData);
        return;
    }

    // Calculate normalized velocity vector (Eq 6 simplified for simulation space)
    double vx = (dx / dist) * MDC_MAX_SPEED;
    double vy = (dy / dist) * MDC_MAX_SPEED;
    
    mdc.mobility->SetVelocity(Vector(vx, vy, 0.0));
    
    // Recheck mobility soon (0.1s interval prevents overshooting!)
    if (nextMdcUpdateEvent.IsRunning()) {
        Simulator::Cancel(nextMdcUpdateEvent);
    }
    nextMdcUpdateEvent = Simulator::Schedule(Seconds(0.1), &UpdateMdcMobility);
}

// Data collection when MDC reaches CH
void CollectData() {
    uint32_t targetId = mdc.currentTargetId;
    double packetsToCollect = chList[targetId].occupancy;
    
    if (packetsToCollect > 0) {
        collectedPackets += packetsToCollect;
        chList[targetId].occupancy = 0;
        chList[targetId].lastOccupancy = 0;
        
        // Energy consumption for transmitting data to MDC
        chList[targetId].residualEnergy -= (packetsToCollect * E_TX_PER_PACKET);
        
        NS_LOG_INFO("Time " << Simulator::Now().GetSeconds() << "s: MDC collected " << packetsToCollect << " packets from CH " << targetId);
    }

    // Cycle the visit queue
    if (!mdc.visitQueue.empty() && mdc.visitQueue.front() == targetId) {
        mdc.visitQueue.erase(mdc.visitQueue.begin());
        mdc.visitQueue.push_back(targetId); // Re-add to end of patrol loop
    }
    
    // Resume mobility towards the next target
    if (nextMdcUpdateEvent.IsRunning()) {
        Simulator::Cancel(nextMdcUpdateEvent);
    }
    Simulator::ScheduleNow(&UpdateMdcMobility);
}

// Component 3: Cooperative Data Offloading
void TriggerCooperativeOffloading(uint32_t sourceChId) {
    double nOff = chList[sourceChId].occupancy - (BUFFER_CAPACITY * THETA_SAFE);
    if (nOff <= 0) return;

    int bestNeighborId = -1;
    double maxAvailableCap = 0;

    // Find neighbor with largest available capacity
    for (uint32_t j = 0; j < NUM_CHS; ++j) {
        if (j == sourceChId) continue;
        
        double dist = CalculateDistance2D(chList[sourceChId].position, chList[j].position);
        if (dist <= CH_COMM_RADIUS * 2) { // Assuming multi-hop or overlapping ranges
            double availableCap = BUFFER_CAPACITY - chList[j].occupancy;
            if (availableCap > maxAvailableCap) {
                maxAvailableCap = availableCap;
                bestNeighborId = j;
            }
        }
    }

    // Perform offload
    if (bestNeighborId != -1 && maxAvailableCap >= nOff) {
        chList[sourceChId].occupancy -= nOff;
        chList[bestNeighborId].occupancy += nOff;
        
        // Energy consumption for offloading data (more expensive than MDC Tx)
        chList[sourceChId].residualEnergy -= (nOff * E_OFFLOAD_PER_PACKET);
        chList[bestNeighborId].residualEnergy -= (nOff * E_RX_PER_PACKET);
        
        NS_LOG_INFO("Time " << Simulator::Now().GetSeconds() << "s: CH " << sourceChId 
                    << " offloaded " << nOff << " packets to CH " << bestNeighborId);
    }
}

// --- Main Simulation Program ---
int main(int argc, char *argv[]) {
    LogComponentEnable("PbomSimulation", LOG_LEVEL_INFO);

    // Command line argument for topology testing
    std::string topology = "grid";
    CommandLine cmd;
    cmd.AddValue("topology", "Deployment topology: 'grid' or 'random'", topology);
    cmd.AddValue("mode", "Simulation mode: 'pbom' or 'uniform'", simMode);
    cmd.Parse(argc, argv);

    // 1. Create Nodes
    NodeContainer chNodes;
    chNodes.Create(NUM_CHS);
    NodeContainer mdcNode;
    mdcNode.Create(1);

    // 2. Setup 802.15.4 LR-WPAN (Physical/MAC layer from paper)
    LrWpanHelper lrWpanHelper;
    NetDeviceContainer chDevices = lrWpanHelper.Install(chNodes);
    NetDeviceContainer mdcDevice = lrWpanHelper.Install(mdcNode);

    // 3. Setup Mobility
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    
    if (topology == "random") {
        NS_LOG_INFO("Using RANDOM cluster topology.");
        Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
        uv->SetAttribute("Min", DoubleValue(0.0));
        uv->SetAttribute("Max", DoubleValue(AREA_SIZE));
        
        for (uint32_t i = 0; i < NUM_CHS; ++i) {
            alloc->Add(Vector(uv->GetValue(), uv->GetValue(), 0.0));
        }
    } else {
        NS_LOG_INFO("Using GRID cluster topology.");
        alloc->Add(Vector(200.0, 200.0, 0.0));
        alloc->Add(Vector(400.0, 200.0, 0.0));
        alloc->Add(Vector(600.0, 200.0, 0.0));
        alloc->Add(Vector(200.0, 600.0, 0.0));
        alloc->Add(Vector(400.0, 600.0, 0.0));
        alloc->Add(Vector(600.0, 600.0, 0.0));
    }
    
    mobility.SetPositionAllocator(alloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(chNodes);

    // MDC is mobile, uses ConstantVelocity to allow dynamic trajectory steering
    mobility.SetPositionAllocator("ns3::GridPositionAllocator", 
                                  "MinX", DoubleValue(0.0), 
                                  "MinY", DoubleValue(0.0));
    mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    mobility.Install(mdcNode);

    // 4. Initialize States
    double heterogeneousRates[6] = {5.0, 2.0, 10.0, 1.0, 8.0, 0.5}; // pkt/sec

    for (uint32_t i = 0; i < NUM_CHS; ++i) {
        chList[i].id = i;
        chList[i].node = chNodes.Get(i);
        chList[i].position = chNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        chList[i].occupancy = 0.0;
        chList[i].lastOccupancy = 0.0;
        chList[i].fillRate = 0.0;
        chList[i].tto = std::numeric_limits<double>::infinity();
        chList[i].dataArrivalRate = heterogeneousRates[i];
        chList[i].residualEnergy = INITIAL_ENERGY; // Initialize battery
        
        mdc.visitQueue.push_back(i); // Initial circular route
    }

    mdc.node = mdcNode.Get(0);
    mdc.mobility = mdcNode.Get(0)->GetObject<ConstantVelocityMobilityModel>();
    mdc.currentSpeed = MDC_MAX_SPEED;

    // Open Metrics CSV File based on mode
    std::string fileName = simMode + "_metrics.csv";
    metricsFile.open(fileName);
    metricsFile << "Time(s),CumulativeOverflows,CumulativeCollected,AvgResidualEnergy(J)\n";

    // 5. Schedule Simulation Events
    Simulator::Schedule(Seconds(1.0), &GenerateTraffic);
    Simulator::Schedule(Seconds(2.0), &UpdateForecasting);
    Simulator::Schedule(Seconds(10.0), &RecordMetrics); // Export metrics periodically
    nextMdcUpdateEvent = Simulator::Schedule(Seconds(0.1), &UpdateMdcMobility);

    // 6. Run Simulation (24 simulated hours scaled down for testing, e.g., 3600 seconds)
    NS_LOG_INFO("Starting PBOM Simulation in NS-3... Mode: " << simMode);
    Simulator::Stop(Seconds(3600.0)); 
    Simulator::Run();
    Simulator::Destroy();

    // Close Metrics CSV File
    if (metricsFile.is_open()) {
        metricsFile.close();
    }

    // 7. Print Results
    NS_LOG_INFO("--- Simulation Complete ---");
    NS_LOG_INFO("Total Buffer Overflows Prevented/Occurred: " << overflowEvents);
    NS_LOG_INFO("Total Packets Successfully Collected: " << collectedPackets);
    
    double finalEnergySum = 0;
    for (const auto& ch : chList) finalEnergySum += std::max(0.0, ch.residualEnergy);
    NS_LOG_INFO("Average Residual Energy per CH: " << finalEnergySum / NUM_CHS << " Joules");

    return 0;
}
