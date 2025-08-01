// simu71_irs.cc
// This file simulates a mmWave network with UAVs, UEs, and IRS support, including a jammer
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-module.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"
#include "ns3/mmwave-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/config-store.h"
#include "ns3/log.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/channel-condition-model.h" // Include full definition.
#include <fstream>
#include <sstream>
#include <vector>
#include <complex>
#include "ns3/three-gpp-propagation-loss-model.h"

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE("MmwaveUavIrsSimulation");

static std::vector<std::ofstream> g_rssiFiles;
static std::vector<std::ofstream> g_throughputFiles;

void PrintSinr(std::string context, double sinr)
{
    std::cout << Simulator::Now().GetSeconds() << "s SINR: " << sinr << " dB" << std::endl;
}

void PrintPathLoss(std::string context, double pathLoss)
{
    std::cout << Simulator::Now().GetSeconds() << "s Path Loss: " << pathLoss << " dB" << std::endl;
}

void PathLossTraceSink(Ptr<const SpectrumPhy> txPhy, Ptr<const SpectrumPhy> rxPhy, double lossDb)
{
    uint32_t rxNodeId = rxPhy->GetDevice()->GetNode()->GetId();
    if (rxNodeId < g_rssiFiles.size() && g_rssiFiles[rxNodeId].is_open())
    {
        g_rssiFiles[rxNodeId] << Simulator::Now().GetSeconds() << "," << (10 - lossDb) << std::endl;
    }
}

void ThroughputTrace(std::string context, Ptr<const Packet> packet)
{
    static std::map<std::string, uint64_t> bytesReceived;
    static std::map<std::string, Time> lastTime;

    bytesReceived[context] += packet->GetSize();
    Time now = Simulator::Now();

    if (lastTime[context].GetSeconds() == 0 || (now - lastTime[context]).GetSeconds() >= 0.1)
    {
        double throughput = (bytesReceived[context] * 8.0) / (now.GetSeconds() * 1000000.0);

        size_t pos = context.find("/NodeList/");
        if (pos != std::string::npos)
        {
            size_t start = pos + 10;
            size_t end = context.find("/", start);
            if (end != std::string::npos)
            {
                uint32_t nodeId = std::stoul(context.substr(start, end - start));
                if (nodeId < g_throughputFiles.size() && g_throughputFiles[nodeId].is_open())
                {
                    g_throughputFiles[nodeId] << now.GetSeconds() << "," << throughput << std::endl;
                }
            }
        }
        lastTime[context] = now;
    }
}

int main(int argc, char *argv[])
{
    uint32_t numUsers = 5;
    double simTime = 1800.0;
    bool enableIrs = true;
    double irsGain = 300.0;      // IRS gain in dB
    uint32_t elementsPerUE = 64; // Number of IRS elements per UE
    double kFactor = 3.0;        // Rician K-factor

    // IRS position parameters
    double irsX = 15.0; // X position of IRS
    double irsY = 7.5;  // Y position of IRS
    double irsZ = 15.0; // Z position of IRS (height)

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUsers", "Number of UE nodes", numUsers);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("enableIrs", "Enable IRS functionality", enableIrs);
    cmd.AddValue("irsGain", "IRS gain in dB", irsGain);
    cmd.AddValue("elementsPerUE", "Number of IRS elements per UE", elementsPerUE);
    cmd.AddValue("kFactor", "Rician K-factor for IRS path", kFactor);
    cmd.AddValue("irsX", "IRS X position", irsX);
    cmd.AddValue("irsY", "IRS Y position", irsY);
    cmd.AddValue("irsZ", "IRS Z position (height)", irsZ);
    cmd.Parse(argc, argv);

    // Enable logging for debugging
    // LogComponentEnable("MmwaveUavIrsSimulation", LOG_LEVEL_INFO);
    // LogComponentEnable("ThreeGppUmaPropagationLossModel", LOG_LEVEL_DEBUG);

    NodeContainer ueNodes, uavNode, jammerNode;
    ueNodes.Create(numUsers);
    uavNode.Create(1);
    jammerNode.Create(1);

    NodeContainer allNodes(ueNodes, uavNode, jammerNode);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    // Position UEs in a line
    for (uint32_t i = 0; i < numUsers; ++i)
        positionAlloc->Add(Vector(0.0 + i * 10.0, 0.0, 1.5)); // UEs at ground level

    positionAlloc->Add(Vector(30, 10, 25)); // UAV at 25m height (3GPP UMa BS height)
    positionAlloc->Add(Vector(15, 5, 10));  // Jammer

    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(ueNodes);
    mobility.Install(uavNode);
    mobility.Install(jammerNode);

    // Print node positions for verification
    std::cout << "=== Node Positions ===" << std::endl;
    for (uint32_t i = 0; i < numUsers; ++i)
    {
        Vector pos = ueNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
        std::cout << "UE " << i << ": (" << pos.x << ", " << pos.y << ", " << pos.z << ")" << std::endl;
    }
    Vector uavPos = uavNode.Get(0)->GetObject<MobilityModel>()->GetPosition();
    std::cout << "UAV: (" << uavPos.x << ", " << uavPos.y << ", " << uavPos.z << ")" << std::endl;
    std::cout << "IRS: (" << irsX << ", " << irsY << ", " << irsZ << ")" << std::endl;

    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
    Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper>();
    mmwaveHelper->SetEpcHelper(epcHelper);

    // Configure IRS-enhanced propagation model if enabled
    if (enableIrs)
    {
        std::cout << "=== Configuring IRS-Enhanced Propagation Model ===" << std::endl;

        // Set the propagation loss model to our custom IRS-enhanced model
        mmwaveHelper->SetPathlossModelType("ns3::ThreeGppUmaPropagationLossModel");

        // Configure IRS attributes
        Config::SetDefault("ns3::ThreeGppUmaPropagationLossModel::IrsPosition",
                           VectorValue(Vector(irsX, irsY, irsZ)));
        Config::SetDefault("ns3::ThreeGppUmaPropagationLossModel::IrsGain",
                           DoubleValue(irsGain));
        Config::SetDefault("ns3::ThreeGppUmaPropagationLossModel::ElementsPerUE",
                           UintegerValue(elementsPerUE));
        Config::SetDefault("ns3::ThreeGppUmaPropagationLossModel::KFactor",
                           DoubleValue(kFactor));

        std::cout << "IRS Configuration:" << std::endl;
        std::cout << "  Position: (" << irsX << ", " << irsY << ", " << irsZ << ")" << std::endl;
        std::cout << "  Base Gain: " << irsGain << " dB" << std::endl;
        std::cout << "  Elements per UE: " << elementsPerUE << std::endl;
        std::cout << "  K-factor: " << kFactor << std::endl;

        // Calculate effective IRS gain
        double effectiveGain = irsGain + 20.0 * std::log10(static_cast<double>(elementsPerUE));
        std::cout << "  Effective Gain: " << effectiveGain << " dB" << std::endl;
    }
    else
    {
        std::cout << "=== Using Standard Propagation Model (No IRS) ===" << std::endl;
        // Use standard 3GPP UMa model without IRS
        mmwaveHelper->SetPathlossModelType("ns3::ThreeGppUmaPropagationLossModel");
    }

    // Set channel condition model
    mmwaveHelper->SetChannelConditionModelType("ns3::ThreeGppUmaChannelConditionModel");

    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(uavNode);
    internet.Install(jammerNode);

    NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice(uavNode);
    NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice(ueNodes);
    NetDeviceContainer jammerDevs = mmwaveHelper->InstallUeDevice(jammerNode);

    mmwaveHelper->AttachToClosestEnb(ueDevs, enbDevs);     // Only attach UEs
    mmwaveHelper->AttachToClosestEnb(jammerDevs, enbDevs); // Attach jammer too

    Ipv4InterfaceContainer ueIps = epcHelper->AssignUeIpv4Address(ueDevs);
    Ipv4InterfaceContainer enbIfaces = epcHelper->AssignUeIpv4Address(enbDevs);
    Ipv4Address uavIp = enbIfaces.GetAddress(0);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

    Ptr<Ipv4StaticRouting> jammerRouting = ipv4RoutingHelper.GetStaticRouting(jammerNode.Get(0)->GetObject<Ipv4>());
    jammerRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);

    // Setup UDP server on UAV
    uint16_t port = 50000;
    UdpServerHelper udpServer(port);
    ApplicationContainer serverApp = udpServer.Install(uavNode);
    serverApp.Start(Seconds(0.0));
    serverApp.Stop(Seconds(simTime));

    // UE clients
    for (uint32_t i = 0; i < numUsers; ++i)
    {
        UdpClientHelper udpClient(uavIp, port);
        udpClient.SetAttribute("MaxPackets", UintegerValue(1000000000));
        udpClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
        udpClient.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer clientApp = udpClient.Install(ueNodes.Get(i));
        clientApp.Start(Seconds(0.0));
        clientApp.Stop(Seconds(simTime));
    }

    // Jammer sends high-rate UDP traffic to simulate interference
    UdpClientHelper jammerClient(Ipv4Address("255.255.255.255"), 9999);
    jammerClient.SetAttribute("MaxPackets", UintegerValue(1000000000));
    jammerClient.SetAttribute("Interval", TimeValue(MicroSeconds(100)));
    jammerClient.SetAttribute("PacketSize", UintegerValue(512));
    ApplicationContainer jammerApp = jammerClient.Install(jammerNode);
    jammerApp.Start(Seconds(0.0));
    jammerApp.Stop(Seconds(simTime));

    g_rssiFiles.resize(allNodes.GetN());
    g_throughputFiles.resize(allNodes.GetN());

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        uint32_t nodeId = ueNodes.Get(i)->GetId();

        std::ostringstream rssiFileName;
        rssiFileName << "mmwave_user" << i << "_rssi.csv";
        g_rssiFiles[nodeId].open(rssiFileName.str(), std::ios_base::out);
        g_rssiFiles[nodeId] << "Time,PathLoss_dB" << std::endl;

        std::ostringstream throughputFileName;
        throughputFileName << "mmwave_user" << i << "_throughput.csv";
        g_throughputFiles[nodeId].open(throughputFileName.str(), std::ios_base::out);
        g_throughputFiles[nodeId] << "Time,Throughput_Mbps" << std::endl;
    }

    Config::ConnectWithoutContext("/ChannelList/*/$ns3::SpectrumChannel/PathLoss",
                                  MakeCallback(&PathLossTraceSink));

    Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpServer/Rx",
                    MakeCallback(&ThroughputTrace));

    // Set high TX power for jammer
    Ptr<MmWaveUeNetDevice> jammerNetDev = jammerDevs.Get(0)->GetObject<MmWaveUeNetDevice>();
    Ptr<MmWaveUePhy> jammerPhy = jammerNetDev->GetPhy();
    jammerPhy->SetTxPower(3000.0); // High jammer power

    // Set lower TX power for UEs
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<MmWaveUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<MmWaveUeNetDevice>();
        ueDev->GetPhy()->SetTxPower(10.0);
    }

    // // Set UAV (eNB) TX power
    // Ptr<MmWaveEnbNetDevice> enbDev = enbDevs.Get(0)->GetObject<MmWaveEnbNetDevice>();
    // Ptr<MmWaveEnbPhy> enbPhy = enbDev->GetPhy();
    // enbPhy->SetTxPower(1000.0); // UAV TX power

    // Enable SINR and path loss tracing
    // Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/MmWaveUePhy/Sinr",
    //                              MakeCallback(&PrintSinr));

    // Enable mmWave traces
    mmwaveHelper->EnableTraces();

    std::cout << "=== Starting Simulation ===" << std::endl;
    std::cout << "Simulation time: " << simTime << " seconds" << std::endl;
    std::cout << "Number of UEs: " << numUsers << std::endl;
    std::cout << "IRS enabled: " << (enableIrs ? "Yes" : "No") << std::endl;

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Print results
    std::cout << "\n=== Simulation Results ===" << std::endl;
    Ptr<UdpServer> server = DynamicCast<UdpServer>(serverApp.Get(0));
    uint32_t totalPacketsReceived = server->GetReceived();
    std::cout << "Total packets received by UAV: " << totalPacketsReceived << std::endl;

    // Calculate and print throughput
    double throughput = (totalPacketsReceived * 1024 * 8) / (simTime * 1000.0); // kbps
    std::cout << "Average throughput: " << throughput << " kbps" << std::endl;

    for (auto &f : g_rssiFiles)
        if (f.is_open())
            f.close();
    for (auto &f : g_throughputFiles)
        if (f.is_open())
            f.close();

    Simulator::Destroy();
    return 0;
}
