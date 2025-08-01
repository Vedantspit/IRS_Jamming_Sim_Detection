#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/mmwave-module.h"
#include "ns3/mmwave-point-to-point-epc-helper.h"

using namespace ns3;
using namespace ns3::mmwave;

NS_LOG_COMPONENT_DEFINE("MmwaveUavSimulation");

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
        g_rssiFiles[rxNodeId] << Simulator::Now().GetSeconds() << "," << lossDb << std::endl;
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

    CommandLine cmd(__FILE__);
    cmd.AddValue("numUsers", "Number of UE nodes", numUsers);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.Parse(argc, argv);

    NodeContainer ueNodes, uavNode, jammerNode;
    ueNodes.Create(numUsers);
    uavNode.Create(1);

    NodeContainer allNodes(ueNodes, uavNode);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0; i < numUsers; ++i)
        positionAlloc->Add(Vector(0.0 + i * 10.0, 0.0, 1.5)); // UEs
    positionAlloc->Add(Vector(30, 10, 25));                   // UAV

    mobility.SetPositionAllocator(positionAlloc);
    mobility.Install(ueNodes);
    mobility.Install(uavNode);

    Ptr<MmWaveHelper> mmwaveHelper = CreateObject<MmWaveHelper>();
    Ptr<MmWavePointToPointEpcHelper> epcHelper = CreateObject<MmWavePointToPointEpcHelper>();
    mmwaveHelper->SetEpcHelper(epcHelper);

    InternetStackHelper internet;
    internet.Install(ueNodes);
    internet.Install(uavNode);

    NetDeviceContainer enbDevs = mmwaveHelper->InstallEnbDevice(uavNode);
    NetDeviceContainer ueDevs = mmwaveHelper->InstallUeDevice(ueNodes);

    mmwaveHelper->AttachToClosestEnb(ueDevs, enbDevs); // Only attach UEs

    Ipv4InterfaceContainer ueIps = epcHelper->AssignUeIpv4Address(ueDevs);
    Ipv4InterfaceContainer enbIfaces = epcHelper->AssignUeIpv4Address(enbDevs);
    Ipv4Address uavIp = enbIfaces.GetAddress(0);

    Ipv4StaticRoutingHelper ipv4RoutingHelper;
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting(ueNodes.Get(i)->GetObject<Ipv4>());
        ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
    }

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

    // Set lower TX power for UEs
    for (uint32_t i = 0; i < ueDevs.GetN(); ++i)
    {
        Ptr<MmWaveUeNetDevice> ueDev = ueDevs.Get(i)->GetObject<MmWaveUeNetDevice>();
        ueDev->GetPhy()->SetTxPower(10.0);
    }

    // Enable mmWave traces
    mmwaveHelper->EnableTraces();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    Ptr<UdpServer> server = DynamicCast<UdpServer>(serverApp.Get(0));
    uint32_t totalPacketsReceived = server->GetReceived();
    std::cout << "Total packets received by UAV: " << totalPacketsReceived << std::endl;

    for (auto &f : g_rssiFiles)
        if (f.is_open())
            f.close();
    for (auto &f : g_throughputFiles)
        if (f.is_open())
            f.close();

    Simulator::Destroy();
    return 0;
}
