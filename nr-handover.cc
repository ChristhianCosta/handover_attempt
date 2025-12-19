#include "ns3/flow-monitor-module.h"

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/buildings-helper.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-module.h"
#include "ns3/nr-helper.h"
#include "ns3/nr-mac-scheduler-tdma-rr.h"
#include "ns3/nr-module.h"
#include "ns3/nr-point-to-point-epc-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/nr-handover-algorithm.h"
#include <fstream>

#include <filesystem> // Quero mover os arquivos de trace depois de gerados
namespace fs = std::filesystem; // apelido pra digitar menos

using namespace ns3;

#include <fstream>

void organizar(std::string caminho_res);


void LogRsrp(Ptr<NrUePhy> phy) {
    static bool firstWrite = true; // Flag to track the first write in this run

    std::ios_base::openmode mode = (firstWrite) ? std::ios::out : std::ios::app; 
    std::ofstream file("measurements.txt", mode); // Open file with chosen mode

    if (file.is_open()) {
        // If it's the first write, add the header
        if (firstWrite) {
            file << "time\tRSRP\n";
            firstWrite = false; // Change flag after first write
        }

        // Write the time and RSRP value
        file << Simulator::Now().As(Time::S) << "\t" << phy->GetRsrp() << std::endl;
        file.close(); // Close after writing
    } else {
        std::cerr << "Error: Unable to open measurements.txt\n";
    }

    Simulator::Schedule(Seconds(0.01), &LogRsrp, phy);
}


int
main(int argc, char* argv[])
{
    std::string scenario = "UMa"; // scenario
    double frequency = 28e9;      // central frequency
    double bandwidth = 100e6;     // bandwidth
    double mobility = true;       // enable mobility
    double simTime = 7;           // in second
    double speed = 15;            // UE speed increased to 10 m/s
    double hBS;          // base station antenna height
    double hUT;          // user antenna height
    double txPower = 40; // txPower
    bool logging = true;

    CommandLine cmd(__FILE__);
    cmd.AddValue("scenario",
                 "The scenario for the simulation. Choose among 'RMa', 'UMa', 'UMi', "
                 "'InH-OfficeMixed', 'InH-OfficeOpen'.",
                 scenario);
    cmd.AddValue("frequency", "The central carrier frequency in Hz.", frequency);
    cmd.AddValue("mobility",
                 "Enable UE mobility (1) or static UEs (0)",
                 mobility);
    cmd.AddValue("logging", "Enable logging (1) or disable (0)", logging);
    cmd.Parse(argc, argv);
    
    if (logging)
    {
        LogComponentEnable("ThreeGppPropagationLossModel", LOG_LEVEL_WARN);
    }
 
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));
 
    // Set antenna heights based on scenario
    if (scenario == "RMa")
    {
        hBS = 35;
        hUT = 1.5;
    }
    else if (scenario == "UMa")
    {
        hBS = 25;
        hUT = 1.5;
    }
    else if (scenario == "UMi-StreetCanyon")
    {
        hBS = 10;
        hUT = 1.5;
    }
    else if (scenario == "InH-OfficeMixed" || scenario == "InH-OfficeOpen")
    {
        hBS = 3;
        hUT = 1;
    }
    else
    {
        NS_ABORT_MSG("Scenario not supported. Choose among 'RMa', 'UMa', 'UMi', "
                     "'InH-OfficeMixed', and 'InH-OfficeOpen'.");
    }
 
    // Create nodes: 1 gNB and 2 UEs
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    gnbNodes.Create(2);  // Single gNodeB
    ueNodes.Create(1);   // Single UE
 
    // Position the gNB
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(0.0, 0.0, hBS));  // First gNB at origin
    gnbPositionAlloc->Add(Vector(0.0, 100.0, hBS)); // Second gNB at (0, 100)
    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);
 
    // Configure UE mobility with 10 m/s speed
    MobilityHelper ueMobility;
    ueMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    ueMobility.Install(ueNodes);
 
    if (mobility)
    {
        // UE0 position and velocity (10 m/s along Y-axis)
        ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(50, 10, hUT));
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, speed, 0));
    }
    else
    {
        // Static positions if mobility disabled
        ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(50, 10, hUT));
    }
 
    // Create NR helpers
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Configure handover parameters
    nrHelper->SetHandoverAlgorithmType("ns3::A2A4RsrqHandoverAlgorithm");
    nrHelper->SetHandoverAlgorithmAttribute("ServingCellThreshold", UintegerValue(30));  // 30 dB
    nrHelper->SetHandoverAlgorithmAttribute("NeighbourCellOffset", UintegerValue(5));    // 5 dB offset
    // nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm"); // A3 RSRP-based handover
    //nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(3.0));            // 3 dB hysteresis when using A3
    //nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(100))); // 100 ms

    // Configure other helpers
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);
 
    // Spectrum configuration
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;
    
    CcBwpCreator::SimpleOperationBandConf bandConf(frequency, bandwidth, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    
    // Channel configuration
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    channelHelper->ConfigureFactories(scenario, "Default", "ThreeGpp");
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});
 
    // Beamforming and scheduler
    idealBeamformingHelper->SetAttribute("BeamformingMethod",
                                         TypeIdValue(DirectPathBeamforming::GetTypeId()));
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());
 
    // Antenna configurations
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
 
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement", PointerValue(CreateObject<IsotropicAntennaModel>()));
 
    // Install NR devices
    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);
 
    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);
 
    // Set TX power for gNB
    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        nrHelper->GetGnbPhy(gnbNetDev.Get(i), 0)->SetTxPower(txPower);
    }

    // Create internet stack and assign IP addresses
    auto [remoteHost, remoteHostIpv4Address] = nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.010));
    InternetStackHelper internet;
    internet.Install(ueNodes);
    Ipv4InterfaceContainer ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));
 
    // Create UDP applications
    uint16_t dlPort = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        // UDP server on each UE
        UdpServerHelper dlPacketSinkHelper(dlPort);
        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));
 
        // UDP client sending to UE
        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(1)));
        //dlClient.SetAttribute("MaxPackets", UintegerValue(10));
        dlClient.SetAttribute ("MaxPackets", UintegerValue(0xFFFFFFFF));

        dlClient.SetAttribute("PacketSize", UintegerValue(1500));
        clientApps.Add(dlClient.Install(remoteHost));
    }
 
    // Attach UEs to the single gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
    Ptr<Node> ueNode = ueNodes.Get(0);
    Ptr<NrUeNetDevice> ueNetDevice = ueNode->GetDevice(0)->GetObject<NrUeNetDevice>();
    if (ueNetDevice) {
        Ptr<NrUePhy> uePhy = ueNetDevice->GetPhy(0);
	Simulator::Schedule(Seconds(0.5),&LogRsrp,uePhy);
    }

    // Start applications
    serverApps.Start(Seconds(0.4));
    clientApps.Start(Seconds(0.4));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime - 0.2));
 
    // Enable traces
    nrHelper->EnableTraces();

    //configuração do flowmonitor

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    monitor->CheckForLostPackets();
    
    // Run simulation
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    std::string tr_name("/home/christhian/5g/ns-3-dev/scratch/results/ex_nrHandover");


    monitor->SerializeToXmlFile(tr_name + ".xml", true, true);

 
    // Check received packets on first UE
    Ptr<UdpServer> serverApp = serverApps.Get(0)->GetObject<UdpServer>();
    uint64_t receivedPackets = serverApp->GetReceived();
 
    Simulator::Destroy();

    organizar("scratch/results/nrHandover/");

 
    return (receivedPackets == 10) ? EXIT_SUCCESS : EXIT_FAILURE;
}

void organizar(std::string caminho_res){
    std::vector<std::string> traceFiles = {
        "DlPhyTrace.txt",
        "UlPhyTrace.txt",
        "RxPacketTrace.txt",
        "TxPacketTrace.txt",
        "DlRxPhyStats.txt",
        "UlRxPhyStats.txt"
    };

    // Sua pasta de destino (garantindo que existe)
    // Note: tr_name já tem o caminho completo, vamos extrair o diretório ou usar a string direta
    //std::string outputDir = "scratch/results/ex005/"; 

    std::string outputDir = caminho_res;

    // Cria o diretório se não existir
    try {
        if (!fs::exists(outputDir)) {
            fs::create_directories(outputDir);
        }

        // Loop para mover cada arquivo
        for (const auto& filename : traceFiles) {
            fs::path sourcePath = filename;              // Arquivo na raiz
            fs::path destPath = outputDir + filename;    // Destino

            if (fs::exists(sourcePath)) {
                // Sobrescreve se já existir lá dentro
                fs::rename(sourcePath, destPath);
                std::cout << "Movido: " << filename << " -> " << outputDir << std::endl;
            }
        }
    }
    catch (const fs::filesystem_error& e) {
        std::cerr << "Erro ao mover arquivos de trace: " << e.what() << std::endl;
    }
}