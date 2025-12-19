// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * @file cttc-3gpp-channel-example.cc
 * @ingroup examples
 * @brief Channel Example
 *
 * This example describes how to setup a simulation using the 3GPP channel model
 * from TR 38.901. Topology consists by default of 2 UEs and 2 gNbs, and can be
 * configured to be either mobile or static scenario.
 *
 * The output of this example are default NR trace files that can be found in
 * the root ns-3 project folder.
 */

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

#include <filesystem> // Quero mover os arquivos de trace depois de gerados
namespace fs = std::filesystem; // apelido pra digitar menos

using namespace ns3;

void ondeTa(NodeContainer ueNodes);
void organizar(std::string caminho_res);
void checaNode(const NetDeviceContainer& gnbNetDev, const NetDeviceContainer& ueNetDev);

int
main(int argc, char* argv[])
{
    //desativar rrc ideal, se não desabilita handover
    //Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    std::string scenario = "UMa"; // scenario
    double frequency = 28e9;      // central frequency
    double bandwidth = 100e6;     // bandwidth
    double mobility = true;      // whether to enable mobility
    double simTime = 5;           // in second
    double speed = 20;             // in m/s for walking UT.
    bool logging = true; // whether to enable logging from the simulation, another option is by
                         // exporting the NS_LOG environment variable
    double hBS;          // base station antenna height in meters
    double hUT;          // user antenna height in meters
    double txPower = 40; // txPower

    //std::string tr_name("/home/christhian/5g/ns-3-dev/scratch/results/ex005");
    std::string tr_name("CAMINHO PARA RESULTADOS");

    CommandLine cmd(__FILE__);
    cmd.AddValue("scenario",
                 "The scenario for the simulation. Choose among 'RMa', 'UMa', 'UMi', "
                 "'InH-OfficeMixed', 'InH-OfficeOpen'.",
                 scenario);
    cmd.AddValue("frequency", "The central carrier frequency in Hz.", frequency);
    cmd.AddValue("mobility",
                 "If set to 1 UEs will be mobile, when set to 0 UE will be static. By default, "
                 "they are mobile.",
                 mobility);
    cmd.AddValue("logging", "If set to 0, log components will be disabled.", logging);
    cmd.Parse(argc, argv);

    // enable logging
    if (logging)
    {

        /**
        msg="Logging component "NrHandoverAlgorithmA3" not found. See above for a list of available log components", file=/home/christhian/5g/ns-3-dev/src/core/model/log.cc, line=293
        NS_FATAL, terminating



         */
        //LogComponentEnable("NrHandoverAlgorithmA3", LOG_LEVEL_INFO);
        //LogComponentEnable("NrGnbRrc", LOG_LEVEL_INFO);

        LogComponentEnable("LteUeRrc", LOG_LEVEL_ALL);
        LogComponentEnable("LteEnbRrc", LOG_LEVEL_ALL);  
        LogComponentEnable("NrA3RsrpHandoverAlgorithm", LOG_LEVEL_ALL);
        LogComponentEnable("NrUePhy", LOG_LEVEL_INFO);
        LogComponentEnable("NrGnbPhy", LOG_LEVEL_INFO);
        LogComponentEnable("UdpClient", LOG_LEVEL_INFO);
    }

    /*
     * Default values for the simulation. We are progressively removing all
     * the instances of SetDefault, but we need it for legacy code (LTE)
     */
    Config::SetDefault("ns3::NrRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    // set mobile device and base station antenna heights in meters, according to the chosen
    // scenario
    if (scenario == "RMa")
    {
        hBS = 35;
        hUT = 1.8;
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

    // create base stations and mobile terminals
    NodeContainer gnbNodes;
    NodeContainer ueNodes;
    //ueNodes.Create(2);
    ueNodes.Create(1);
    gnbNodes.Create(2);



    // position the base stations
    Ptr<ListPositionAllocator> gnbPositionAlloc = CreateObject<ListPositionAllocator>();
    gnbPositionAlloc->Add(Vector(0.0, 0.0, hBS));
    gnbPositionAlloc->Add(Vector(0.0, 80.0, hBS));

    MobilityHelper gnbMobility;
    gnbMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    gnbMobility.SetPositionAllocator(gnbPositionAlloc);
    gnbMobility.Install(gnbNodes);

    // position the mobile terminals and enable the mobility
    MobilityHelper uemobility;
    uemobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    uemobility.Install(ueNodes);

    if (mobility)
    {
        ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(
            Vector(0, 0, hUT)); // (x, y, z) in m
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(0, speed, 0)); // move UE1 along the y axis

        /*   
        ueNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(
            Vector(30, 50.0, hUT)); // (x, y, z) in m
        ueNodes.Get(1)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(
            Vector(-speed, 0, 0)); // move UE2 along the x axis
        */
    }
    else
    {
        ueNodes.Get(0)->GetObject<MobilityModel>()->SetPosition(Vector(90, 15, hUT));
        ueNodes.Get(0)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
        /*
        ueNodes.Get(1)->GetObject<MobilityModel>()->SetPosition(Vector(30, 50.0, hUT));
        ueNodes.Get(1)->GetObject<ConstantVelocityMobilityModel>()->SetVelocity(Vector(0, 0, 0));
        */
        
    }

    /*
     * Create NR simulation helpers
     */
    Ptr<NrPointToPointEpcHelper> nrEpcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<IdealBeamformingHelper> idealBeamformingHelper = CreateObject<IdealBeamformingHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();
    nrHelper->SetBeamformingHelper(idealBeamformingHelper);
    nrHelper->SetEpcHelper(nrEpcHelper);

    Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false));

    // Measurement and handover configuration (simplified for NR)
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue(0));

    // Handover: O Início


    // configuração de parâmetros

    // Define o algoritmo baseado em RSRP (Potência)
        nrHelper->SetHandoverAlgorithmType("ns3::NrA3RsrpHandoverAlgorithm");

    // Define a "Histerese" (Margem de segurança) de 3.0 dB
        nrHelper->SetHandoverAlgorithmAttribute("Hysteresis", DoubleValue(0.5));

    // Define o tempo para disparar (evita trocas por ruído momentâneo)
        nrHelper->SetHandoverAlgorithmAttribute("TimeToTrigger", TimeValue(MilliSeconds(10)));

    // Force RRC to REAL mode (not IDEAL)
    //Config::SetDefault("ns3::LteHelper::UseIdealRrc", BooleanValue(false)); 

    //nrHelper->SetAttribute("UseIdealRrc", BooleanValue(false));

    // Configure measurement reporting
    Config::SetDefault("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue(0));


    /*
     * Spectrum configuration. We create a single operational band and configure the scenario.
     */

    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1; // in tha throughput  is example we have a single band, and that band is
                                    // composed of a single component carrier

    /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf creates
     * a single BWP per CC and a single BWP in CC.
     *
     * Hence, the configured spectrum is:
     *
     * |---------------Band---------------|
     * |---------------CC-----------------|
     * |---------------BWP----------------|
     */
    CcBwpCreator::SimpleOperationBandConf bandConf(frequency, bandwidth, numCcPerBand);
    OperationBandInfo band = ccBwpCreator.CreateOperationBandContiguousCc(bandConf);
    // Create the channel helper
    Ptr<NrChannelHelper> channelHelper = CreateObject<NrChannelHelper>();
    // Set and configure the channel to the current band
    channelHelper->ConfigureFactories(
        scenario,
        "Default",
        "ThreeGpp"); // Configure the spectrum channel with the scenario
    channelHelper->AssignChannelsToBands({band});
    allBwps = CcBwpCreator::GetAllBwps({band});

    // Configure ideal beamforming method

    //std::string beamformingMethod = "Isotropic";

   //idealBeamformingHelper->SetAttribute("BeamformingMethod", beamformingMethod)
 

   idealBeamformingHelper->SetAttribute("BeamformingMethod", TypeIdValue(DirectPathBeamforming::GetTypeId()));

    // Configure scheduler
    nrHelper->SetSchedulerTypeId(NrMacSchedulerTdmaRR::GetTypeId());

    // Antennas for the UEs
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(4));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    // Antennas for the gNbs
    nrHelper->SetGnbAntennaAttribute("NumRows", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("NumColumns", UintegerValue(8));
    nrHelper->SetGnbAntennaAttribute("AntennaElement",
                                     PointerValue(CreateObject<IsotropicAntennaModel>()));

    // install nr net devices




    NetDeviceContainer gnbNetDev = nrHelper->InstallGnbDevice(gnbNodes, allBwps);
    NetDeviceContainer ueNetDev = nrHelper->InstallUeDevice(ueNodes, allBwps);

    checaNode(gnbNetDev, ueNetDev);


    int64_t randomStream = 1;
    randomStream += nrHelper->AssignStreams(gnbNetDev, randomStream);
    randomStream += nrHelper->AssignStreams(ueNetDev, randomStream);

    NrHelper::GetGnbPhy(gnbNetDev.Get(0), 0)->SetTxPower(txPower);
    NrHelper::GetGnbPhy(gnbNetDev.Get(1), 0)->SetTxPower(txPower);

    // create the internet and install the IP stack on the UEs
    // get SGW/PGW and create a single RemoteHost
    auto [remoteHost, remoteHostIpv4Address] =
        nrEpcHelper->SetupRemoteHost("100Gb/s", 2500, Seconds(0.010));

    InternetStackHelper internet;
    internet.Install(ueNodes);

    Ipv4InterfaceContainer ueIpIface;
    ueIpIface = nrEpcHelper->AssignUeIpv4Address(NetDeviceContainer(ueNetDev));

    // assign IP address to UEs, and install UDP downlink applications
    uint16_t dlPort = 1234;
    ApplicationContainer clientApps;
    ApplicationContainer serverApps;
    for (uint32_t u = 0; u < ueNodes.GetN(); ++u)
    {
        Ptr<Node> ueNode = ueNodes.Get(u);
        UdpServerHelper dlPacketSinkHelper(dlPort);
        serverApps.Add(dlPacketSinkHelper.Install(ueNodes.Get(u)));

        UdpClientHelper dlClient(ueIpIface.GetAddress(u), dlPort);
        dlClient.SetAttribute("Interval", TimeValue(MicroSeconds(100)));
        dlClient.SetAttribute ("MaxPackets", UintegerValue(0xFFFFFFFF));
        //dlClient.SetAttribute("MaxPackets", UintegerValue(3612));
        dlClient.SetAttribute("PacketSize", UintegerValue(1500));
        clientApps.Add(dlClient.Install(remoteHost));
    }


    // 1. interface Xn ou X2
    /*
     * é um link entre as gNBs para passar as informações dos clientes quando tiver 
     * trocando
     * 
     * NOTA: a função espera 2 gnodes como argumento
     */

    nrEpcHelper->AddX2Interface(gnbNodes.Get(0), gnbNodes.Get(1));


    // attach UEs to the closest gNB
    nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);



    // start server and client apps

    serverApps.Start(Seconds(1.4));
    clientApps.Start(Seconds(1.4));
    serverApps.Stop(Seconds(simTime));
    clientApps.Stop(Seconds(simTime - 0.2));

    // enable the traces provided by the nr module
    nrHelper->EnableTraces();

        // Anexe o UE inicialmente à torre mais próxima (gNB 0)
        //nrHelper->AttachToClosestGnb(ueNetDev, gnbNetDev);
                //nrHelper->AttachToGnb(ueNetDev.Get(0), gnbNetDev.Get(0));

    
    
    //configuração do flowmonitor

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    monitor->CheckForLostPackets();

    //funções em agendamento

    //Simulator::Schedule(Seconds(0.1), &ondeTa, ueNodes);


    Simulator::Schedule(Seconds(0.6), &checaNode, gnbNetDev, ueNetDev);
    Simulator::Schedule(Seconds(1.6), &checaNode, gnbNetDev, ueNetDev);
    Simulator::Schedule(Seconds(2.6), &checaNode, gnbNetDev, ueNetDev);
    Simulator::Schedule(Seconds(3.6), &checaNode, gnbNetDev, ueNetDev);
    Simulator::Schedule(Seconds(4.6), &checaNode, gnbNetDev, ueNetDev);





    Simulator::Stop(Seconds(simTime));
    Simulator::Run();


    monitor->SerializeToXmlFile(tr_name + ".xml", true, true);

    Ptr<UdpServer> serverApp = serverApps.Get(0)->GetObject<UdpServer>();
    uint64_t receivedPackets = serverApp->GetReceived();

    Simulator::Destroy();


    

    if (receivedPackets >= 10)
    {
        organizar("scratch/results/ex005/");
        return EXIT_SUCCESS;
    }
    else
    {
        return EXIT_FAILURE;
    }
}

void ondeTa(NodeContainer ueNodes){

    Ptr<Node> node = ueNodes.Get(0);

    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();

    Vector pos = mobility->GetPosition();

    std::cout << "Tempo " << Simulator::Now().GetSeconds() << " s "
              << "Node: " << node->GetId() << " "
              << "Posição: < " << pos.x << ", " << pos.y << ", " << pos.z << "> " 
              << "\n ==============================" << std::endl; 

    Simulator::Schedule(Seconds(0.5), &ondeTa, ueNodes);
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
    try {656175
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

void checaNode(const NetDeviceContainer& gnbNetDev, const NetDeviceContainer& ueNetDev){
    // Loop gNB

    std::cout << "===========================\n" << "Tempo " << Simulator::Now().GetSeconds() << std::endl;

    for (uint32_t i = 0; i < gnbNetDev.GetN(); ++i)
    {
        // Nota: Quando usamos const&, o Get(i) retorna um Ptr constante, mas o DynamicCast lida bem com isso
        uint16_t cellId = DynamicCast<NrGnbNetDevice>(gnbNetDev.Get(i))->GetCellId();
        std::cout << "gNB " << i << " has CellId: " << cellId << std::endl;
    }

    // Loop UE
    for (uint32_t i = 0; i < ueNetDev.GetN(); ++i)
    {
        uint16_t cellId = DynamicCast<NrUeNetDevice>(ueNetDev.Get(i))->GetCellId();
        std::cout << "UE " << i << " initially attached to CellId: " << cellId << std::endl;
    }
}
