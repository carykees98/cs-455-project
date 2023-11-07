/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/dvhop-module.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"
#include "ns3/netanim-module.h"

#include "ns3/random-variable-stream.h"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>

using namespace ns3;

/**
 * \brief Test script.
 *
 * This script creates 1-dimensional grid topology and then ping last node from the first one:
 *
 * [10.0.0.1] <-- step --> [10.0.0.2] <-- step --> [10.0.0.3] <-- step --> [10.0.0.4]
 *
 *
 */
class DVHopExample
{
public:
  DVHopExample ();
  /// Configure script parameters, \return true on successful configuration
  bool Configure (int argc, char **argv);
  /// Run simulation
  void Run ();
  /// Report results
  void Report (std::ostream & os);

private:
  ///\name parameters
  //\{
  uint32_t beaconCount; // number of beacons
  double max_rand_val; // Maximum random value
  double min_rand_val; // Minimum random value
  uint32_t seed; // randomization seed
  std::string savePath; // path to save layout to
  std::string loadPath; // path to load layout from


  /// Number of nodes
  uint32_t size;
  /// Distance between nodes, meters
  double step;
  /// Simulation time, seconds
  double totalTime;
  /// Write per-device PCAP traces if true
  bool pcap;
  /// Print routes if true
  bool printRoutes;

  //\}

  ///\name network
  //\{
  NodeContainer nodes;
  NetDeviceContainer devices;
  Ipv4InterfaceContainer interfaces;
  //\}

private:
  void CreateNodes ();
  void CreateDevices ();
  void InstallInternetStack ();
  void InstallApplications ();
  void CreateBeacons();

  // Custom
  void SetupLayout();
  void LoadLayout();
};

int main (int argc, char **argv)
{
  DVHopExample test;
  if (!test.Configure (argc, argv))
    NS_FATAL_ERROR ("Configuration failed. Aborted.");

  test.Run ();
  test.Report (std::cout);
  return 0;
}

//-----------------------------------------------------------------------------
// Initialize default DVHopExample variable values
DVHopExample::DVHopExample () :
  beaconCount(3),
  max_rand_val(10000.0),
  min_rand_val(0.0),
  seed(12345),
  savePath(""),
  loadPath(""),
  size (10),
  step (100),
  totalTime (10),
  pcap (true),
  printRoutes (true)
{
}

// Parse and load command line args into variables
bool
DVHopExample::Configure (int argc, char **argv)
{
  // Enable DVHop logs by default. Comment this if too noisy
  LogComponentEnable("DVHopRoutingProtocol", LOG_LEVEL_ALL);

  CommandLine cmd;

  cmd.AddValue ("pcap", "Write PCAP traces.", pcap);
  cmd.AddValue ("printRoutes", "Print routing table dumps.", printRoutes);
  cmd.AddValue ("size", "Number of nodes.", size);
  cmd.AddValue ("time", "Simulation time, s.", totalTime);
  cmd.AddValue ("step", "Grid step, m", step);
  cmd.AddValue ("seed", "Randomization seed", seed);
  cmd.AddValue ("beaconCount", "Number of beacons", beaconCount);
//  cmd.AddValue ("saveTo", "Save the generated layout to a file", savePath);
  cmd.AddValue ("loadFrom", "Load a layout from a file", loadPath);

  cmd.Parse (argc, argv);

  SeedManager::SetSeed (seed);

  return true;
}

// Simple function that decides how to initialize the layout of the nodes
void DVHopExample::SetupLayout() {
  // Layout file provided
  if(loadPath != "") {
    LoadLayout();
    return;
  }

  // No layout file provided
  CreateNodes();
  CreateDevices();
  InstallInternetStack();
  CreateBeacons();
}

// Run the simulation
void
DVHopExample::Run ()
{
//  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", UintegerValue (1)); // enable rts cts all the time.

  SetupLayout();

  std::cout << "Starting simulation for " << totalTime << " s ...\n";

  Simulator::Stop (Seconds (totalTime));

  AnimationInterface anim("animation.xml");

  Simulator::Run ();
  Simulator::Destroy ();
}

void
DVHopExample::Report (std::ostream &)
{
}


// Load the layout of nodes from file provided in command arguments ('loadPath')
void DVHopExample::LoadLayout() {
  struct nodeLoadStruct {
    bool isBeacon;
    double xPos;
    double yPos;
  };

  std::vector<nodeLoadStruct> nodesVector;

  std::ifstream layoutStream(loadPath);
  std::cout << "Loading from file " << loadPath << std::endl;

  // Load file into vector of data
  std::string line;
  while( layoutStream.peek() != EOF ) {
    std::getline( layoutStream, line );
    std::istringstream stringStream(line);

    nodeLoadStruct node;
    stringStream >> node.isBeacon;
    stringStream >> node.xPos;
    stringStream >> node.yPos;

    nodesVector.push_back(node);
  }

  // Create and name nodes
  nodes.Create( nodesVector.size() );
  auto i = nodesVector.size(); // set to vector size to get proper var type, then set back to 0 in loop
  for( i=0; i<nodesVector.size(); ++i ) {
    // Name node
    std::ostringstream os;
    os << "node-" << i;
    std::cout << "Loading node: " << os.str() << std::endl;
    Names::Add( os.str(), nodes.Get(i) );
  }

  // Do internet stuff
  CreateDevices();
  InstallInternetStack();

  // install mobility helper
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  // Move node positions and set beacon status
  for( i=0; i<nodesVector.size(); ++i) {
    Ptr<Ipv4RoutingProtocol> proto = nodes.Get( i )->GetObject<Ipv4>()->GetRoutingProtocol();
    Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol>(proto);

    // Set beacon status
    dvhop->SetIsBeacon( nodesVector[i].isBeacon );

    // Move node into position
    Ptr<ConstantPositionMobilityModel> p = nodes.Get(i)->GetObject<ConstantPositionMobilityModel>();
    std::cout << nodesVector[i].xPos << " " << nodesVector[i].yPos << std::endl;
    p->SetPosition( Vector( nodesVector[i].xPos, nodesVector[i].yPos, 0 ) );
  }

}

// Create nodes in a line based on 'step' and 'size' (optionally provided in command line)
void
DVHopExample::CreateNodes ()
{
  std::cout << "Creating " << (unsigned)size << " nodes " << step << " m apart.\n";
  nodes.Create (size);
  // Name nodes
  for (uint32_t i = 0; i < size; ++i)
    {
      std::ostringstream os;
      os << "node-" << i;
      std::cout << "Creating node: "<< os.str ()<< std::endl ;
      Names::Add (os.str (), nodes.Get (i));
    }
  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (step),
                                 "DeltaY", DoubleValue (0),
                                 "GridWidth", UintegerValue (size),
                                 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);
}

// Turn 'beaconCount' number of nodes into beacons (optionally provided in command line)
void
DVHopExample::CreateBeacons ()
{
  uint32_t step_size = size / beaconCount;
  uint32_t i;
  for(i=0; i<beaconCount; ++i) {
    uint32_t node_id = i * step_size;

    Ptr<Ipv4RoutingProtocol> proto = nodes.Get( node_id )->GetObject<Ipv4>()->GetRoutingProtocol();
    Ptr<dvhop::RoutingProtocol> dvhop = DynamicCast<dvhop::RoutingProtocol>(proto);
    dvhop->SetIsBeacon(true);


   Ptr<UniformRandomVariable> rand = CreateObject<UniformRandomVariable>();
   dvhop->SetPosition( rand->GetValue( min_rand_val, max_rand_val ), rand->GetValue( min_rand_val, max_rand_val ) );
  }
}

// Create wireless devices to install into all nodes
void
DVHopExample::CreateDevices ()
{
  WifiMacHelper wifiMac = WifiMacHelper();
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  WifiHelper wifi = WifiHelper();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
  devices = wifi.Install (wifiPhy, wifiMac, nodes);

  if (pcap)
    {
      wifiPhy.EnablePcapAll (std::string ("aodv"));
    }
}

// Install wireless devices into all nodes
void
DVHopExample::InstallInternetStack ()
{
  DVHopHelper dvhop;
  // you can configure DVhop attributes here using aodv.Set(name, value)
  InternetStackHelper stack;
  stack.SetRoutingHelper (dvhop); // has effect on the next Install ()
  stack.Install (nodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.0.0.0");
  interfaces = address.Assign (devices);

  Ptr<OutputStreamWrapper> distStream = Create<OutputStreamWrapper>("dvhop.distances", std::ios::out);
  dvhop.PrintDistanceTableAllAt(Seconds(9), distStream);

  if (printRoutes)
    {
      Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("dvhop.routes", std::ios::out);
      dvhop.PrintRoutingTableAllAt (Seconds (8), routingStream);
    }
}
