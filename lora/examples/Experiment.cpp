#include "Experiment.h"

#include "Ansi.h"
#include "LoraMeshNetDevice.h"
#include "RandomNumbers.h"
#include "TermColors.h"

#include "ns3/time-printer.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-phy-gen.h"
#include "ns3/lora-prop-model-ideal.h"
#include "ns3/lora-prop-model-thorp.h"
#include "ns3/lora-transducer-hd.h"
#include "ns3/lora-tx-mode.h"
#include "ns3/mac-lora-gw.h"
#include "ns3/object-factory.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/show-progress.h"
#include "ns3/simulator.h"

#include <algorithm>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace Experiment {

void displayPacketSources(std::ostream &s, const SentPacketMap &sentPackets,
                          const std::string &title) {
  s << title << " size: " << sentPackets.size() << std::endl;
  for (auto &p : sentPackets) {

    s << p.first << ": [";
    std::map<LoraMeshNetDevice::IDAddress, size_t> counts;
    for (auto &id : p.second) {
      counts[id]++;
    }

    bool first = true;
    for (auto &count : counts) {
      if (!first) {
        s << ", ";
      }
      first = false;
      s << count.first << "x" << count.second;
    }
    s << "]" << std::endl;
  }
}

void displayQueue(const LoraMeshNetDevice::Queue &q, const std::string &title) {
  std::cout << title << " Size: " << q.size() << std::endl;
  if (!q.empty()) {
    std::cout << "[ ";
    for (auto &pkt : q) {
      bool first = true;
      if (!first) {
        std::cout << ", ";
      }
      first = false;
      std::cout << LoraMesh::IDAddressFromPacketData(pkt) << " "
                << LoraMesh::commandFromPacketData(pkt) << std::endl;
    }
    std::cout << "]" << std::endl;
  }
}

void packetMapReport(std::ostream &s, Nodes &nodes) {
  SentPacketMap sentPackets;
  SentPacketMap failedToSendPackets;
  for (const auto &node : nodes) {
    auto nodeSentPackets = node->getSentPackets();
    sentPackets.insert(nodeSentPackets.begin(), nodeSentPackets.end());
    auto failedPackets = node->getFailedToSendPackets();
    failedToSendPackets.insert(failedPackets.begin(), failedPackets.end());
    s << node->getNodeIndex() << " TQ Size: " << node->getTQ().size()
      << " BQ Size: " << node->getBQ().size() << std::endl;
  }
  displayPacketSources(s, sentPackets, "Packet Sources");
  displayPacketSources(s, failedToSendPackets, "Failed Packet Sources");
}

void packetsSeenReport(std::ostream &s, Nodes &nodes) {
  std::cout << "Packets seen" << std::endl;
  for (auto &node : nodes) {
    auto &seen = node->getPacketsSeen();
    if (!seen.empty()) {
      std::cout << node->getNodeIndex() << ":[";
      bool first = true;
      for (auto &id : seen) {
        if (!first) {
          std::cout << ", ";
        }
        first = false;
        std::cout << id;
      }
      std::cout << "]" << std::endl;
    }
  }
}

void packetTraceReport(std::ostream &s,
                       const LoraMesh::IDAddress &idaddressTofollow,
                       const Nodes &nodes) {
  s << idaddressTofollow << " visited: ";
  bool first = true;
  for (auto &node : nodes) {
    auto packetsSeen = node->getPacketsSeen();
    if (packetsSeen.find(idaddressTofollow) != packetsSeen.end()) {
      auto node_index = node->getNodeIndex();
      if (!first) {
        s << ", ";
      }
      first = false;
      s << node_index;
    }
  }
  s << std::endl;
}

std::string timestamp() {
  auto now = std::chrono::system_clock::now();
  auto nowTT = std::chrono::system_clock::to_time_t(now);
  std::stringstream s;
  s << std::put_time(std::localtime(&nowTT), "%F_%H_%M_%S");
  // s << std::put_time(std::localtime(&nowTT), "%F_%H");
  return s.str();
}

fs::path root("../../../../../");
fs::path makeExperimentsDir() {
  // root/timestamp
  // std::cout << "root: " << root.string() << "experiments/" << timestamp()
  //          << std::endl;
  // auto basepath0 = root / "experiments" / timestamp();
  auto exppath = getExpSubPath();
  std::cout << "exppath: " << exppath.string() << std::endl;

  fs::create_directories(exppath);
  exppath = fs::canonical(exppath);
  return exppath;
}

std::ofstream makeFStream(const fs::path &dir, const fs::path &base) {
  auto filename = dir / base;
  filename += ".csv";
  std::ofstream f(filename, std::ios::out | std::ios::trunc);
  return f;
}

std::ofstream makeFStream(const fs::path &dir, const fs::path &base,
                          int runNo) {
  std::stringstream filename;
  filename << base.string() << "_" << runNo << ".csv";
  auto filepath = dir / fs::path(filename.str());
  std::cout << std::endl << "filename: " << filepath << std::endl;
  std::ofstream f(filepath, std::ios::out | std::ios::trunc);

  return f;
}

std::string addrInt(const AddressInt &address) {
  std::stringstream s;
  s << "{" << LoraMeshNetDevice::lookUpNodeIndex(address) << "}";
  return s.str();
}

std::string nodeInt(DevicePtr node) {
  std::stringstream s;
  s << "{" << node->getNodeIndex() << "}";
  return s.str();
}

void packetRoundTrip(std::ostream &s, const Nodes &nodes) {
  s << "PacketTrace" << std::endl;

  auto tracker = LoraMeshNetDevice::getPacketTracker();
  for (auto &tracked : tracker) {
    auto pt = tracked.second;
    if (LoraMesh::isBad(pt.mPacket)) {
      continue;
    }
    s << pt.mPacket << ","
      << LoraMeshNetDevice::lookUpNodeIndex(pt.mDestination) << ","
      << LoraMesh::toString(pt.mCommand) << "," << pt.mSent << ","
      << pt.mReceived << "," << pt.mDestReceiptAtSender << "," << pt.mSentAt
      << "," << pt.mReceivedAt << "," << pt.mReceiptReceivedAt << ","
      << pt.mReceiptCount << "," << pt.mDestCount << "," << pt.mForwardCount
      << std::endl;
  }
}

void packetStatReport(std::ostream &s, const Nodes &nodes) {
  auto badTracker = LoraMeshNetDevice::getBadPacketTracker();
  auto tracker = LoraMeshNetDevice::getPacketTracker();
  // Number of packets sent, Number of packets at Destination, Number of packets
  // lost, by type
  struct PackStat {
    PackStat() : mNumSent(0), mNumComplete() {}
    size_t mNumSent;
    size_t mNumComplete;
  };
  std::map<LoraMesh::Command, PackStat> stats;
  for (auto &tracked : tracker) {
    auto pt = tracked.second;
    if (pt.mSent) {
      stats[pt.mCommand].mNumSent++;
      if (pt.mReceived) {
        stats[pt.mCommand].mNumComplete++;
      }
    }
  }
  s << "Delivery" << std::endl;
  for (auto stat : stats) {
    s << LoraMesh::toString(stat.first) << "," << stat.second.mNumComplete
      << "," << stat.second.mNumSent << std::endl;
  }

  int nodeNo = 0;
  std::map<LoraMeshNetDevice::AddressInt, int> address_nodeNo_map;
  for (auto &node : nodes) {
    address_nodeNo_map[node->getSavedAddressInt()] = nodeNo++;
  }

  s << "Route" << std::endl;
  int packetNo = 0;
  for (auto &tracked : tracker) {
    auto pt = tracked.second;
    s << packetNo++ << "," << LoraMesh::toString(pt.mCommand);
    for (auto addr : pt.mAddressTrack) {
      s << ", " << address_nodeNo_map[addr];
    }
    if (!pt.mReceived) {
      s << ", -1";
    }
    s << std::endl;
  }

  nodeNo = 0;
  s << "SendFail" << std::endl;
  for (auto &node : nodes) {
    s << nodeNo++ << ", " << node->getFailedToSendPackets().size() << std::endl;
  }

  nodeNo = 0;
  s << "ReceiveFail" << std::endl;
  for (auto &node : nodes) {
    s << nodeNo++ << ", " << node->getFailedToReceivePackets().size()
      << std::endl;
  }

  nodeNo = 0;
  s << "TQ" << std::endl;
  for (auto &node : nodes) {
    s << nodeNo++ << ", " << node->getTQ().size() << ", " << node->getTQMax()
      << std::endl;
  }

  nodeNo = 0;
  s << "BQ" << std::endl;
  for (auto &node : nodes) {
    s << nodeNo++ << ", " << node->getBQ().size() << ", " << node->getBQMax()
      << std::endl;
  }

  packetNo = 0;
  s << "Corrupt" << std::endl;
  for (auto b : badTracker) {
    s << packetNo++ << ", " << b.second << std::endl;
  }

  nodeNo = 0;
  s << "Seen" << std::endl;
  for (auto &node : nodes) {
    auto numPacketsSeen = node->getPacketsSeen().size();
    s << nodeNo++ << ", " << numPacketsSeen << std::endl;
  }

  auto sendTracker = LoraMeshNetDevice::sendTimeTracker();
  s << "SendTime" << std::endl;
  for (auto &time : sendTracker) {
    s << std::setprecision(10) << time.first << ", "
      << LoraMesh::toString(time.second) << std::endl;
  }

  auto receiveTracker = LoraMeshNetDevice::receiveTimeTracker();
  s << "ReceiveTime" << std::endl;
  for (auto &time : receiveTracker) {
    s << std::setprecision(10) << time.first << ", "
      << LoraMesh::toString(time.second) << std::endl;
  }
  packetRoundTrip(s, nodes);
}

void packetTrackerReport(std::ostream &s, const Nodes &nodes) {
  auto tracker = LoraMeshNetDevice::getPacketTracker();
  for (auto &tracked : tracker) {
    auto pt = tracked.second;
    if (LoraMesh::isBad(pt.mPacket)) {
      s << boldred << "Bad mPacket " << reset << std::endl;
      continue;
    }
    s << pt.mPacket << " " << LoraMesh::toString(pt.mCommand) << "->"
      << addrInt(pt.mDestination);
    if (pt.mSent) {
      s << " forwarded " << pt.mForwardCount << " times at " << pt.mSentAt;
    } else {
      s << "not sent";
    }
    if (pt.mReceived) {
      s << " received " << pt.mDestCount << " times at " << pt.mReceivedAt;
    } else {
      s << " not received ";
    }
    if (pt.mDestReceiptAtSender) {
      s << " receipt received " << pt.mReceiptCount << " times at "
        << pt.mReceiptReceivedAt;
    } else {
      // Only report no destination recepit for Measurement commands.
      if (pt.mCommand == LoraMesh::Command::Measurement) {
        s << " no receipt ";
      }
    }
    s << std::endl;
    if (!pt.mSent || !pt.mReceived || !pt.mDestReceiptAtSender) {
      packetTraceReport(s, pt.mPacket, nodes);
    }
  }
  packetStatReport(s, nodes);
}

void nodeLocationReport(std::ostream &s, Nodes &nodes) {
  for (auto &node : nodes) {
    s << nodeInt(node) << " : At " << node->getLocation() << std::endl
      << "Distance to:" << std::endl;
    bool first = true;
    for (auto &otherNode : nodes) {
      auto dist =
          ns3::CalculateDistance(node->getLocation(), otherNode->getLocation());
      if (!first) {
        s << ", ";
      }
      first = false;
      s << nodeInt(otherNode) << " " << dist;
    }
    s << std::endl;
  }
}

void exportNodeInfo(std::ostream &s, Nodes &nodes) {
  int nodeNum = 0;
  for (auto &node : nodes) {
    auto loc = node->getLocation();
    s << nodeNum++ << "," << loc.x << "," << loc.y << "," << loc.z << std::endl;
  }
}

// For reference if I need this later.
#if 0
  std::cout << "Visibility Map" << std::endl;
  for (auto &visPair : mVisibilityMap) {
    std::cout << visPair.first << ": [";
    bool first = true;
    for (auto &vis : visPair.second) {
      if (!first) {
        std::cout << ", ";
      }
      first = false;
      std::cout << vis;
    }
    std::cout << "]" << std::endl;
  }
#endif

void visibilityMapCounts(std::ostream &s, Nodes &nodes) {
  s << "Visibility Map Counts:" << std::endl;
  for (auto &node : nodes) {
    auto visibleNodes = node->getVisibleAddresses();
    s << node->getNodeIndex() << ": sees " << visibleNodes.size() << " nodes"
      << std::endl;
  }
}

void queueReport(std::ostream &s, Nodes &nodes) {
  for (const auto &node : nodes) {
    std::cout << node->getNodeIndex() << ": Max " << node->getTQMax() << " ";
    displayQueue(node->getTQ(), "TQ");
    std::cout << node->getNodeIndex() << ": Max " << node->getBQMax() << " ";
    displayQueue(node->getBQ(), "BQ");
  }
}

void simulate(double duration) {
  // Simulator::Stop (Seconds (120.0));
  using namespace ns3;
  // Time::SetResolution(ns3::Time::MS);

  // Run forever...
  // Simulator::Stop(Seconds(duration * 4));
  // ShowProgress progress(Seconds(1), std::cerr);
  try {
    Simulator::Run();
    Simulator::Destroy();
  } catch (std::bad_alloc &ba) {
    std::cout << "bad_alloc caught: " << ba.what() << std::endl;
    std::cerr << "bad_alloc caught: " << ba.what() << std::endl;
  }
}

void stopSimulationNow() {
  using namespace ns3;
  Simulator::Stop();
  std::cout << "=========Stop " << Simulator::Now().GetSeconds()
            << " s =========" << std::endl;
  // Simulator::Destroy();
}

void Base::initRun() {

  mOutputDir = Experiment::makeExperimentsDir();
  std::string vis = "visualize.py";
  auto vispy = mOutputDir / vis;
  // if (!fs::exists(vispy)) {
  std::cout << "copy examples/visualize.py, vis_support.py to " << mOutputDir
            << std::endl;
  Experiment::fs::copy("examples/visualize.py", mOutputDir,
                       fs::copy_options::update_existing);
  Experiment::fs::copy("examples/vis_support.py", mOutputDir,
                       fs::copy_options::update_existing);
  //}

// ns3 Time is not designed to handle re-initialization. Do once.
// The guts of ns3::Time contain some scary code...
#if 0
  static bool resolutionInited = false;
  if (!resolutionInited) {
    ns3::Time::SetResolution(ns3::Time::MS);
    resolutionInited = true;
  }
#endif

  LoraMeshNetDevice::clearPacketTrackers();

  // Experiment is reused for multiple runs so we need
  // to reset the stop count everytime we run.
  mStopCount = 0;
}

std::ostream &Base::reportHeader(std::ostream &s, int runNo) {
  s << Ansi::clear();
  s << yellow << "----" << getTitle() << " run " << runNo << " ----"
    << std::endl;
  reportOptions(s);
  s << reset;
  return s;
}

void Zero::run(int runNo) {
  Experiment::setVerbosity(mVerbose);
  Experiment::setBasePath(mExpSubPath);
  Experiment::setCorruptionProb(mCorruptionProb);
  Experiment::setSeed(mSeed);

  initRun();
  reportHeader(std::cout, runNo);
  initPhyFactory();

  auto prop = makePropogationModel(!mUseThorpProp);
  BroadCastMode mode = 0;

  // Create the nodes and initialize their positions.
  // Range 0 - width in x
  //       0 - depth in y
  //       0 - height in z
  const double width = mMeshSize;
  const double depth = mMeshSize;
  const double height = 25;

  mNodes.clear();
  mNodes.resize(mNumNodes);

  // Create mesh.
  if (mMeshType == 0) {
    createMesh(mNodes, prop, width, depth, height, mode, mBQMult);
  } else if (mMeshType == 1) {
    createLinearMesh(mNodes, prop, width, depth, height, mode, mBQMult);
  }
  Experiment::connectReceiveCallback(mNodes);

  auto visibilityMap = Experiment::createVisibilityMap(mNodes, false);
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << "visibilityMap Count: " << visibilityMap.size() << std::endl;
  }
  Experiment::assignVisibilityMap(mNodes, visibilityMap);

  // Pick a source and destination at random.
  auto nodePair = Experiment::pickRandomSourceAndDestination(mNumNodes);
  auto source = nodePair.first;
  auto dest = nodePair.second;

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    // Schedule packet to send at next time slot.
    std::cout << "Schedule broadcast between: " << source << " and " << dest
              << std::endl;
  }
  double transmitStartTime = 0; // RanTxTime(0, kTransmitPeriod);

  Experiment::scheduleBroadcastBetween(transmitStartTime, mNodes[source],
                                       mNodes[dest]);

  Experiment::startForwarding(mNodes, *this, transmitStartTime, mPeriod,
                              mReceiptDelay, mPacketDelay);

  // Simulator::Stop (Seconds (120.0));
  simulate(mSimulationDuration);
}

std::ostream &Zero::report(std::ostream &s, int runNo) {
  // s << "mNodes.size()" << mNodes.size() << std::endl;
  packetMapReport(s, mNodes);
  packetsSeenReport(s, mNodes);
  visibilityMapCounts(s, mNodes);
  queueReport(s, mNodes);
  packetTrackerReport(s, mNodes);
  nodeLocationReport(s, mNodes);
  return s;
}

void Zero::exportResults(int runNo) {
  auto f = makeFStream(getOutputDir(), getName() + "NodeLocs", runNo);
  exportNodeInfo(f, mNodes);
  auto f2 = makeFStream(getOutputDir(), getName() + "PacketStats", runNo);
  packetStatReport(f2, mNodes);
}

void One::run(int runNo) {
  initRun();
  reportHeader(std::cout, runNo);
  initPhyFactory();

  auto prop = makePropogationModel(!mUseThorpProp);
  BroadCastMode mode = 0;

  // Create the nodes and initialize their positions.
  // Range 0 - width in x
  //       0 - depth in y
  //       0 - height in z
  const double width = mMeshSize;
  const double depth = mMeshSize;
  const double height = 25;

  mNodes.clear();
  mNodes.resize(mNumNodes);

  // Create mesh.
  createMesh(mNodes, prop, width, depth, height, mode, mBQMult);

  connectReceiveCallback(mNodes);

  auto visibilityMap = Experiment::createVisibilityMap(mNodes, false);
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << "visibilityMap Count: " << visibilityMap.size() << std::endl;
  }
  Experiment::assignVisibilityMap(mNodes, visibilityMap);

  pickAndSchedule(mRecurTime, mSimulationDuration - mSimulationDuration / 10,
                  mNodes);

  startForwarding(mNodes, *this, 0, mPeriod, mReceiptDelay, mPacketDelay);

  simulate(mSimulationDuration);
}

std::ostream &One::report(std::ostream &s, int runNo) {
  packetMapReport(s, mNodes);
  packetsSeenReport(s, mNodes);
  visibilityMapCounts(s, mNodes);
  queueReport(s, mNodes);
  packetTrackerReport(s, mNodes);
  nodeLocationReport(s, mNodes);
  return s;
}

void One::exportResults(int runNo) {
  auto fSum = makeFStream(getOutputDir(), getName() + "Summary");
  fSum << getTitle() << "," << mNumRuns << "," << mNumNodes << ","
       << mSimulationDuration << "," << mMeshSize << "," << mMeshType << ","
       << mPeriod << "," << mReceiptDelay << ", " << mPacketDelay << ", "
       << mBQMult << "," << mUseThorpProp << "," << mRecurTime << ", "
       << mCorruptionProb << std::endl;
  fSum.flush();
  fSum.close();

  auto fInfo = makeFStream(getOutputDir(), getName() + "NodeLocs", runNo);
  exportNodeInfo(fInfo, mNodes);
  fInfo.flush();
  fInfo.close();
  auto fStats = makeFStream(getOutputDir(), getName() + "PacketStats", runNo);
  packetStatReport(fStats, mNodes);
  fStats.flush();
  fStats.close();
}

void Two::run(int runNo) {
  Experiment::setVerbosity(mVerbose);
  Experiment::setBasePath(mExpSubPath);
  Experiment::setCorruptionProb(mCorruptionProb);
  Experiment::setSeed(mSeed);

  initRun();
  reportHeader(std::cout, runNo);
  initPhyFactory();

  auto prop = makePropogationModel(!mUseThorpProp);
  BroadCastMode mode = 0;

  // Create the nodes and initialize their positions.
  // Range 0 - width in x
  //       0 - depth in y
  //       0 - height in z
  const uint32_t width = mMeshSize;
  const uint32_t depth = mMeshSize;
  const uint32_t height = 25;

  mNodes.clear();
  mNodes.resize(mNumNodes);

  // Create mesh.
  if (mMeshType == 0) {
    createMesh(mNodes, prop, width, depth, height, mode, mBQMult);
  } else if (mMeshType == 1) {
    createLinearMesh(mNodes, prop, width, depth, height, mode, mBQMult);
  }

  connectReceiveCallback(mNodes);

  auto visibilityMap = Experiment::createVisibilityMap(mNodes, false);
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << "visibilityMap Count: " << visibilityMap.size() << std::endl;
  }
  Experiment::assignVisibilityMap(mNodes, visibilityMap);

  scheduleFromAllToOne(mRecurTime, mSimulationDuration, mNodes);

  startForwarding(mNodes, *this, 0, mPeriod, mReceiptDelay, mPacketDelay);

  simulate(mSimulationDuration);
}

std::ostream &Two::report(std::ostream &s, int runNo) {
  One::report(s, runNo);
  return s;
}

void Two::exportResults(int runNo) { One::exportResults(runNo); }

// This is starting to look a lot like a class
ns3::ObjectFactory mPhyFac("ns3::LoraPhyDual");
int mVerbosity;
double mCorruptionProb;
fs::path mExpSubPath;
SecondsType mDuration;

void setVerbosity(int verbosity) { mVerbosity = verbosity; }
int getVerbosity() { return mVerbosity; }
void setSeed(unsigned int seed) {
  static bool seed_set = false;
  if (!seed_set) {
    Random::seed(seed);
    ns3::RngSeedManager::SetSeed(seed);
    seed_set = true;
  }
}

void setCorruptionProb(double p) { mCorruptionProb = p; }

double getCorruptionProb() { return mCorruptionProb; }
void duration(SecondsType d) { mDuration = d; }
SecondsType duration() { return mDuration; }
SecondsType durationLimit() { return mDuration * 2; }

fs::path getExpSubPath() { return mExpSubPath; }
void setBasePath(const std::string &path) { mExpSubPath = path; }

DevicePtr createNode(ns3::Vector pos, LoraChannelPtr chan, double bqMult) {

  auto phy = mPhyFac.Create<ns3::LoraPhy>();
  phy->SetUseRandomNoise(false); // For now we don't want this
  auto node = ns3::CreateObject<ns3::Node>();
  auto dev = ns3::CreateObject<LoraMeshNetDevice>();
  auto mac = ns3::CreateObject<ns3::MacLoraAca>();
  auto mobility = ns3::CreateObject<ns3::ConstantPositionMobilityModel>();

  auto trans = ns3::CreateObject<ns3::LoraTransducerHd>();

  mobility->SetPosition(pos);
  node->AggregateObject(mobility);
  mac->SetAddress(ns3::LoraAddress::Allocate());

  // Because we want to keep track of the address and location for reporting.
  dev->setAddress(mac->GetAddress());
  dev->setLocation(pos);

  dev->SetPhy(phy);
  dev->SetMac(mac);
  dev->SetChannel(chan);
  dev->SetTransducer(trans);
  node->AddDevice(dev);
  dev->setBQMult(bqMult);

  return dev;
}

// Mesh that transmits on a single channel.
void createMesh(Nodes &nodes, LoraPropModelPtr prop, double width, double depth,
                double height, BroadCastMode mode, double bqMult) {
  LoraChannelPtr channel = ns3::CreateObject<ns3::LoraChannel>();
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    channel->SetAttribute("PropagationModel", ns3::PointerValue(prop));

    // Random location in grid
    auto x = Random::uniform_real<double>(0, width);
    auto y = Random::uniform_real<double>(0, depth);
    auto z = Random::uniform_real<double>(0, height);
    nodes[i] = createNode(ns3::Vector(x, y, z), channel, bqMult);
    nodes[i]->setBroadCastMode(mode);
  }
}

/// Line nodes up on diagonal of space using equidistant spacing
void createLinearMesh(Nodes &nodes, LoraPropModelPtr prop, double width,
                      double depth, double height, BroadCastMode mode,
                      double bqMult) {

  double num_nodes = nodes.size();
  double dx = width / num_nodes;
  double dy = depth / num_nodes;
  double dz = height / num_nodes;
  LoraChannelPtr channel = ns3::CreateObject<ns3::LoraChannel>();
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    channel->SetAttribute("PropagationModel", ns3::PointerValue(prop));

    nodes[i] = createNode(ns3::Vector(i * dx, i * dy, i * dz), channel, bqMult);
    nodes[i]->setBroadCastMode(mode);
  }
}

// Create a map of addresses between nodes where the visibility of
// another node is chosen at random. This can lead to connectivity
// where one node can see another but not vice versa.
AddressMap createVisibilityMap(Nodes &nodes, bool random) {
  // For each node pick a random selection of other nodes that
  // are visible.

  AddressMap aMap;

  for (auto &node : nodes) {
    auto nodeAddr = node->getSavedAddressInt();
    if (random) {
      // Pick a random number of nodes that we want to select
      auto numToSelect = Random::uniform_int<size_t>(0, nodes.size());

      Nodes selectedNodes = nodes;
      std::random_shuffle(selectedNodes.begin(), selectedNodes.end());

      selectedNodes.resize(numToSelect);

      std::vector<AddressInt> visible;
      for (auto &visNode : selectedNodes) {
        visible.push_back(visNode->getSavedAddressInt());
      }
      aMap[nodeAddr] = visible;
    } else {
      for (size_t i = 0; i < nodes.size(); ++i) {
        aMap[nodeAddr].push_back(nodes[i]->getSavedAddressInt());
      }
    }
  }
  return aMap;
}

void connectReceiveCallback(Nodes &nodes) {
  // Connect the receive callback
  for (auto &node : nodes) {
    node->SetReceiveCallback(
        ns3::MakeCallback(&LoraMeshNetDevice::receive, node));
    auto phy = node->GetPhy();
    phy->SetReceiveErrorCallback(
        ns3::MakeCallback(&LoraMeshNetDevice::receiveError, node));
  }
}

std::pair<uint32_t, uint32_t> pickRandomSourceAndDestination(
    uint32_t numNodes) { // Pick a source and destination at random.
  auto source = Random::uniform_int<uint32_t>(0, numNodes - 1);
  auto dest = source;
  do {
    dest = Random::uniform_int<uint32_t>(0, numNodes - 1);
  } while (source == dest);
  return std::make_pair(source, dest);
}

void assignVisibilityMap(Nodes &nodes, const AddressMap &visibilityMap) {
  for (auto &node : nodes) {
    auto address = node->getSavedAddressInt();
    node->setVisibleAddresses(visibilityMap.at(address));
  }
}

void broadcastOnePacketBetween(DevicePtr source, DevicePtr destination) {
  auto pay = LoraMesh::makeSimplePayload(source->getNextPacketID(),
                                         source->getSavedAddressInt(),
                                         destination->getSavedAddressInt());
  source->enqueue(pay);
  // source->broadcastToVisible(pay);
}

void scheduleBroadcastBetween(double transmitStartTime, DevicePtr source,
                              DevicePtr destination) {
  source->SetTransmitStartTime(transmitStartTime);

  ns3::Simulator::Schedule(ns3::Seconds(transmitStartTime),
                           &Experiment::broadcastOnePacketBetween, source,
                           destination);
  // if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
  //  std::cout << "Event: "
  //            << " Scheduled for: " << transmitStartTime << std::endl;
  //}
}

void pickAndSchedule(double recurtime, double maxtime, Nodes &nodes) {
  auto numNodes = nodes.size();

  double transmitTime = 0;
  do {
    // Pick a source and destination at random.
    auto nodePair = Experiment::pickRandomSourceAndDestination(numNodes);
    auto source = nodePair.first;
    auto dest = nodePair.second;

    // Schedule packet to send at next time slot.
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      std::cout << "Schedule broadcast between: " << source << " and " << dest
                << std::endl;
    }

    // Tweak broadcast time.
    // auto p = Random::uniform_real<double>(0, recurtime / 10);
    auto p = 0.0;

    Experiment::scheduleBroadcastBetween(transmitTime + p, nodes[source],
                                         nodes[dest]);

    transmitTime += recurtime;
  } while (transmitTime < maxtime);
}

#define USE_ONE_PACKET_ONLY 0
void scheduleFromAllToOne(double recurtime, double maxtime, Nodes &nodes) {
  auto numNodes = nodes.size();

#if USE_ONE_PACKET_ONLY
  Nodes::size_type maxPackets = 1; // numNodes;
#endif                             // USE_ONE_PACKET_ONLY

  // Destination is always node 0
  Nodes::size_type dest = 0;

  Nodes::size_type count = 0;

  std::cout << blue << "Transmit Schedule:" << std::endl;
  ;

  // We want every node to send on a regular schedule
  for (Nodes::size_type source = 1; source < numNodes; ++source) {
    // Restart
    double transmitTime = 0;
    std::cout << source << " @";
    do {

      // Schedule packet to send at next time slot.
      // if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      // std::cout << "Schedule broadcast between: " << source << " and " <<
      // dest
      //            << std::endl;
      //}

      // Tweak broadcast time.
      auto p = Random::uniform_real<double>(0, recurtime);
      // auto p = 0.0;
      std::cout << " " << transmitTime + p;

      Experiment::scheduleBroadcastBetween(transmitTime + p, nodes[source],
                                           nodes[dest]);
      transmitTime += recurtime;

      count++;

#if USE_ONE_PACKET_ONLY
      if (count >= maxPackets) {
        break;
      }
#endif // USE_ONE_PACKET_ONLY

    } while (transmitTime < maxtime);
    std::cout << std::endl;
#if USE_ONE_PACKET_ONLY
    if (count >= maxPackets) {
      break;
    }
#endif // USE_ONE_PACKET_ONLY
  }
  std::cout << "Total: " << count << reset << std::endl;
}

void checkForStop(const Nodes &nodes, Experiment::Base &experiment) {
  size_t maxPacketsReceived = 0;
  for (auto &node : nodes) {
    maxPacketsReceived = std::max(maxPacketsReceived, node->packetsReceived());
    if (node->getTQ().empty() && !node->getBQ().empty()) {
      // schedule another BQ pop.
      node->scheduleNextBQPop();
    }
    if (!node->getBQ().empty() || !node->getTQ().empty()) {
      return;
    }
  }

#if 0
  const size_t packetLimit = 10000;
  if (maxPacketsReceived > packetLimit) {
    std::cout << boldred << "Stopping Simulation: exceeded " << packetLimit
              << " packets." << reset << std::endl;
    stopSimulationNow();
  }
#endif

  // Wait until simulation has run to completion.
  auto nw_secs = ns3::Simulator::Now().GetSeconds();
  if (nw_secs > experiment.getSimulationDuration()) {
    // After the last packet gets sent a series of follow on events need to
    // be processed for things to complete.
    if (experiment.incrementStop()) {
      stopSimulationNow();
    }
  }
}

void startForwarding(
    Nodes &nodes, Experiment::Base &experiment, double startTime, double period,
    double receiptDelay,
    double packetDelay) { // Schedule relays to be run periodically
                          // near the scheduled time slot
  // frequency.
  for (auto &node : nodes) {
    node->startScheduling(
        startTime, period, receiptDelay, packetDelay,
        [nodes, &experiment] { checkForStop(nodes, experiment); });
  }
}

void initPhyFactory() {
  using namespace ns3;
  LoraModesList modeList;
  LoraTxMode mode = ns3::LoraTxModeFactory::CreateMode(
      ns3::LoraTxMode::LORA, 80, 80, 10000, 4000, 2, "TestMode");
  modeList.AppendMode(ns3::LoraTxMode(mode));

  //  Ptr<LoraPhyPerGenDefault> perDef = CreateObject<LoraPhyPerGenDefault>();

  Ptr<LoraPhyPerUmodem> perDef = CreateObject<LoraPhyPerUmodem>();

  Ptr<LoraPhyCalcSinrDefault> sinrDef = CreateObject<LoraPhyCalcSinrDefault>();
  mPhyFac.SetTypeId("ns3::LoraPhyGen");
  mPhyFac.Set("PerModel", PointerValue(perDef));
  mPhyFac.Set("SinrModel", PointerValue(sinrDef));
  mPhyFac.Set("SupportedModes", LoraModesListValue(modeList));

  LoraTxMode mode00 = LoraTxModeFactory::CreateMode(
      LoraTxMode::LORA, 300, 120, 10000, 125, 2, "TestMode00");
  LoraTxMode mode01 = LoraTxModeFactory::CreateMode(
      LoraTxMode::LORA, 300, 120, 11000, 125, 2, "TestMode01");
  LoraTxMode mode02 = LoraTxModeFactory::CreateMode(
      LoraTxMode::LORA, 300, 120, 12000, 125, 2, "TestMode02");

  LoraModesList m0;
  m0.AppendMode(mode00);
  LoraModesList m1;
  m1.AppendMode(mode01);
  LoraModesList m2;
  m2.AppendMode(mode02);

  // No, why? WTF...
  // mPhyFac = ObjectFactory();
  mPhyFac.SetTypeId("ns3::LoraPhyDual");

  mPhyFac.Set("SupportedModesPhy1", LoraModesListValue(m0));
  mPhyFac.Set("SupportedModesPhy2", LoraModesListValue(m1));
  mPhyFac.Set("SupportedModesPhy3", LoraModesListValue(m2));
}

PropModelPtr makePropogationModel(bool ideal) {
  using namespace ns3;
  if (ideal) {
    return CreateObject<LoraPropModelIdeal>();
  } else {
    return CreateObject<LoraPropModelThorp>();
  }
}

void factory(std::vector<std::unique_ptr<Base>> &mExperiments, int argc,
             char **argv) {
  ns3::CommandLine cmd0;

  // These are positional arguments they come after the Value arguments
  std::string ex1;
  cmd0.AddNonOption("e0", "Run experiment 0", ex1);
  std::string ex2;
  cmd0.AddNonOption("e1", "Run experiment 1", ex2);

  ns3::CommandLine cmd1;
  ns3::CommandLine cmd2;

  // Create experiments on a speculative basis so we can do parsing.
  std::cout << "Making exp0,1, and 2" << std::endl;

  auto exp0 = std::unique_ptr<Base>(new Zero);
  auto exp1 = std::unique_ptr<Base>(new One);
  auto exp2 = std::unique_ptr<Base>(new Two);
  exp0->addValues(cmd0);
  cmd0.Parse(argc, argv);
  std::cout << "ex1: " << ex1 << "ex2: " << ex2 << std::endl;

  exp1->addValues(cmd1);
  cmd1.Parse(argc, argv);

  exp2->addValues(cmd2);
  cmd2.Parse(argc, argv);

  if (ex1 == "ex0" || ex2 == "ex0") {
    std::cout << "taking exp0 " << exp0->getTitle() << std::endl;
    mExperiments.push_back(std::unique_ptr<Base>(exp0.release()));
  }

  if (ex1 == "ex1" || ex2 == "ex1") {
    std::cout << "taking exp1 " << exp1->getTitle() << std::endl;
    mExperiments.push_back(std::unique_ptr<Base>(exp1.release()));
  }

  if (ex1 == "ex2" || ex2 == "ex2") {
    std::cout << "taking exp2 " << exp2->getTitle() << std::endl;
    mExperiments.push_back(std::unique_ptr<Base>(exp2.release()));
  }
}

} // namespace Experiment