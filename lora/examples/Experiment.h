#ifndef EXPERIMENT_H
#define EXPERIMENT_H

//#include "LoraMeshNetDevice.h"
#include "SimplePacket.h"

#include "ns3/command-line.h"
#include "ns3/lora-channel.h"
#include "ns3/lora-prop-model.h"
#include "ns3/pointer.h"

#include <filesystem>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <vector>

class LoraMeshNetDevice;

// Base class for running LoraMesh experiments
namespace Experiment {

using LoraPropModelPtr = ns3::Ptr<ns3::LoraPropModel>;
using LoraChannelPtr = ns3::Ptr<ns3::LoraChannel>;
using DevicePtr = ns3::Ptr<LoraMeshNetDevice>;
using Nodes = std::vector<DevicePtr>;
using Address = SimplePacketBase::Address;
using AddressInt = SimplePacketBase::AddressInt;
using VisibleAddressArray = std::vector<AddressInt>;
using AddressMap = std::map<AddressInt, VisibleAddressArray>;
using PacketID = SimplePacket::PacketID;
using IDAddress = std::tuple<AddressInt, PacketID>;
using IDAddressSet = std::set<IDAddress>;
using SentPacketMap = std::map<AddressInt, std::vector<IDAddress>>;
using BroadCastMode = uint16_t;
using PropModelPtr = ns3::Ptr<ns3::LoraPropModel>;
using SecondsType = float;

using CommandLine = ns3::CommandLine;
namespace fs = std::filesystem;

class Base;

enum Type { Baseline = 0, NodesToGateway };
enum Verbosity : int {
  Error = 0,
  Report = 1,
  Warning = 2,
  Info = 3,
  Special = 4
};

void initPhyFactory();
PropModelPtr makePropogationModel(bool ideal);
DevicePtr createNode(ns3::Vector pos, LoraChannelPtr chan, double bqMult);
void createMesh(Nodes &nodes, LoraPropModelPtr prop, double width, double depth,
                double height, BroadCastMode mode, double bqMult);
void createLinearMesh(Nodes &nodes, LoraPropModelPtr prop, double width,
                      double depth, double height, BroadCastMode mode,
                      double bqMult);
AddressMap createVisibilityMap(Nodes &nodes, bool random);
void assignVisibilityMap(Nodes &nodes, const AddressMap &visibilityMap);
void connectReceiveCallback(Nodes &nodes);
std::pair<uint32_t, uint32_t> pickRandomSourceAndDestination(uint32_t numNodes);
void broadcastOnePacketBetween(DevicePtr source, DevicePtr destination);
void scheduleBroadcastBetween(double transmitStartTime, DevicePtr source,
                              DevicePtr destination);
void startForwarding(Nodes &nodes, Experiment::Base &experiment,
                     double startTime, double period, double receiptDelay,
                     double packetDelay);
void pickAndSchedule(double recurtime, double maxtime, Nodes &nodes);
void scheduleFromAllToOne(double recurtime, double maxtime, Nodes &nodes);
int getVerbosity();
void setVerbosity(int v);
void setSeed(unsigned int seed);
double getCorruptionProb();
void setCorruptionProb(double p);
fs::path getExpSubPath();
void setBasePath(const std::string &path);
void duration(SecondsType);
SecondsType duration();
SecondsType durationLimit();

fs::path makeExperimentsDir();

class Base {
public:
  Base() : mStopCount(0){};
  virtual ~Base(){};

  virtual void addValues(CommandLine &cmd) = 0;

  virtual void run(int runNo) = 0;
  virtual std::string getTitle() = 0;
  virtual std::string getName() = 0;
  virtual std::ostream &reportOptions(std::ostream &s) = 0;
  virtual std::ostream &report(std::ostream &s, int runNo) = 0;
  virtual void exportResults(int runNo) = 0;

  virtual int numRuns() { return 1; }

  void initRun();
  std::ostream &reportHeader(std::ostream &s, int runNo);

  /// Increments stop count and returns true if it is greater than 1.
  bool incrementStop() {
    mStopCount++;
    return mStopCount > 1;
  }
  virtual double getSimulationDuration() = 0;
  const fs::path &getOutputDir() const { return mOutputDir; }

private:
  unsigned int mStopCount;
  fs::path mOutputDir;
};

class Zero : public Base {
public:
  Zero()
      : Base(), mNumNodes(10), mNodes(), mSimulationDuration(10000),
        mMeshSize(5000), mMeshType(0), mPeriod(60.0), mReceiptDelay(2),
        mPacketDelay(10), mBQMult(10.0), mUseThorpProp(true), mRecurTime(500.0),
        mVerbose(0), mNumRuns(1), mSeed(0), mCorruptionProb(0),
        mExpSubPath("/exp") {}

  void addValues(CommandLine &cmd) override {
    cmd.AddValue("size", "Number of nodes.", mNumNodes);
    cmd.AddValue("time", "Simulation time, s.", mSimulationDuration);
    cmd.AddValue("mesh_size", "X Y size of mesh", mMeshSize);
    cmd.AddValue("mesh_type", "type of mesh, 0, 1", mMeshType);
    cmd.AddValue("bq_mult", "Multiplier for BQ transmmit", mBQMult);
    cmd.AddValue("period", "Transmit period", mPeriod);
    cmd.AddValue("rdelay", "Delay untill next receipt transmit", mReceiptDelay);
    cmd.AddValue("pdelay", "Delay untill next regular packet transmit",
                 mPacketDelay);
    cmd.AddValue("thorp", "Propgation model (false == ideal)", mUseThorpProp);
    cmd.AddValue("repeat", "Time between repeating transmits", mRecurTime);
    cmd.AddValue("v", "Verbosity of output 0, 1, 2... higher is more",
                 mVerbose);
    cmd.AddValue("r", "Number of runs", mNumRuns);
    cmd.AddValue("seed", "Random number seed", mSeed);
    cmd.AddValue("corrupt", "Packet Corruption Probability", mCorruptionProb);
    cmd.AddValue("out", "Sub path for output", mExpSubPath);
  }

  std::string getTitle() override { return "Experiment 0"; }
  std::string getName() override { return "Exp0"; }
  std::ostream &reportOptions(std::ostream &s) override {
    s << "Nodes: " << mNumNodes << std::endl;
    s << "Duration: " << mSimulationDuration << std::endl;
    s << "Mesh X x Y: " << mMeshSize << std::endl;
    s << "Mesh Type: " << mMeshType << std::endl;
    s << "Period: " << mPeriod << std::endl;
    s << "Receipt Delay: " << mReceiptDelay << std::endl;
    s << "Packet Delay: " << mPacketDelay << std::endl;
    s << "BQ Mult: " << mBQMult << std::endl;
    s << "Propgation Model: " << (mUseThorpProp ? "Thorp" : "Ideal")
      << std::endl;
    s << "verbosity: " << mVerbose << std::endl;
    s << "runs: " << mNumRuns << std::endl;
    Experiment::setVerbosity(mVerbose);
    s << "seed: " << mSeed << std::endl;
    s << "corrupt: " << mCorruptionProb << std::endl;
    s << "out:" << mExpSubPath << std::endl;
    Experiment::setBasePath(mExpSubPath);
    Experiment::setCorruptionProb(mCorruptionProb);
    Experiment::setSeed(mSeed);
    Experiment::duration(mSimulationDuration);
    return s;
  }

  void run(int runNo) override;
  double getSimulationDuration() override { return mSimulationDuration; };

  int numRuns() override { return mNumRuns; }

  std::ostream &report(std::ostream &s, int runNo) override;
  void exportResults(int runNo) override;

private:
  uint32_t mNumNodes;
  Nodes mNodes;
  double mSimulationDuration; // Simulation time, seconds
  double mMeshSize;           // X Y mesh size.
  int mMeshType;              // 0 random X*Y grid, 1 linear X*Y grid on diagonl
  double mPeriod;             // Transmit period
  double mReceiptDelay;       // Delay until next receipt transmit.
  double mPacketDelay;        // Delay until next packet transmit
  double mBQMult;             // Multiplier on mPeriod for BQ retransmit.
  bool mUseThorpProp;         // Use Thorp propogation else ideal.
  double mRecurTime;          // Time between recurring transmissions.
  int mVerbose;               // Verbosity of output 0, 1, 2 higher is more
  int mNumRuns;               // Number of times to run experiment
  unsigned int mSeed;         // Random number generator seed.
  double mCorruptionProb;     // Probability of packet corruption 0-1.
  std::string mExpSubPath;    // Sub path for output
};

class One : public Base {
public:
  One()
      : Base(), mNumNodes(10), mNodes(), mSimulationDuration(10000),
        mMeshSize(5000), mMeshType(0), mPeriod(120.0), mReceiptDelay(2),
        mPacketDelay(10), mBQMult(10.0), mUseThorpProp(true), mRecurTime(500.0),
        mVerbose(0), mNumRuns(1), mSeed(0), mCorruptionProb(0),
        mExpSubPath("/exp") {}

  void addValues(CommandLine &cmd) override {
    cmd.AddValue("size", "Number of nodes.", mNumNodes);
    cmd.AddValue("time", "Simulation time, s.", mSimulationDuration);
    cmd.AddValue("mesh_size", "X Y size of mesh", mMeshSize);
    cmd.AddValue("mesh_type", "type of mesh, 0, 1", mMeshType);
    cmd.AddValue("bq_mult", "Multiplier for BQ transmmit", mBQMult);
    cmd.AddValue("period", "Transmit period", mPeriod);
    cmd.AddValue("rdelay", "Delay untill next receipt transmit", mReceiptDelay);
    cmd.AddValue("pdelay", "Delay untill next regular packet transmit",
                 mPacketDelay);
    cmd.AddValue("thorp", "Propgation model (false == ideal)", mUseThorpProp);
    cmd.AddValue("repeat", "Time between repeating transmits", mRecurTime);
    cmd.AddValue("v", "Verbosity of output 0, 1, 2... higher is more",
                 mVerbose);
    cmd.AddValue("r", "Number of runs", mNumRuns);
    cmd.AddValue("seed", "Random number seed", mSeed);
    cmd.AddValue("corrupt", "Packet Corruption Probability", mCorruptionProb);
    cmd.AddValue("out", "Sub path for output", mExpSubPath);
  }

  std::string getTitle() override { return "Experiment 1"; }
  std::string getName() override { return "Exp1"; }
  std::ostream &reportOptions(std::ostream &s) override {
    s << "Nodes: " << mNumNodes << std::endl;
    s << "Duration: " << mSimulationDuration << std::endl;
    s << "Mesh X x Y: " << mMeshSize << std::endl;
    s << "Mesh Type: " << mMeshType << std::endl;
    s << "Period: " << mPeriod << std::endl;
    s << "Receipt Delay: " << mReceiptDelay << std::endl;
    s << "Packet Delay: " << mPacketDelay << std::endl;
    s << "BQ Mult: " << mBQMult << std::endl;
    s << "Propgation Model: " << (mUseThorpProp ? "Thorp" : "Ideal")
      << std::endl;
    s << "Recurance Time:" << mRecurTime << std::endl;
    s << "verbosity: " << mVerbose << std::endl;
    s << "runs: " << mNumRuns << std::endl;
    s << "seed: " << mSeed << std::endl;
    s << "corrupt: " << mCorruptionProb << std::endl;
    s << "out:" << mExpSubPath << std::endl;
    Experiment::setBasePath(mExpSubPath);
    Experiment::setCorruptionProb(mCorruptionProb);
    Experiment::setSeed(mSeed);
    Experiment::duration(mSimulationDuration);
    return s;
  }
  void run(int runNo) override;
  double getSimulationDuration() override { return mSimulationDuration; };

  int numRuns() override { return mNumRuns; }
  std::ostream &report(std::ostream &s, int runNo) override;
  void exportResults(int runNo) override;

protected:
  uint32_t mNumNodes;
  Nodes mNodes;
  double mSimulationDuration; // Simulation time, seconds
  double mMeshSize;           // X Y mesh size.
  int mMeshType;              // 0 random X*Y grid, 1 linear X*Y grid on diagonl
  double mPeriod;             // Transmit period
  double mReceiptDelay;       // Delay until next Receipt transmit.
  double mPacketDelay;        // Delay until next packet transmit
  double mBQMult;             // Multiplier on mPeriod for BQ retransmit.
  bool mUseThorpProp;         // Use Thorp propogation else ideal.
  double mRecurTime;          // Time between recurring transmissions.
  int mVerbose;               // Verbosity of output 0, 1, 2 higher is more.
  int mNumRuns;               // Number of times to run experiment
  unsigned int mSeed;         // Random number generator seed.
  double mCorruptionProb;     // Probability of packet corruption 0-1.
  std::string mExpSubPath;    // Sub path for output
};

class Two : public One {
public:
  Two() : One() {}

  std::string getTitle() override { return "Experiment 2"; }
  std::string getName() override { return "Exp2"; }
  std::ostream &reportOptions(std::ostream &s) override {
    One::reportOptions(s);
    return s;
  }
  void run(int runNo) override;
  std::ostream &report(std::ostream &s, int runNo) override;
  void exportResults(int runNo) override;

private:
};

void factory(std::vector<std::unique_ptr<Base>> &mExperiments, int argc,
             char **argv);

} // namespace Experiment
#endif // EXPERIMENT_H