#ifndef LORAMESHNETDEVICE_H
#define LORAMESHNETDEVICE_H

#include "ns3/lora-net-device.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"

#include "Experiment.h"
#include "LoraMeshPacket.h"

#include <chrono>
#include <deque>
#include <functional>
#include <set>

class LoraMeshNetDevice : public ns3::LoraNetDevice {
public:
  LoraMeshNetDevice();

  using PacketID = Experiment::PacketID;
  using SecondsType = Experiment::SecondsType;
  using BroadCastMode = Experiment::BroadCastMode;
  using SimPacket = LoraMesh::SimPacket;
  using SimPacketPtr = LoraMesh::SimPacketPtr;
  using ConstSimPacketPtr = LoraMesh::ConstSimPacketPtr;
  using Address = SimplePacketBase::Address;
  using AddressInt = SimplePacketBase::AddressInt;
  using IDAddress = Experiment::IDAddress;
  using VisibleAddressArray = Experiment::VisibleAddressArray;
  using AddressMap = Experiment::AddressMap;
  using IDAddressSet = Experiment::IDAddressSet;
  using SentPacketMap = Experiment::SentPacketMap;
  using PacketData = LoraMesh::PacketData;
  using Queue = std::deque<PacketData>; // deque allows iteration and removal
  using PacketIndex = std::map<ns3::LoraAddress::LoraAddressInt, size_t>;
  using Vector = ns3::Vector;
  using PacketTracker = std::map<IDAddress, LoraMesh::PacketStatus>;
  using BadPacketTracker = std::map<uint64_t, size_t>;
  using TimeTracker =
      std::vector<std::pair<SecondsType, SimplePacketBase::Command>>;
  using Callback = std::function<void()>;

  bool receive(ns3::Ptr<NetDevice> device, ns3::Ptr<const ns3::Packet> packet,
               uint16_t mode, const Address &sender);
  void receiveError(ns3::Ptr<ns3::Packet>, double sinr);

  void setVisibleAddresses(const VisibleAddressArray &visibleAddresses);
  VisibleAddressArray getVisibleAddresses() const { return mVisibleAddresses; }
  void setBroadCastMode(BroadCastMode mode);
  void broadcastToVisible(const PacketData &packet);
  void enqueue(const PacketData &packet);
  void forwardAPacketNow();
  void startScheduling(double firstStart, double schedulePeriod,
                       double receiptDelay, double packetDelay,
                       Callback bqEmpty);

  size_t packetsReceived() const { return mPacketsReceived; }

  void processReceivedPacket(const PacketData &packet, uint64_t ns3PacketID);

  const SentPacketMap &getSentPackets() const { return mSentPackets; }
  const SentPacketMap &getFailedToSendPackets() const {
    return mFailedToSendPackets;
  }
  const SentPacketMap &getFailedToReceivePackets() const {
    return mFailedToReceivePackets;
  }
  const IDAddressSet &getPacketsSeen() { return mVisitorList; }

  void setAddress(Address addr) { mSavedAddress = addr; }
  const Address &getSavedAddress() const { return mSavedAddress; }
  int getSavedAddressInt() const { return mIntAddress; }
  size_t getNodeIndex() const { return mNodeIndex[mIntAddress]; }

  /// \TODO Static should be moved to more appropriate container.
  static size_t lookUpNodeIndex(ns3::LoraAddress::LoraAddressInt addrint) {
    return mNodeIndex[addrint];
  }

  void setLocation(const Vector &loc) { mLocation = loc; }
  Vector getLocation() const { return mLocation; }

  static const PacketTracker &getPacketTracker() { return mPacketTracker; }
  static const BadPacketTracker &getBadPacketTracker() {
    return mBadPacketTracker;
  }
  static const TimeTracker &sendTimeTracker() { return mSendTimeTracker; }
  static const TimeTracker &receiveTimeTracker() { return mReceiveTimeTracker; }
  void trackSendTime(SimplePacketBase::Command command) {
    SecondsType nw = ns3::Simulator::Now().GetSeconds();
    if (nw < Experiment::durationLimit()) {
      mSendTimeTracker.emplace_back(std::make_pair(nw, command));
    }
  }
  void trackReceiveTime(SimplePacketBase::Command command) {
    SecondsType nw = ns3::Simulator::Now().GetSeconds();
    if (nw < Experiment::durationLimit()) {
      mReceiveTimeTracker.emplace_back(std::make_pair(nw, command));
    }
  }

  static void clearPacketTrackers() {
    mPacketTracker.clear();
    mBadPacketTracker.clear();
    mSendTimeTracker.clear();
    mReceiveTimeTracker.clear();
    mNodeIndex.clear();
    mNodeCounter = 0;
    mTimerStart = std::chrono::high_resolution_clock::now();
  }

  static LoraMesh::PacketStatus *getTracked(IDAddress &idAddress) {
    auto iter = mPacketTracker.find(idAddress);
    if (iter != mPacketTracker.end()) {
      return &iter->second;
    } else {
      return nullptr;
    }
  }

  const Queue &getTQ() const { return mTQ; }
  size_t getTQMax() const { return mTQMax; }
  size_t getBQMax() const { return mBQMax; }
  const Queue &getBQ() const { return mBQ; }
  void setBQMult(double mult) { mBQmult = mult; }

  PacketID getNextPacketID(); // should be private.
  void scheduleNextBQPop();

private:
  void enqueueBQ(const PacketData &packet);
  void enqueueTQ(const PacketData &packet, bool front = false);
  void dumpQ(const Queue &q);
  bool seenBefore(const PacketData &packet);
  bool isPacketInTQ(const IDAddress &idAddress);
  bool isPacketInBQ(const IDAddress &idAddress);
  void removePacketFromTQ(const IDAddress &idAddress);
  void removePacketFromBQ(const IDAddress &idAddress);
  void removePacketFromTQ(const PacketData &packet);
  void removePacketFromBQ(const PacketData &packet);
  void processReceiptPacket(const SimplePacketBase::Ptr &packet_ptr);
  void processReceipts(const SimpleReceiptPacket &receipt);
  void processDestinationReceipt(const IDAddress &trackedPacketIDAddress,
                                 const IDAddress &trackingPacketIDAddress);
  void forwardDestinationReceipt(const SimpleReceiptPacket &receipt);

  void processMeasurementPacket(const SimplePacketBase::Ptr &packet_ptr);
  void addToBQIfNeeded(const PacketData &packet);

  void startTracking(const SimplePacketBase::Ptr &packet_ptr);

  void doDestinationProcessing(const SimplePayloadPacket &payloadPacket);
  void queuePacketForForwarding(const SimplePayloadPacket &payloadPacket);
  void sendNodeReceipt(const SimplePacket &packet);

  void scheduleNextRelative(double delay);
  void scheduleNextAbsolute(double actualTime);
  void scheduleAReceiptSend();
  void periodicSchedule();

  void popBQNow();

  std::string logStamp();
  void mapNodeAddress();
  std::string idOut(const LoraMesh::PacketData &packetData);
  std::string idOut(const SimplePacketBase::Ptr &packet_ptr);

private:
  Queue mTQ;
  Queue mBQ;
  IDAddressSet mVisitorList;
  VisibleAddressArray mVisibleAddresses;
  BroadCastMode mBroadCastMode;
  Address mSavedAddress;
  ns3::LoraAddress::LoraAddressInt mIntAddress;
  Vector mLocation;
  size_t mPacketsReceived;

  double mSchedulePeriod;
  double mReceiptDelay;
  double mPacketDelay;
  double mBQmult;
  double mNextPopTime;

  PacketID mIDCounter;

  // Data collection
  SentPacketMap mSentPackets;
  SentPacketMap mFailedToSendPackets;
  SentPacketMap mFailedToReceivePackets;
  size_t mTQMax;
  size_t mBQMax;
  Callback mCheckForStop;

  static PacketTracker mPacketTracker;
  static BadPacketTracker mBadPacketTracker;
  static TimeTracker mSendTimeTracker;
  static TimeTracker mReceiveTimeTracker;
  static PacketIndex mNodeIndex;
  static size_t mNodeCounter;
  static std::chrono::time_point<std::chrono::system_clock> mTimerStart;
};

#endif // LORAMESHNETDEVICE_H