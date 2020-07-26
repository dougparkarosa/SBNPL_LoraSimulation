#include "LoraMeshNetDevice.h"
#include "DebugBreak.h"
#include "Experiment.h"
#include "RandomNumbers.h"
#include "TermColors.h"

#include "Ansi.h"
#include "ns3/channel.h"
#include "ns3/log.h"
#include "ns3/lora-header-common.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cassert>

#define USE_NODE_RECEIPT 0
#define USE_COMBINED_PACKET 0

namespace {
template <typename T, typename Pred> int eraseFrom(T &c, Pred pred) {
  int count = 0;
  for (auto it = c.begin(); it != c.end();) {
    if (pred(*it)) {
      it = c.erase(it);
      count++;
    } else {
      ++it;
    }
  }
  return count;
}

#if 0
// To help with figuring out if something really is a measurement until
// there is time to refactor the packet structs into a proper class.
bool isMeasurement(const SimplePayloadPacket &measurement) {
  if (measurement.command() == SimplePacket::Command::Measurement) {
    return true;
  }
  return false;
}

bool isDestReceipt(const LoraMesh::SimpleReceiptPacket *receipt) {
  if (receipt) {
    if (receipt->mHeader.mCommand == LoraMesh::Command::DestinationReceipt) {
      return true;
    }
  }
  return false;
}
#endif

bool headerLooksCorrupt(const SimplePacket::Header &header) {
  if (!(header.command() >= SimplePacket::Command::Measurement &&
        header.command() <= SimplePacket::Command::CombinedPacket)) {
    // corrupted.
    return true;
  }
  return false;
}

/// Generate now time as a string with "s" appended to it
/// for use in logs.
///
/// \return string with time in seconds with "s" after it.
std::string nowtime() {
  std::stringstream s;
  s << ns3::Simulator::Now().GetSeconds() << "s ";
  return s.str();
}

/// Current simulation time in seconds.
/// \return double current simulation time in seconds.
double now() { return ns3::Simulator::Now().GetSeconds(); }

std::string idOut(size_t nodeNum, LoraMesh::PacketID packetID,
                  SimplePacketBase::Command command) {
  std::stringstream s;
  s << "{" << nodeNum << ", " << packetID << "}"
    << " " << LoraMesh::toString(command);
  return s.str();
}

} // namespace

/// Format a compact ID string with packet type appended.
///
/// \return String of the form {node#, pktid} PacketTypeString
/// Something like "{1, 0} Measurement"
std::string LoraMeshNetDevice::idOut(const SimplePacketBase::Ptr &packet_ptr) {
  auto idAddress = LoraMesh::IDAddressFromPacketPtr(packet_ptr);
  auto intaddr = std::get<0>(idAddress);
  return ::idOut(mNodeIndex[intaddr], std::get<1>(idAddress),
                 packet_ptr->command());
}

std::string LoraMeshNetDevice::idOut(const LoraMesh::PacketData &packetData) {
  auto idAddress = LoraMesh::IDAddressFromPacketData(packetData);
  auto intaddr = std::get<0>(idAddress);
  return ::idOut(mNodeIndex[intaddr], std::get<1>(idAddress),
                 commandIDFromPacketData(packetData));
}

std::string LoraMeshNetDevice::logStamp() {
  mapNodeAddress();
  std::stringstream s;
  if (mSavedAddress.IsInvalid()) {
    s << -1 << ": " << nowtime();
  } else {
    auto nodeNo = mNodeIndex[LoraMesh::getIntAddress(mSavedAddress)];
    s << nodeNo << ": " << nowtime();
  }
  return s.str();
}

/// Static trackers. Must be cleared externally with call to
/// LoraMeshNetDevice::clearPacketTrackers() after each run.
LoraMeshNetDevice::PacketTracker LoraMeshNetDevice::mPacketTracker;
LoraMeshNetDevice::BadPacketTracker LoraMeshNetDevice::mBadPacketTracker;
LoraMeshNetDevice::TimeTracker LoraMeshNetDevice::mSendTimeTracker;
LoraMeshNetDevice::TimeTracker LoraMeshNetDevice::mReceiveTimeTracker;
LoraMeshNetDevice::PacketIndex LoraMeshNetDevice::mNodeIndex;
size_t LoraMeshNetDevice::mNodeCounter = 0;
std::chrono::time_point<std::chrono::high_resolution_clock>
    LoraMeshNetDevice::mTimerStart;

/// Constructor
LoraMeshNetDevice::LoraMeshNetDevice()
    : LoraNetDevice(), mTQ(), mBQ(), mVisitorList(), mVisibleAddresses(),
      mBroadCastMode(1), mSavedAddress(), mIntAddress(0xff),
      mPacketsReceived(0), mSchedulePeriod(120), mReceiptDelay(1),
      mPacketDelay(10), mBQmult(10.0), mNextPopTime(0), mIDCounter(0),
      mSentPackets(), mFailedToSendPackets(), mFailedToReceivePackets(),
      mTQMax(0), mBQMax(0) {
  // SetReceiveCallback(MakeCallback(&LoraMeshNetDevice::receive, this));
}

/// Receive callback as hooked in Constructor,
/// Also hooked in Experiment::connectReceiveCallback
///
/// \todo double check device documentation
/// \param device The LoraMeshNetDevice that sent the packet

/// \param packet_in  ns3 packet that was sent
/// \param mode Mode that was used to send the packet.
/// \param sender ns3 Address of LoraMeshNetDevice that sent the packet
///
/// \return true if handled. Currently always true.
bool LoraMeshNetDevice::receive(ns3::Ptr<NetDevice> device,
                                ns3::Ptr<const ns3::Packet> packet_in,
                                uint16_t mode, const Address &sender)

{
  // MacLoraAca::RxPacketGood removes header, so we don't need to worry about
  // doing so here.
  auto packetData = LoraMesh::extractPayload(ns3::GetPointer(packet_in));
  processReceivedPacket(packetData, packet_in->GetUid());

  mPacketsReceived++;

  return true;
}

/// Callback for receive error hooked in Experiment::connectReceiveCallback
///
/// \param packet_in Input ns3 packet that was unable to be received.
/// \param sinr Signal to Noise ratio. (May be the reason for failure.)
void LoraMeshNetDevice::receiveError(ns3::Ptr<ns3::Packet> packet_in,
                                     double sinr) {
  // packet_in->RemoveHeader();
  ns3::LoraHeaderCommon header;
  packet_in->RemoveHeader(header);
  auto address = getSavedAddressInt();
  auto packetData = LoraMesh::extractPayload(ns3::GetPointer(packet_in));
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Warning) {
    std::cout << ">>>>>>> Packet Receive Error: " << idOut(packetData)
              << " SINR = " << sinr << " at " << mNodeIndex[address]
              << std::endl;
  }
  auto idAddress = LoraMesh::IDAddressFromPacketData(packetData);
  mFailedToReceivePackets[address].push_back(idAddress);
}

/// Return a probably unique to this device packet ID.
/// ID is unsigned and allowed to purposely wrap around. The idea
/// is that packets will be produced and consumed quickly enough that
/// if wraparound occurs there won't be more than 1 with the same ID
/// in the system at any given time.
///
/// \return next available packet ID. Starts at zero and increments up.
LoraMeshNetDevice::PacketID LoraMeshNetDevice::getNextPacketID() {
  return mIDCounter++;
}

/// Process a SimplePayloadPacket that has reached its desired
/// destination. This will check and report on duplicate receipt.
/// It will generate a destination receipt every time a packet is
/// received, even if it is a duplicate.
///
/// \payloadPacket Packet to process.
void LoraMeshNetDevice::doDestinationProcessing(
    const SimplePayloadPacket &payloadPacket) {
  auto ourAddressInt = mIntAddress;

  auto sourceIDAddress = LoraMesh::originIDAddress(payloadPacket);
  if (LoraMesh::isBad(sourceIDAddress)) {
    std::cout << boldred << "Bad mPacket " << reset << std::endl;
  }

  if (mVisitorList.find(sourceIDAddress) != mVisitorList.end()) {
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Warning) {
      std::cout << logStamp() << "----Packet " << sourceIDAddress
                << " already processed, ignoring duplicate----" << std::endl;
    }
  } else {
    mPacketTracker[sourceIDAddress].mReceived = true;
    mPacketTracker[sourceIDAddress].mReceivedAt = now();
    mPacketTracker[sourceIDAddress].mAddressTrack.push_back(ourAddressInt);
  }
  mPacketTracker[sourceIDAddress].mDestCount++;

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "----Packet " << sourceIDAddress << " "
              << LoraMesh::toString(payloadPacket.header().command())
              << " at destination " << ourAddressInt << "----" << std::endl;
  }
  // We got where we wanted to.
  // For simulation the packet is now dead just toss it.
  // Put a destination receipt into the mTQ.
  auto payload = LoraMesh::makeSimpleDestinationReceipt(
      payloadPacket, ourAddressInt, getNextPacketID());

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "Created Destination Receipt " << idOut(payload)
              << std::endl;
  }
  // This will get scheduled for the next receive window by caller.
  enqueueTQ(payload);
}

void LoraMeshNetDevice::queuePacketForForwarding(
    const SimplePayloadPacket &payloadPacket) {

#if USE_COMBINED_PACKET
  // Make a combined packet and put it in our TQ.
  // std::cout << "add to mTQ" << std::endl;
  auto ourAddressInt = mIntAddress;

  auto receipt = LoraMesh::makeSimpleNodeReceiptPacket(
      payloadPacket, ourAddressInt, getNextPacketID());
  auto packetToBroadCast = LoraMesh::makeCombinedPacket(payloadPacket, receipt);
  auto dataToBroadCast =
      LoraMesh::extractPayload(ns3::GetPointer(packetToBroadCast));
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "packet " << idOut(dataToBroadCast) << " built"
              << std::endl;
  }
  enqueueTQ(dataToBroadCast);
#else
  // If we have seen this measurement before, we shouldn't forward it,
  // It is the job of the BQ to make that happen if needed.
  auto sourceIDAddress = LoraMesh::originIDAddress(payloadPacket);
  if (mVisitorList.find(sourceIDAddress) == mVisitorList.end()) {
    auto ourAddressInt = mIntAddress;

    auto receipt = LoraMesh::makeSimpleNodeReceiptPacket(
        payloadPacket, ourAddressInt, getNextPacketID());
    auto receiptPay = LoraMesh::toPacketData(receipt);
    enqueueTQ(receiptPay);
    auto payload = LoraMesh::toPacketData(payloadPacket);
    enqueueTQ(payload);
  }
#endif // USE_COMBINED_PACKET
}

/// Queue a node receipt for a packet
void LoraMeshNetDevice::sendNodeReceipt(const SimplePacket &packet) {

  auto receipt = LoraMesh::makeSimpleNodeReceiptPacket(packet, mIntAddress,
                                                       getNextPacketID());
  auto dataToBroadCast = LoraMesh::toPacketData(receipt);

  enqueueTQ(dataToBroadCast);
}

void LoraMeshNetDevice::processReceiptPacket(
    const SimplePacketBase::Ptr &packet_ptr) {
  auto receipt_ptr =
      dynamic_cast<const SimpleReceiptPacket *>(packet_ptr.get());
  // assert(receipt_ptr);
  if (receipt_ptr == nullptr) {
    auto cmd = packet_ptr->command();
    if (cmd == SimplePacketBase::Command::NodeReceipt) {
      std::cout << logStamp() << "cmd == SimplePacketBase::Command::NodeReceipt"
                << std::endl;
    }
  }
  processReceipts(*receipt_ptr);
}

void LoraMeshNetDevice::processDestinationReceipt(
    const IDAddress &trackedPacketIDAddress,
    const IDAddress &trackingPacketIDAddress) {
  if (LoraMesh::isBad(trackedPacketIDAddress)) {
    std::cout << boldred << logStamp() << "Bad mPacket " << reset << std::endl;
  }
  if (LoraMesh::isBad(trackingPacketIDAddress)) {
    std::cout << boldred << logStamp() << "Bad mPacket " << reset << std::endl;
  }

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "DestinationReceipt from "
              << trackingPacketIDAddress << " for packet "
              << trackedPacketIDAddress << " received by sender." << std::endl;
  }
  mPacketTracker[trackedPacketIDAddress].mDestReceiptAtSender = true;
  mPacketTracker[trackedPacketIDAddress].mReceiptReceivedAt = now();
  mPacketTracker[trackedPacketIDAddress].mReceiptCount++;
  // mPacketTracker[idAddress].mAddressTrack.push_back(GetAddress());
  // Also tracking the receipt.
  mPacketTracker[trackingPacketIDAddress].mDestCount++;
  mPacketTracker[trackingPacketIDAddress].mReceived = true;
  mPacketTracker[trackingPacketIDAddress].mReceivedAt = now();
  mPacketTracker[trackingPacketIDAddress].mAddressTrack.push_back(
      getSavedAddressInt());
  // if (mPacketTracker[trackingPacketIDAddress].mAddressTrack.size() > 10) {
  //  DebugBreak();
  //
}

void LoraMeshNetDevice::forwardDestinationReceipt(
    const SimpleReceiptPacket &receipt) {
  // Forward if we haven't seen this packet before
  // Otherwise ignore it.
  auto packetData = LoraMesh::toPacketData(receipt);
  // mVisitorList is updated after we return.

  if (!seenBefore(packetData)) {
    // mPacketTracker[receiptIDAddress].mAddressTrack.push_back(GetAddress());
    enqueueTQ(packetData);
  }
}

void LoraMeshNetDevice::processReceipts(const SimpleReceiptPacket &receipt) {

  // ID of the packet that needed a receipt.
  auto trackedPacketIDAddress = LoraMesh::trackedIDAddress(receipt);
  // ID of the packet carrying the receipt
  auto trackingPacketIDAddress = LoraMesh::originIDAddress(receipt);

  bool receiptIsForThisNode = receipt.nodeGettingReceipt() == mIntAddress;

  switch (receipt.command()) {
  case LoraMesh::Command::DestinationReceipt:
    if (receiptIsForThisNode) {
      processDestinationReceipt(trackedPacketIDAddress,
                                trackingPacketIDAddress);
    }
    removePacketFromBQ(trackedPacketIDAddress);
    if (!receiptIsForThisNode) {
#if USE_NODE_RECEIPT
      sendNodeReceipt(receipt);
#endif // USE_NODE_RECEIPT
      // If this packet is in the BQ remove it
      if (receipt.header().originatingNode() == mIntAddress) {
        // We sent this, don't need to forward it.
        // Permanently forget it, it is now in the wild.
        removePacketFromBQ(trackingPacketIDAddress);
      } else {
        if (isPacketInBQ(trackingPacketIDAddress)) {
          // We sent this already, it has now come back
          // So it is propogating.
          removePacketFromBQ(trackingPacketIDAddress);
        } else if (isPacketInTQ(trackingPacketIDAddress)) {
          // We saw this and queued it already.
          // Someone else sent it already. So it is
          // propogating.
          removePacketFromBQ(trackingPacketIDAddress);
        } else {
          forwardDestinationReceipt(receipt);
        }
      }
    }
    break;
  case LoraMesh::Command::NodeReceipt:
    // Node receipts are never forwarded. They are used as broadcast flood
    // control
    if (receiptIsForThisNode) {
      if (Experiment::getVerbosity() >= Experiment::Verbosity::Warning) {
        std::cout << logStamp() << " NodeReceipt from "
                  << trackingPacketIDAddress << " for packet "
                  << trackedPacketIDAddress << " received by sender."
                  << std::endl;
      }
    }
    removePacketFromTQ(trackedPacketIDAddress);
    // removePacketFromBQ(trackedPacketIDAddress);
    break;
  default:
    break;
  }
}

void LoraMeshNetDevice::processMeasurementPacket(
    const SimplePacketBase::Ptr &packet_ptr) {

  assert(packet_ptr->hasPayload());
  auto measurement = packet_ptr->getPayloadPtr();
  assert(measurement &&
         measurement->command() == SimplePacket::Command::Measurement);

  if (measurement) {
    if (measurement->header().destination() == mIntAddress) {
      doDestinationProcessing(*measurement);
    } else {
      queuePacketForForwarding(*measurement);
    }
  }

  if (packet_ptr->command() == SimplePacket::Command::CombinedPacket) {
    auto combined =
        dynamic_cast<CombinedPayloadNodeReceipt *>(packet_ptr.get());
    processReceipts(combined->receiptPacket());
  }
}

void LoraMeshNetDevice::mapNodeAddress() {
  // if (mSavedAddress.IsInvalid()) {
  auto ourAddress = GetAddress();
  // In case we need it after the simulation is done.
  mSavedAddress = ourAddress;
  mIntAddress = LoraMesh::getIntAddress(mSavedAddress);
  if (mNodeIndex.find(mIntAddress) == mNodeIndex.end()) {
    mNodeIndex[mIntAddress] = mNodeCounter++;
  }
  //}
}

void LoraMeshNetDevice::processReceivedPacket(const PacketData &packetData,
                                              uint64_t ns3PacketID) {
  // add to transmit queue
  mapNodeAddress();

  auto packet_ptr = makeFromPacketData(packetData);

  // nullptr may mean corrupt packet.
  // Also an empty packetData is unexpected. Count as a detected corruption.
  if (!packet_ptr || packetData.empty()) {
    mBadPacketTracker[ns3PacketID]++;
    // all bets are off if we are corrupt. Just ignore the packet.
    return;
  }
  if (packet_ptr) {
    if (headerLooksCorrupt(packet_ptr->header())) {
      // corrupted.
      mBadPacketTracker[ns3PacketID]++;
      return;
    }
  }

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::green << "receive " << idOut(packet_ptr)
              << " TQ Size: " << mTQ.size() << " BQ Size: " << mBQ.size()
              << Ansi::reset << std::endl;

    // std::cout << "Payload: size = " << payload.size() << std::endl;
  }

  if (Experiment::getVerbosity() < Experiment::Verbosity::Error) {
    //  std::cout << Ansi::cursor_back("1000") << Ansi::clear_line() <<
    //  mSavedAddress
    //            << " @" << nowtime() << " receive " << idOut(packet_ptr)
    //           << " TQ Size: " << mTQ.size() << " BQ Size: " << mBQ.size();

    static size_t count_check = 0;
    count_check++;
    if (count_check % 100 == 0) {
      std::chrono::duration<double> duration =
          std::chrono::system_clock::now() - mTimerStart;
      int secs = duration.count();

      // if ((secs % 2) == 0) {
      const int maxUp = 20;
      auto up = mNodeIndex[mIntAddress] + 1;
      int nw = now();
      int hr = nw / (60 * 60);
      int mn = (nw - (hr * 60 * 60)) / 60;
      int sc = nw - (((hr * 60) + mn) * 60);
      std::cout << Ansi::cursor_back(1000) << Ansi::clear_line()
                << std::setfill('0') << std::setw(2) << hr << std::setw(0)
                << ":" << std::setw(2) << mn << std::setw(0) << ":"
                << std::setw(2) << sc << " run: " << secs
                << "s Sent: " << mSendTimeTracker.size();
      if (up < maxUp) {
        std::cout << Ansi::cursor_back(1000) << Ansi::cursor_up(up)
                  << Ansi::clear_line() << std::setfill(' ') << std::setw(3)
                  << up - 1 << " TQ: " << std::setw(3) << mTQ.size()
                  << " BQ: " << std::setw(3) << mBQ.size()
                  << " V: " << mVisitorList.size() << Ansi::cursor_down(up);
      }
      //}
    }
  }
  // NS_ASSERT(header);

  if (packet_ptr) {
    // auto nw = now();
    // if (nw > 300) {
    //  DebugBreak();
    //}
    // std::cout << "logging receive: " << nw << "s "
    //          << LoraMesh::toString(packet_ptr->command()) << std::endl;
    // auto nw = now();
    trackReceiveTime(packet_ptr->command());
    switch (packet_ptr->command()) {
    case LoraMesh::Command::DestinationReceipt:
    case LoraMesh::Command::NodeReceipt:
      processReceiptPacket(packet_ptr);
      break;
    case LoraMesh::Command::Measurement:
    case LoraMesh::Command::CombinedPacket:
      if (!LoraMesh::checkPayload(packet_ptr)) {
        mBadPacketTracker[ns3PacketID]++;
        return; // Can't handle bad packets other than to ignore them.
      }
      processMeasurementPacket(packet_ptr);
      break;
    default:
      break;
    }

    if (mBadPacketTracker.find(ns3PacketID) != mBadPacketTracker.end()) {
      if (mBadPacketTracker[ns3PacketID] == 0) {
        std::cout << logStamp() << "Didn't detect bad packet: " << ns3PacketID
                  << std::endl;
      }
    }
  }

  // Remember that we saw this packet.
  auto idAddress = LoraMesh::IDAddressFromPacketPtr(packet_ptr);
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "mVisitorList Insert " << idOut(packet_ptr)
              << std::endl;
  }

  // Technically We don't use this for NodeReceipts. We will collect them for
  // the simulation.
  mVisitorList.insert(idAddress);

  // If we just finished a receive, then we can try  to transmit.
  // We want to limit the data so only send receipts if we have them.
  scheduleAReceiptSend();
}

void LoraMeshNetDevice::scheduleAReceiptSend() {
  if (!mTQ.empty()) {
    auto packet = mTQ.front();
    switch (commandIDFromPacketData(packet)) {
    case SimplePacketBase::Command::DestinationReceipt:
    case SimplePacketBase::Command::NodeReceipt:
      scheduleNextRelative(mReceiptDelay);
      break;

    default:
      scheduleNextRelative(mPacketDelay);
      break;
    }
  }
}

namespace {
template <typename T>
bool isPacketIn(T &c, LoraMeshNetDevice::IDAddress idAddress) {
  auto it =
      std::find_if(c.begin(), c.end(),
                   [idAddress](const LoraMeshNetDevice::PacketData &pkt) {
                     return LoraMesh::IDAddressFromPacketData(pkt) == idAddress;
                   });
  return it != c.end();
}
} // namespace

bool LoraMeshNetDevice::isPacketInTQ(const IDAddress &idAddress) {
  return isPacketIn(mTQ, idAddress);
}

bool LoraMeshNetDevice::isPacketInBQ(const IDAddress &idAddress) {
  return isPacketIn(mBQ, idAddress);
}

bool LoraMeshNetDevice::seenBefore(const PacketData &packet) {

  // if (commandIDFromPacketData(packet) ==
  //    SimplePacketBase::Command::DestinationReceipt) {
  auto sourceIDAddress = LoraMesh::IDAddressFromPacketData(packet);
  if (mVisitorList.find(sourceIDAddress) != mVisitorList.end()) {
    // We already saw this so ignore.
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      std::cout << logStamp() << "DestinationReceipt " << idOut(packet)
                << "seen before. Ignoring." << std::endl;
    }
    return true;
  }
  //}
  return false;
}
void LoraMeshNetDevice::enqueue(const PacketData &packet) {
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::blue << "New: " << idOut(packet)
              << Ansi::reset << std::endl;
  }
  enqueueTQ(packet);
}

void LoraMeshNetDevice::enqueueTQ(const PacketData &packet, bool front) {
  // We make a copy because we want to keep these around if they are still
  // in the queue after the simulation is over so we can see what didn't
  // get cleaned up.
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "TQ enqueueing " << idOut(packet) << std::endl;
  }
  // We shouldn't be adding things to the TQ that are already in it.
  if (std::find(mTQ.begin(), mTQ.end(), packet) != mTQ.end()) {
    // std::cout << logStamp() " >>>>>>>>>>>>>>>>>>   TQ enqueing Duplicate
    // "
    //          << idOut(packet) << std::endl;
    // already in queue so nothing to do.
    return;
  }
  if (front) {
    mTQ.push_front(packet);
  } else {
    mTQ.push_back(packet);
  }
  periodicSchedule();
  mTQMax = std::max(mTQMax, mTQ.size());
}

void LoraMeshNetDevice::dumpQ(const Queue &q) {
  for (auto &pkt : q) {
    std::cout << " " << idOut(pkt);
  }
  std::cout << std::endl;
}

void LoraMeshNetDevice::enqueueBQ(const PacketData &packet) {
  // Don't want to queue packets we have seen before.
  // There may be a reason in a particularly poor network
  // To queue measurements every time they are sent. But eventually
  // this will result in packets going around that aren't going
  // to get anywhere. Better to lose a packet and fix the node
  // distributioin?
  if (seenBefore(packet)) {
    return;
  }

  // We make a copy because we want to keep these around if they are still
  // in the queue after the simulation is over so we can see what didn't
  // get cleaned up.
  // auto idAddress = LoraMesh::IDAddressFromPacketData(packet);

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::magenta << "BQ equeueing " << idOut(packet)
              << Ansi::reset << std::endl;
  }
  mBQ.push_back(packet);
  mBQMax = std::max(mBQMax, mBQ.size());
}

void LoraMeshNetDevice::removePacketFromTQ(const IDAddress &idAddress) {
  auto count = eraseFrom(mTQ, [idAddress](const PacketData &pkt) {
    return LoraMesh::IDAddressFromPacketData(pkt) == idAddress;
  });
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::boldcyan << "TQ Erased: " << count << " "
              << idAddress << Ansi::reset << std::endl;
    if (!mTQ.empty()) {
      std::cout << Ansi::boldcyan;
      dumpQ(mTQ);
      std::cout << Ansi::reset;
    }
  }
}

void LoraMeshNetDevice::removePacketFromTQ(const PacketData &packet) {
  int count = 0;
  auto iter = std::find(mTQ.begin(), mTQ.end(), packet);
  if (iter != mTQ.end()) {
    mTQ.erase(iter);
    count++;
  }
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::boldcyan << "TQ Erased: " << count << " "
              << idOut(packet) << Ansi::reset << std::endl;
  }
}

void LoraMeshNetDevice::removePacketFromBQ(const PacketData &packet) {
  int count = 0;
  removePacketFromTQ(packet);
  auto iter = std::find(mBQ.begin(), mBQ.end(), packet);
  if (iter != mBQ.end()) {
    mBQ.erase(iter);
    count++;
  }
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::boldmagenta << "BQ Erased: " << count
              << " " << idOut(packet) << Ansi::reset << std::endl;
  }
}

void LoraMeshNetDevice::removePacketFromBQ(const IDAddress &idAddress) {
  int count = 0;
  // If we got a destination receipt but not yet a node receipt
  // We need to stop trying to retransmit too.
  removePacketFromTQ(idAddress);

  // Make this a bit easier to debug.
  auto iter =
      std::find_if(mBQ.begin(), mBQ.end(), [idAddress](const PacketData &pkt) {
        return LoraMesh::IDAddressFromPacketData(pkt) == idAddress;
      });
  if (iter != mBQ.end()) {
    mBQ.erase(iter);
    count++;
  }
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::boldmagenta << "BQ Erased: " << count
              << " " << idAddress << Ansi::reset << std::endl;
  }
}

void LoraMeshNetDevice::setVisibleAddresses(
    const VisibleAddressArray &visibleAddresses) {
  mVisibleAddresses = visibleAddresses;
}

void LoraMeshNetDevice::setBroadCastMode(BroadCastMode mode) {
  mBroadCastMode = mode;
  SetChannelMode(mode);
}

void LoraMeshNetDevice::broadcastToVisible(const PacketData &packetData) {
  auto addressint = getSavedAddressInt();
  auto packet_ptr = makeFromPacketData(packetData);
  auto idAddress = LoraMesh::IDAddressFromPacketPtr(packet_ptr);
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << Ansi::yellow << "BCast: " << idOut(packetData)
              << Ansi::reset << std::endl;
  }
#if 0
  if (idOut(packetData) == "{50, 6} DestinationReceipt") {
    // DebugBreak();
    // ns3::LogComponentEnableAll(ns3::LOG_LEVEL_ALL);
    ns3::LogComponentEnable("MacLoraAca", ns3::LOG_LEVEL_ALL);
    ns3::LogComponentEnable("LoraTransducerHd", ns3::LOG_LEVEL_ALL);
    ns3::LogComponentEnable("LoraChannel", ns3::LOG_LEVEL_ALL);
    ns3::LogComponentEnable("LoraPhyGen", ns3::LOG_LEVEL_ALL);
  }
#endif
  auto phy = GetPhy();
  if (!phy->IsStateIdle()) {
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Warning) {
      std::cout << logStamp() << "is busy, can't broadcast!"
                << idOut(packetData) << std::endl;
    }
    mFailedToSendPackets[addressint].push_back(idAddress);
    // Can't broadcast. Put back at front of TQ to try again next time.
    enqueueTQ(packetData, true);
    // enqueueTQ(packetData); // place at back of TQ. Probably wrong.
  } else {
    addToBQIfNeeded(packetData);

    mSentPackets[addressint].push_back(idAddress);
    startTracking(packet_ptr);
    if (LoraMesh::isBad(idAddress)) {
      std::cout << boldred << "Bad mPacket " << reset << std::endl;
    }

    // Not tracking NodeReceipts
    if (commandIDFromPacketData(packetData) !=
        SimplePacketBase::Command::NodeReceipt) {
      auto tracked = getTracked(idAddress);
      if (tracked) {

        tracked->mForwardCount++;
        // This might be the result of forwarding after the packet already
        // reached the destination. Don't track this if so
        if (!tracked->mReceived) {
          tracked->mAddressTrack.push_back(addressint);
          // if (mPacketTracker[idAddress].mAddressTrack.size() > 10) {
          // DebugBreak();
          //  std::cout << logStamp() << Ansi::boldyellow << "Track: " <<
          //  idAddress
          //            << " AddrInt: " << addressint << Ansi::reset <<
          //            std::endl;
          //}
        }
      }
    }

    // Send will add a header to the packet (3 or 5 bytes depending on which
    // LoraAddress size we use). packet should not be used after this call
    // without removing the header. Since we just created it. Calling
    // createPacket and not storing it in a local variable helps to avoid
    // using it by accident.

    // Corrupt the packet intentionally.
    auto packetDataToSend = packetData;
    auto p = Experiment::getCorruptionProb();
    bool corrupted = false;
    if (p > 0) {
      // Do random packet corruption.
      corrupted = LoraMesh::maybeCorruptPayload(packet_ptr, p);
      PacketData corruptedPacketData;
      packet_ptr->write(corruptedPacketData);
      packetDataToSend = corruptedPacketData;
    }
    auto pkt = LoraMesh::createPacket(packetDataToSend);
    if (corrupted ||
        Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      if (corrupted) {
        std::cout << boldred;
      }
      std::cout << logStamp() << "NS3 Packet ID: " << pkt->GetUid() << " "
                << idOut(packetData) << (corrupted ? " (Corrupted)" : "");
      if (corrupted) {
        std::cout << reset;
      }
      std::cout << std::endl;
    }

    if (corrupted) {
      mBadPacketTracker[pkt->GetUid()] = 0;
    }
#if 0
    // Cycle through to next broadcase mode?
    // I don't think this works. Kind of messy as
    // coded.
    mBroadCastMode++;
    if (mBroadCastMode > 2) {
      mBroadCastMode = 0;
    }
#endif
    trackSendTime(packet_ptr->command());
    this->Send(pkt, GetBroadcast(), mBroadCastMode);
  }
}

// We need to only transmit when we know everyone else can listen.
//
// |<---TX--->|<---I--->|<---RX--->|
// |<---P--------------------------|
//
// I  - Idle
// TX - Transmitting
// RX - Receiving
// P  - Total period
// The period P is the primary variable here. Everything else
// is derived from it. TX and RX windows are 1/2 P at most.
// Broadcast transmission is done only if nothing is being
// received. Listen before transmit (LBT) is used to determine
// if TX is allowed.
void LoraMeshNetDevice::startScheduling(double firstStart,
                                        double schedulePeriod,
                                        double receiptDelay, double packetDelay,
                                        Callback checkForStop) {
  mapNodeAddress();
  mCheckForStop = checkForStop;
  mSchedulePeriod = schedulePeriod;
  mReceiptDelay = receiptDelay;
  mPacketDelay = packetDelay;
  periodicSchedule();
  // We want to try to transmit on a regular interval.
}
void LoraMeshNetDevice::periodicSchedule() {
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "Periodic Schedule ";
  }
  auto n = ns3::Simulator::Now().GetSeconds();
  // auto r = std::fmod(n + mSchedulePeriod, mSchedulePeriod);
  auto r = std::round((n + mSchedulePeriod) / mSchedulePeriod);
  // auto p = Random::uniform_real<double>(0, mSchedulePeriod / 4);
  // auto p = 0.0;
  // auto transmitStartTime = r + n + mSchedulePeriod + p;
  auto transmitStartTime = r * mSchedulePeriod;

  scheduleNextAbsolute(transmitStartTime);
}

void LoraMeshNetDevice::scheduleNextRelative(double delay) {
  scheduleNextAbsolute(now() + delay);
}

void LoraMeshNetDevice::scheduleNextAbsolute(double start) {
  // Stop scheduling if this has gone on  to long.
  if (start > Experiment::durationLimit()) {
    return;
  }
  double transmitStartTime = start - now();
  // Disregard no time progress.
  if (transmitStartTime < 1) {
    return;
  }
  // Let's not schedule anything more than a year from now.
  auto constexpr one_year = 60 * 60 * 24 * 365;
  if (transmitStartTime < one_year) {
    ns3::Simulator::Schedule(ns3::Seconds(transmitStartTime),
                             &LoraMeshNetDevice::forwardAPacketNow, this);
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      // if (eventID.GetUid() == 6951) {
      //  DebugBreak();
      //}
      std::cout << logStamp() << "forwardAPacketNow: "
                << " Scheduled for: " << transmitStartTime + now() << std::endl;
    }
  } else {
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Error) {
      std::cout << "scheduleNextAbsolute Failed: " << transmitStartTime
                << " more than 1 year in the future!" << std::endl;
    }
  }
}

void LoraMeshNetDevice::scheduleNextBQPop() {
  // Keep these very regular. We don't want to many as
  // that would tend to inflate a busy TQ
  // mNextPopTime was already scheduled. We don't want
  // to schedule another until we are close to or beyond
  // that time.
  double popPeriod = mSchedulePeriod * mBQmult;
  auto transmitStartTime = mNextPopTime + popPeriod;
  auto n = ns3::Simulator::Now().GetSeconds();
  while (transmitStartTime < n) {
    transmitStartTime += popPeriod;
  }

  // Use 1/2 of our regular schedule period for the
  // overlap.
  if (n + (mSchedulePeriod / 2) < mNextPopTime) {
    return;
  }

  // Stop scheduling if this has gone on  to long.
  if (transmitStartTime > Experiment::durationLimit()) {
    return;
  }

  if (transmitStartTime > mNextPopTime) {

    // Let's not schedule anything more than a year from now.
    auto constexpr one_year = 60 * 60 * 24 * 365;
    if (transmitStartTime < one_year) {
      ns3::Simulator::Schedule(ns3::Seconds(transmitStartTime - now()),
                               &LoraMeshNetDevice::popBQNow, this);
      mNextPopTime = transmitStartTime;
      if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
        std::cout << logStamp() << "BQ Pop Event: "
                  << " Scheduled for: " << transmitStartTime << std::endl;
      }
    } else {
      if (Experiment::getVerbosity() >= Experiment::Verbosity::Error) {
        std::cout << logStamp()
                  << "scheduleNextBQPop Failed: " << transmitStartTime
                  << " more than 1 year in the future!" << std::endl;
      }
    }
  }
}

void LoraMeshNetDevice::forwardAPacketNow() {
  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    std::cout << logStamp() << "forwardAPacketNow: TQ Size: " << mTQ.size()
              << std::endl;
  }
  if (!mTQ.empty()) {
    auto packet = mTQ.front();
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      std::cout << logStamp() << "forwarding: " << idOut(packet) << std::endl;
    }
    mTQ.pop_front();
    broadcastToVisible(packet);
  }
  if (!mBQ.empty()) {
    scheduleNextBQPop();
  }
  if (mTQ.empty()) {
    mCheckForStop();
  } else {
    periodicSchedule();
  }
}

void LoraMeshNetDevice::popBQNow() {
  if (mTQ.empty() && !mBQ.empty()) {
    auto packet = mBQ.front();
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      std::cout << logStamp() << Ansi::magenta << "Popped BQ: " << idOut(packet)
                << Ansi::reset << std::endl;
    }
    mBQ.pop_front();
    // Regardless of HOW this packet got here. We have now officially
    // seen it. We may have produced it. Add to the visitor list.
    // \TODO does this belong in enqueueTQ?
    mVisitorList.insert(LoraMesh::IDAddressFromPacketData(packet));

    enqueueTQ(packet);
    scheduleNextBQPop();
  } else {
    mCheckForStop();
  }
}

void LoraMeshNetDevice::addToBQIfNeeded(const PacketData &packet) {
  // Packets that are bound for a destination (Measurement, DestinationReceipt
  // etc.) Are placed into the BQ.
  // This acts as a short term memory of things we have seen along with a
  // long term memory of things we want to get to a destination.

  switch (commandIDFromPacketData(packet)) {
  case SimplePacket::Command::DestinationReceipt:
  case SimplePacket::Command::Measurement:
  case SimplePacket::Command::CombinedPacket:
    enqueueBQ(packet);
    scheduleNextBQPop();
    break;
  default:
    break;
  }
}

// Conditionally start tracking. Only if we havn't tracked this packet yet.
void LoraMeshNetDevice::startTracking(const SimplePacketBase::Ptr &packet_ptr) {
  // Not tracking NodeReceipts
  if (packet_ptr->command() == SimplePacketBase::Command::NodeReceipt) {
    return;
  }

  IDAddress idAddress = LoraMesh::IDAddressFromPacketPtr(packet_ptr);
  if (mPacketTracker.find(idAddress) == mPacketTracker.end()) {
    if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
      std::cout << logStamp() << "Tracking: " << idOut(packet_ptr) << std::endl;
    }

    // Don't track if idAddress is bad, probably came from corrupted packet...
    if (!LoraMesh::isBad(idAddress)) {
      if (packet_ptr) {
        auto header = packet_ptr->header();
        if (!headerLooksCorrupt(header)) {
          mPacketTracker[idAddress].mDestination = header.destination();
          mPacketTracker[idAddress].mCommand = header.command();
        }
      }

      mPacketTracker[idAddress].mPacket = idAddress;
      mPacketTracker[idAddress].mSent = true;
      mPacketTracker[idAddress].mSentAt = now();
    }
  }
  // Otherwise we are already tracking
}
