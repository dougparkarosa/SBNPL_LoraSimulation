// Packet for testing LoraMesh
#ifndef LORAMESHPACKET_H
#define LORAMESHPACKET_H

#include "ns3/lora-address.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"

#include "CryptHash.h"
#include "Experiment.h"
#include "SimplePacket.h"

#include <array>
#include <tuple>

namespace LoraMesh {
using SimPacket = ns3::Packet;
using SimPacketPtr = ns3::Ptr<SimPacket>;
using ConstSimPacketPtr = ns3::Ptr<const SimPacket>;
using Address = SimplePacket::Address;
using AddressInt = SimplePacket::AddressInt;
using Byte = SimplePacket::Byte;
using ByteArray = SimplePacket::ByteArray;
using PacketData = SimplePacket::ByteArray;
using PacketID = SimplePacket::PacketID;
using SecondsType = Experiment::SecondsType;
using IDAddress = Experiment::IDAddress;
using Command = SimplePacket::Command;

std::string toString(Command cmd);

std::string commandFromPacketData(const PacketData &packet);

constexpr SecondsType kSecsMax = std::numeric_limits<SecondsType>::max();
struct PacketStatus {
  PacketStatus()
      : mPacket(), mDestination(), mSent(false), mReceived(false),
        mDestReceiptAtSender(false), mSentAt(kSecsMax), mReceivedAt(kSecsMax),
        mReceiptReceivedAt(kSecsMax), mReceiptCount(0), mDestCount(0),
        mForwardCount(0), mAddressTrack() {}
  IDAddress mPacket;              // Packet identity that was sent
  AddressInt mDestination;        // Packet destination address.
  Command mCommand;               // Packet command
  bool mSent;                     // True if sent
  bool mReceived;                 // True if packet recived at destination
  bool mDestReceiptAtSender;      // True is sender got a destination receipt
  SecondsType mSentAt;            // Time packet was sent in seconds
  SecondsType mReceivedAt;        // Time packet was received at destination
  SecondsType mReceiptReceivedAt; // Time sender got the receipt
  uint32_t
      mReceiptCount;   // Number of times packet receipt was received at sender.
  uint32_t mDestCount; // Number of times packet was seen at destination
  uint32_t mForwardCount; // Number of times packet was forwarded (includes
                          // initial send)
  std::vector<AddressInt> mAddressTrack; // Addresses visited.
};

PacketData extractPayload(const SimPacket *packet);
#if 0
template <typename T> const T *castPayloadTo(const PacketData &payload) {
  // std::cout << "castPayloadTo payload.size(): " << payload.size()
  //         << " sizeof(T): " << sizeof(T) << std::endl;
  if (payload.size() >= sizeof(T)) {
    const T *p = reinterpret_cast<const T *>(&payload[0]);
    return p;
  } else {
    return nullptr;
  }
}
#else
template <typename T> const T convertPacketDataTo(const PacketData &payload) {
  T res;
  size_t offset = 0;
  res.read(payload, offset);
  return res;
}
#endif

#if 0
template <typename T> void appendinto(PacketData &v, const T &val) {
  constexpr auto s = sizeof(T);
  // std::cout << "appendinto size: " << s << std::endl;
  // Cast so we can treat val as an array of bytes.
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&val);
  for (std::size_t i = 0; i < s; ++i) {
    v.push_back(p[i]);
  }
  // std::cout << "appendinto v.size(): " << v.size() << std::endl;
  return;
}

template <typename T> PacketData toPacketData(const T &val) {
  PacketData result;
  appendinto(result, val);
  // std::cout << "stuffinto result.size()" << result.size() << std::endl;
  return result;
}
#endif

template <typename T> PacketData toPacketData(const T &val) {
  PacketData result;
  val.write(result);
  return result;
}

PacketID badID();

inline bool isBad(const Address &address) {
  return (address.GetLength() >= Address::MAX_SIZE);
}

inline bool isBad(const IDAddress &idAddress) {
  return (
      (std::get<0>(idAddress) > std::numeric_limits<AddressInt>::max() / 2) ||
      std::get<1>(idAddress) == badID());
}

IDAddress IDAddressFromPacketData(const PacketData &payload);
IDAddress IDAddressFromPacketPtr(const SimplePacketBase::Ptr &packet_ptr);

IDAddress makeIDAddress(const AddressInt &address, const PacketID &packet);

template <typename T> IDAddress originIDAddress(const T &packet) {
  return makeIDAddress(packet.header().originatingNode(),
                       packet.header().packetID());
}

IDAddress trackedIDAddress(const SimpleReceiptPacket &receiptPacket);

ns3::LoraAddress::LoraAddressInt getIntAddress(const Address &address);

PacketData makeSimplePayload(const PacketID &nodePacketID,
                             const AddressInt &source, const AddressInt &dest);

bool checkPayload(const SimplePacketBase::Ptr &packet_ptr);
void corruptPayload(SimplePacketBase::Ptr &packet_ptr);
bool maybeCorruptPayload(SimplePacketBase::Ptr &packet_ptr, double p);

SimpleReceiptPacket
makeSimpleNodeReceiptPacket(const SimplePacket &sourcePacket,
                            const AddressInt &receivedAt,
                            const PacketID &newPacketID);

SimpleReceiptPacket
makeSimpleDestinationReceiptPacket(const SimplePacket &sourcePacket,
                                   const AddressInt &receivedAt,
                                   const PacketID &newPacketID);

SimPacketPtr makeSimpleNodeReceipt(const SimplePacket &sourcePacket,
                                   const AddressInt &receivedAt,
                                   const PacketID &receivedPacketID);
PacketData makeSimpleDestinationReceipt(const SimplePacket &sourcePacket,
                                        const AddressInt &receivedAt,
                                        const PacketID &newPacketID);

SimPacketPtr createPacket(const PacketData &pay);

SimPacketPtr makeCombinedPacket(const SimplePayloadPacket &payload,
                                const SimpleReceiptPacket &nodeReceipt);

// Packet makeRebroadcastPacket(const Packet &packet, );
// Packet makeNodeReceiptPacket(const);
// Packet makeDestinationREceiptPacket(const);

}; // namespace LoraMesh

std::ostream &operator<<(std::ostream &os,
                         const LoraMesh::IDAddress &idAddress);

#endif // LORAMESHPACKET_H