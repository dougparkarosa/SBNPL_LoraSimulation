
#include "LoraMeshPacket.h"

#include "DebugBreak.h"
#include "LoraMeshNetDevice.h"
#include "RandomNumbers.h"

#include "ns3/lora-header-common.h"
#include "ns3/packet.h"

namespace LoraMesh {

SimPacketPtr createPacket(const PacketData &pay) {
  return ns3::Create<SimPacket>(&pay[0], static_cast<uint8_t>(pay.size()));
}

PacketData extractPayload(const SimPacket *packet) {
  PacketData payload;
  if (packet == nullptr) {
    return payload;
  }

  auto size = packet->GetSize();
  // std::cout << "packet Size: " << size << std::endl;

  // Empty payload
  if (size <= 0) {
    return payload;
  }

  payload.resize(size);
  packet->CopyData(&payload[0], size);

  return payload;
}

SimplePacket::Header makeHeader(Command command, const AddressInt &source,
                                const PacketID &newPacketID,
                                const AddressInt &cur_node,
                                const AddressInt &dest_node) {
  SimplePacket::Header header(command, source, cur_node, dest_node,
                              newPacketID);

  return header;
}

PacketData makeSimplePayload(const PacketID &nodePacketID,
                             const AddressInt &source,
                             const AddressInt &dest_node) {
  SimplePayloadPacket packet;
  // Fill it with random data.
  // Calculate the hash.
  SimplePacket::Payload payload;
  for (auto &byte : payload) {
    byte = Random::uniform_int<Byte>(0, 255);
  }
  packet.payload(payload);
  packet.header(makeHeader(Command::Measurement, source, nodePacketID, source,
                           dest_node));

  PacketData pay;
  packet.write(pay);

  return pay;
}

/// Corrupts the packet with random noise, random truncation etc.
///
#define USE_LIMITED_CORRUPTION 1
void corruptPayload(SimplePacketBase::Ptr &packet_ptr) {
#if USE_LIMITED_CORRUPTION
  auto packet = dynamic_cast<SimplePayloadPacket *>(packet_ptr.get());
  if (packet) {
    // Corrupt only the payload.
    auto payload = packet->payload();
    auto num_bytes = payload.size();
    auto num_to_corrupt = Random::uniform_int<int>(0, num_bytes / 2);
    for (auto i = 0; i < num_to_corrupt; ++i) {
      auto corrupt_index = Random::uniform_int<size_t>(0, num_bytes);
      payload[corrupt_index] = Random::uniform_int<Byte>(0, 255);
    }
    packet->payload(payload, false);
  }
#else
  auto num_bytes = packetData.size();
  auto num_to_corrupt = Random::uniform_int<int>(0, num_bytes / 2);
  for (auto i = 0; i < num_to_corrupt; ++i) {
    auto corrupt_index = Random::uniform_int<size_t>(0, num_bytes);
    packetData[corrupt_index] = Random::uniform_int<Byte>(0, 255);
  }

  // Flip a coin to see if we lose some bytes.
  auto lose_bytes = Random::uniform_int<int>(0, 10) > 5;
  if (lose_bytes) {
    auto num_to_lose = Random::uniform_int<PacketData::size_type>(0, num_bytes);
    packetData.resize(num_bytes - num_to_lose);
  }
#endif
}

/// Randomly corrupt payload with a uniform probablity p
///
/// \param packetData - PacketData to possibly corrupt.
/// \param p - Probability to corrupt. 0 to 1.
///
/// \return true if corruption was attempted.
bool maybeCorruptPayload(SimplePacketBase::Ptr &packet_ptr, double p) {
  auto corrupt_it = Random::uniform_real<double>(0.0, 1.0);
  if (corrupt_it < p) {
    // Only corrupting SimplePayloadPacket.
    if (packet_ptr->command() == LoraMesh::Command::Measurement) {
      corruptPayload(packet_ptr);
      return !checkPayload(packet_ptr); // Don't assume corruption worked.
    }
  }
  return false;
}

/// Validates hash, size etc.
///
/// \return true if payload looks good false otherwise.
bool checkPayload(const SimplePacketBase::Ptr &packet_ptr) {
  if (packet_ptr->command() == LoraMesh::Command::Measurement) {

    auto pay_packet = dynamic_cast<SimplePayloadPacket *>(packet_ptr.get());
    auto check_hash = CryptHash::SHA256(pay_packet->payload());
    bool ok = check_hash == pay_packet->hash();
    // if (!ok) {
    //   std::cout << check_hash << std::endl;
    //   std::cout << packet->mHash << std::endl;
    // }
    return ok;
  }
  return true;
}

SimpleReceiptPacket
makeSimpleReceiptPacket(const SimplePacket &packetNeedingReceipt,
                        const AddressInt &receivedAt,
                        const PacketID &newPacketID) {
  // std::cout << "making receipt: " << receivedFrom << ", " << receivedAt <<
  // ",
  // "
  //          << receivedPacketID << std::endl;
  SimpleReceiptPacket receipt;
  receipt.header(makeHeader(Command::NodeReceipt, receivedAt, newPacketID,
                            receivedAt,
                            packetNeedingReceipt.header().originatingNode()));
  receipt.nodeGettingReceipt(packetNeedingReceipt.header().originatingNode());
  receipt.idOfPacketNeedingReceipt(packetNeedingReceipt.header().packetID());
  return receipt;
}

SimpleReceiptPacket
makeSimpleNodeReceiptPacket(const SimplePacket &packetNeedingReceipt,
                            const AddressInt &receivedAt,
                            const PacketID &newPacketID) {
  // std::cout << "makeSimpleNodeReceiptPacket" << std::endl;
  auto receipt =
      makeSimpleReceiptPacket(packetNeedingReceipt, receivedAt, newPacketID);
  auto header = receipt.header();
  header.command(Command::NodeReceipt);
  receipt.header(header);
  return receipt;
}

SimpleReceiptPacket
makeSimpleDestinationReceiptPacket(const SimplePacket &packetNeedingReceipt,
                                   const AddressInt &receivedAt,
                                   const PacketID &newPacketID) {
  auto receipt = makeSimpleNodeReceiptPacket(packetNeedingReceipt, receivedAt,
                                             newPacketID);
  auto header = receipt.header();
  header.command(Command::DestinationReceipt);
  receipt.header(header);

  return receipt;
}

SimPacketPtr makeSimpleNodeReceipt(const SimplePacket &sourcePacket,
                                   const AddressInt &receivedAt,
                                   const PacketID &receivedPacketID) {
  auto pay = toPacketData(
      makeSimpleNodeReceiptPacket(sourcePacket, receivedAt, receivedPacketID));

  return createPacket(pay);
}

PacketData
makeSimpleDestinationReceipt(const SimplePacket &packetNeedingReceipt,
                             const AddressInt &receivedAt,
                             const PacketID &newPacketID) {
  auto pay = toPacketData(makeSimpleDestinationReceiptPacket(
      packetNeedingReceipt, receivedAt, newPacketID));
  return pay;
}

SimPacketPtr makeCombinedPacket(const SimplePayloadPacket &payload,
                                const SimpleReceiptPacket &nodeReceipt) {
  CombinedPayloadNodeReceipt packet(payload, nodeReceipt);
  auto pay = toPacketData(packet);
  return createPacket(pay);
}

PacketID badID() { return std::numeric_limits<PacketID>::max(); }
IDAddress makeIDAddress(const AddressInt &address, const PacketID &packetID) {
  return std::make_tuple(address, packetID);
}

IDAddress trackedIDAddress(const SimpleReceiptPacket &receiptPacket) {
  return makeIDAddress(receiptPacket.nodeGettingReceipt(),
                       receiptPacket.idOfPacketNeedingReceipt());
}

ns3::LoraAddress::LoraAddressInt getIntAddress(const Address &address) {
  auto addr = ns3::LoraAddress::ConvertFrom(address);
  return addr.GetAsInt();
}

IDAddress IDAddressFromPacketData(const PacketData &payload) {
  auto packet = makeFromPacketData(payload);
  return IDAddressFromPacketPtr(packet);
}

IDAddress IDAddressFromPacketPtr(const SimplePacketBase::Ptr &packet_ptr) {
  return makeIDAddress(packet_ptr->header().originatingNode(),
                       packet_ptr->header().packetID());
}

std::string toString(Command cmd) {
  switch (cmd) {
  case Command::ActivateNode:
    return "ActivateNode";
  case Command::NodeActivated:
    return "NodeActivated";
  case Command::Location:
    return "Location";
  case Command::Measurement:
    return "DataPacket";
  case Command::NodeReceipt:
    return "NodeReceipt";
  case Command::DestinationReceipt:
    return "EndReceipt";
  case Command::CombinedPacket:
    return "Combined";
#if 0
  case Command::Alive:
    return "Alive";
  case Command::Trace:
    return "Trace";
  case Command::Ping:
    return "Ping";
#endif
  default:
    return "Other";
  }
}

std::string commandFromPacketData(const PacketData &payload) {
  return toString(commandIDFromPacketData(payload));
}
}; // namespace LoraMesh

std::ostream &operator<<(std::ostream &os,
                         const LoraMesh::IDAddress &idAddress) {

  auto intaddr = std::get<0>(idAddress);
  return os << LoraMeshNetDevice::lookUpNodeIndex(intaddr) << "_"
            << std::get<1>(idAddress);
}