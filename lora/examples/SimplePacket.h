#ifndef SIMPLEPACKET_H
#define SIMPLEPACKET_H

#include "CryptHash.h"

#include "ns3/lora-address.h"

#include <array>
#include <memory>

/// Classes to define a set of simple packets
class Serialize {
public:
  using Byte = uint8_t;
  using ByteArray = std::vector<Byte>;
  using PacketData = ByteArray;
  enum Exception { ReadOutOfData };

public:
  virtual void read(const PacketData &stream, size_t &offset) = 0;
  virtual void write(PacketData &stream) const = 0;

  template <typename T> static void writeBytes(PacketData &stream, T s) {
    auto len = sizeof(T);
    const Byte *bytes = reinterpret_cast<const Byte *>(&s);
    for (decltype(len) i = 0; i < len; ++i) {
      stream.push_back(bytes[i]);
    }
  }

  template <typename T>
  static void readBytes(const PacketData &stream, size_t &offset, T &s) {
    auto len = sizeof(T);
    if (len + offset > stream.size()) {
      throw ReadOutOfData;
    }
    Byte *bytes = reinterpret_cast<Byte *>(&s);
    for (decltype(len) i = 0; i < len; ++i) {
      bytes[i] = stream[offset++];
    }
  }
};

class SimplePayloadPacket;

class SimplePacketBase : public Serialize {
public:
  using Address = ns3::Address;
  using AddressInt = ns3::LoraAddress::LoraAddressInt;
  using PacketID = uint32_t; // Save some space for the simulation
  // using PacketID = uint64_t;
  using Ptr = std::unique_ptr<SimplePacketBase>;

  /// Commands used in the LoraMesh to perform communication
  enum class Command : uint8_t {
    // These enum values are shared among different devices, if they are
    // changed the software on all effected devices needs to be updated.
    ActivateNode = 0,       ///< Activates node
    NodeActivated = 1,      ///< Sent in response to receipt of and processing a
                            ///< ActivateNode command
    Location = 2,           ///< Transmits node physical location
    Measurement = 3,        ///< Transmit a measurement
    NodeReceipt = 4,        ///< Transmit receipt by a node
    DestinationReceipt = 5, ///< Receipt sent from destination.
    CombinedPacket = 6,     ///< Packet is Measurement plus NodeReceipt.
#if 0
    Alive = 7, ///< Periodically transmitted from a node to let the network know
               ///< it is OK
    Trace = 8, ///< Mapping tool. Collects information for each node and is
               ///< passed on
    Ping = 9,  ///< Asks for a receipt from a particular node
    ReservedStart = 10,
    ReservedEnd = 127,
    UserDefinedStart = 128,
    UserDefinedEnd = 0xff
#endif
  };
  class Header {
  public:
    Header() = default;
    Header(Command command, const AddressInt &originatingNode,
           const AddressInt &lastBroadcastNode, const AddressInt &destination,
           const PacketID &id)
        : mCommand(command), mOriginatingNode(originatingNode),
          /* mLastBroadcastNode(lastBroadcastNode), */
          mDestination(destination), mPacketID(id) {}

    Command command() const { return mCommand; }
    void command(Command command) { mCommand = command; }

    const AddressInt &originatingNode() const { return mOriginatingNode; }
    void originatingNode(const AddressInt &address) {
      mOriginatingNode = address;
    }

    // const Address &lastBroadcastNode() const { return mLastBroadcastNode; }
    // void lastBroadCastNode(const Address &address) {
    //  mLastBroadcastNode = address;
    //}

    const AddressInt &destination() const { return mDestination; }
    void destination(const AddressInt &address) { mDestination = address; }

    PacketID packetID() const { return mPacketID; }
    void packetID(PacketID id) { mPacketID = id; }

    void read(const PacketData &stream, size_t &offset) {
      Serialize::readBytes(stream, offset, mCommand);
      Serialize::readBytes(stream, offset, mOriginatingNode);
      Serialize::readBytes(stream, offset, mDestination);
      Serialize::readBytes(stream, offset, mPacketID);
    }
    void write(PacketData &stream) const {
      Serialize::writeBytes(stream, mCommand);
      Serialize::writeBytes(stream, mOriginatingNode);
      Serialize::writeBytes(stream, mDestination);
      Serialize::writeBytes(stream, mPacketID);
    }

  private:
    Command mCommand;
    AddressInt mOriginatingNode; // Node that first broadcast this packet
    // Address mLastBroadcastNode; // Node that last broadcast this packet
    AddressInt mDestination; // Where the packet is being sent. Can be broadcast
                             // address.
    PacketID mPacketID;
  };

public:
  virtual Command command() const = 0;
  virtual Header header() const = 0;
  virtual bool hasPayload() const = 0;
  virtual const SimplePayloadPacket *getPayloadPtr() const = 0;
};

class SimplePacket : public SimplePacketBase {
public:
  using Byte = Serialize::Byte;
  using ByteArray = Serialize::ByteArray;
  using PacketData = Serialize::PacketData;
  using Payload = std::array<Byte, 20>;

public:
  SimplePacket() = default;
  SimplePacket(const Header &header) : mHeader(header) {}
  virtual ~SimplePacket() {}

  Header header() const override { return mHeader; }
  void header(const Header &header) { mHeader = header; }

  void read(const PacketData &stream, size_t &offset) override {
    mHeader.read(stream, offset);
  }
  void write(PacketData &stream) const override { mHeader.write(stream); }

  Command command() const override { return mHeader.command(); }

  bool hasPayload() const override { return false; }
  const SimplePayloadPacket *getPayloadPtr() const override { return nullptr; }

private:
  Header mHeader;
};

class SimpleReceiptPacket : public SimplePacket {
public:
  SimpleReceiptPacket() = default;
  SimpleReceiptPacket(const Header &header, AddressInt &nodeGettingReceipt,
                      PacketID IDOfPacketNeedingReceipt)
      : SimplePacket(header), mNodeGettingReceipt(nodeGettingReceipt),
        mIDOfPacketNeedingReceipt(IDOfPacketNeedingReceipt) {}

  const AddressInt &nodeGettingReceipt() const { return mNodeGettingReceipt; }
  void nodeGettingReceipt(const AddressInt &address) {
    mNodeGettingReceipt = address;
  }

  const PacketID &idOfPacketNeedingReceipt() const {
    return mIDOfPacketNeedingReceipt;
  }
  void idOfPacketNeedingReceipt(const PacketID &id) {
    mIDOfPacketNeedingReceipt = id;
  }

  void read(const PacketData &stream, size_t &offset) override {
    SimplePacket::read(stream, offset);
    Serialize::readBytes(stream, offset, mNodeGettingReceipt);
    Serialize::readBytes(stream, offset, mIDOfPacketNeedingReceipt);
  }

  void write(PacketData &stream) const override {
    SimplePacket::write(stream);
    Serialize::writeBytes(stream, mNodeGettingReceipt);
    Serialize::writeBytes(stream, mIDOfPacketNeedingReceipt);
  }

  bool hasPayload() const override { return false; }
  const SimplePayloadPacket *getPayloadPtr() const override { return nullptr; }

private:
  AddressInt mNodeGettingReceipt;     // Node where received packet originated.
  PacketID mIDOfPacketNeedingReceipt; // ID on Node mNodeGettingReceipt of node
                                      // that requested a receipt.
};

class SimplePayloadPacket : public SimplePacket {
public:
  SimplePayloadPacket() = default;

  const Payload &payload() const { return mPayload; }
  void payload(const Payload &payload, bool computeHash = true) {
    mPayload = payload;
    if (computeHash) {
      mHash = CryptHash::SHA256(mPayload);
    }
  }

  const CryptHash::HashSHA256 &hash() const { return mHash; }

  void read(const PacketData &stream, size_t &offset) override {
    SimplePacket::read(stream, offset);
    Serialize::readBytes(stream, offset, mPayload);
    Serialize::readBytes(stream, offset, mHash);
  }
  void write(PacketData &stream) const override {
    SimplePacket::write(stream);
    Serialize::writeBytes(stream, mPayload);
    Serialize::writeBytes(stream, mHash);
  }

  bool hasPayload() const override { return true; }
  const SimplePayloadPacket *getPayloadPtr() const override { return this; }

private:
  SimplePacket::Payload mPayload;
  CryptHash::HashSHA256 mHash;
};

class CombinedPayloadNodeReceipt : public SimplePacketBase {
public:
  CombinedPayloadNodeReceipt() = default;
  CombinedPayloadNodeReceipt(const SimplePayloadPacket &payloadPacket,
                             const SimpleReceiptPacket &receiptPacket)
      : mPayloadPacket(payloadPacket), mReceiptPacket(receiptPacket) {}

  const SimplePayloadPacket &payloadPacket() const { return mPayloadPacket; }
  void payloadPacket(const SimplePayloadPacket &payloadPacket) {
    mPayloadPacket = payloadPacket;
  }

  const SimpleReceiptPacket &receiptPacket() const { return mReceiptPacket; }
  void receiptPacket(const SimpleReceiptPacket &receiptPacket) {
    mReceiptPacket = receiptPacket;
  }

  void read(const PacketData &stream, size_t &offset) override {
    SimplePacket::Command command;
    Serialize::readBytes(stream, offset, command);
    mPayloadPacket.read(stream, offset);
    mReceiptPacket.read(stream, offset);
  }
  void write(PacketData &stream) const override {
    SimplePacket::Command command = SimplePacket::Command::CombinedPacket;
    Serialize::writeBytes(stream, command);
    mPayloadPacket.write(stream);
    mReceiptPacket.write(stream);
  }

  Command command() const override {
    return SimplePacket::Command::CombinedPacket;
  }
  // This seems slightly wrong...
  Header header() const override { return mPayloadPacket.header(); }

  bool hasPayload() const override { return true; }
  const SimplePayloadPacket *getPayloadPtr() const override {
    return &mPayloadPacket;
  }

private:
  SimplePayloadPacket mPayloadPacket;
  SimpleReceiptPacket mReceiptPacket;
};

// First thing in PacketData stream is command provide a fast accessor for that
inline SimplePacket::Command
commandIDFromPacketData(const SimplePacket::PacketData &stream) {
  SimplePacket::Command command;
  size_t offset = 0;
  Serialize::readBytes(stream, offset, command);
  return command;
}

inline SimplePacketBase::Ptr
makeFromPacketData(const SimplePacket::PacketData &stream) {
  SimplePacket::Command command = commandIDFromPacketData(stream);
  SimplePacketBase::Ptr packet;
  switch (command) {
  case SimplePacket::Command::Measurement:
    packet = SimplePacketBase::Ptr(new SimplePayloadPacket);
    break;

  case SimplePacket::Command::NodeReceipt:
  case SimplePacket::Command::DestinationReceipt:
    packet = SimplePacketBase::Ptr(new SimpleReceiptPacket);
    break;
  case SimplePacket::Command::CombinedPacket:
    packet = SimplePacketBase::Ptr(new CombinedPayloadNodeReceipt);
    break;
  default:
    break;
  }

  try {
    size_t offset = 0;
    packet->read(stream, offset);
  } catch (Serialize::Exception &e) {
    return SimplePacket::Ptr(nullptr);
  }
  return packet;
}

#endif // SIMPLEPACKET_H