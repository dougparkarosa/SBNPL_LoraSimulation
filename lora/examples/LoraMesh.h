#ifndef LORA_MESH_H
#define LORA_MESH_H

#include "Array.h"
#include "SystemID.h"

class LoraMesh {
public:
  /// Commands used in the LoraMesh to perform communication
  enum class Command : uint8_t {
    // These enum values are shared among different devices, if they are
    // changed the software on all effected devices needs to be updated.
    ActivateNode = 0,  ///< Activates node
    NodeActivated = 1, ///< Sent in response to receipt of and processing a
                       ///< ActivateNode command
    Location = 2,      ///< Transmits node physical location
    Measurement = 3,   ///< Transmit a measurement
    NodeReceipt = 4,   ///< Transmit receipt by a node
    ServerReceipt = 5, ///< Transmit receipt by the server
    Alive = 6, ///< Periodically transmitted from a node to let the network know
               ///< it is OK
    Trace = 7, ///< Mapping tool. Collects information for each node and is
               ///< passed on
    Ping = 8,  ///< Asks for a receipt from a particular node
    ReservedStart = 9,
    ReservedEnd = 127,
    UserDefinedStart = 128,
    UserDefinedEnd = 0xff
  };

  /// Type used to define measurement types.
  enum class DataType : uint8_t {
    Single = 0, ///<
    TimeSeries = 1,
    ReservedStart = 2,
    ReservedEnd = 127,
    UserDefinedStart = 128,
    UserDefinedEnd = 0xff
  };

  // Define a packet with a fixed length. Payload size if computed at compile
  // time so that the total size of the packet including payload and overhead
  // adds up to the total packet length
  template <typename HashType, uint8_t PacketLength = 4096> class Packet {
    using IDType = System::IDType;
    const static size_t OverheadLength =
        sizeof(Command) + sizeof(uint16_t) + sizeof(IDType) + sizeof(HashType);
    const static size_t PayloadLength = PacketLength - OverheadLength;
    using PayloadType = Array<uint8_t, PayloadLength>;
    const static size_t Size = PacketLength;

  public:
    Packet();

  private:
    Command mCommand;
    IDType mNodeID;
    uint16_t mPacketID;
    PayloadType mPayload;
    HashType mHash;
  };

public:
  LoraMesh();

  void begin();

private:
  System::IDType mId;
};

#endif // LORA_MESH_H
