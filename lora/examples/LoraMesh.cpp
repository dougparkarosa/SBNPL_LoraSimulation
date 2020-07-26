#include "LoraMesh.h"

// High level ideas
//
// Each device has a unique ID (NodeID), MAC address should do.
// Each device will generate a public/private key pair based on ID and
// random information.
//
// Each node will work more or less like a class A LoraWan. There will be a
// transmit burst of Data with routing information. The data will be a
// (Command, NodeID, TransmitID, Payload) tuple. NodeID is as noted above.
// TransmitID is simply a Node specific counter that is incremented by 1
// each time a new ID is required. It will wrap around back to zero on
// overflow. A 16 bit value would give 64K commands before wraparound which
// should be sufficient to avoid collisions in a reasonably well functioning
// network.
//
// Commands are broadcast by a node. Any node that receives a command will
// put the command into a broadcast queue if the command did not come from
// itself. When a receipt is received that the command was accepted the
// command is removed from the queue on that node.
//
// There are conceptually 2 kinds of receipts. A) another node received the
// data and now has it in its broadcast queue. B) the server received the
// data. A server receipt means the transmission of this particular data is
// done.
//
// Each node can have 2 queues. An active broadcast queue and a server
// broadcast queue. The server broadcast queue keeps data in the queue until
// the server sends a receipt. At that point there is no practical reason to
// keep the data and it can be removed from the queue. The active broadcast
// queue keeps sending the queued data until a receipt is received, either
// server or another node. Presumably the node receipt will be first, but
// there could possibly be a scenario where the receipt from the node is
// missed.
//
// Security is maintained using a symmetric key that is stored on the
// device. THIS IS WEAK! A better system is to use a private/public key
// system.
//
// Activation is done using a symmetric key that is unique for the system.
// This protects the public key exchange. We don't want unauthorized devices
// joining the network.

// A measurement is sent repeatedly until a receipt comes back from the
// server or the send timeout expires. Data can be cached on the node as
// long as there is space for it. A fixed amount of space is allocated. The
// oldest measurement is discarded if there is no more storage available
// when a new measurement is taken. When a receipt is received for data it
// is taken out of the transmit queue.

// Forwarding of

// Commands:
enum Commands {
  Location,    // Data contains GPS information
  Measurement, // Data contains the type, timestamp and value of a
               // measurement
  Receipt,     // Sent from server when data has been successfully recorded.
               // Contains source node ID
  // timestamp and data type of data recorded. This is routed back to the
  // node to tell it that the data made it home.
  Alive,     // No data, just a periodic ping to let the neighbors know the node
             // is alive
  PublicKey, // Data contains the public key of the device
  RequestKey,  // Request that node send the public key.
  ActivateNode // Begins key activation. Uses
};

//
// __BYTE_ORDER__