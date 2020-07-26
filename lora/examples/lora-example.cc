/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * This is an example script for lora protocol.
 *
 * Authors: Pavel Boyko <boyko@iitp.ru>
 *          To Thanh Hai <tthhai@gmail.com>
 *
 */

#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/lora-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include "ns3/callback.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"
#include "ns3/lora-channel.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy-gen.h"
#include "ns3/lora-prop-model-ideal.h"
#include "ns3/lora-transducer-hd.h"
#include "ns3/mac-lora-gw.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/object-factory.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include "LoraMeshNetDevice.h"
#include "LoraMeshPacket.h"

#include "Experiment.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <vector>

using namespace ns3;

/**
 * \brief Test script.
 *
 * This script 10 lora end-devices, one gateway. Lora end-devices send packets
 * on three channels.
 *
 */
class LoraExample {
public:
  LoraExample();
  /**
   * \brief Configure script parameters
   * \param argc is the command line argument count
   * \param argv is the command line arguments
   * \return true on successful configuration
   */
  bool Configure(int argc, char **argv);
  /// Run simulation
  void Run();

private:
  using BroadCastMode = LoraMeshNetDevice::BroadCastMode;
  using PacketID = LoraMeshNetDevice::PacketID;
  using SimPacket = LoraMeshNetDevice::SimPacket;
  using SimPacketPtr = LoraMeshNetDevice::SimPacketPtr;
  using ConstSimPacketPtr = LoraMeshNetDevice::ConstSimPacketPtr;
  using PacketCount = uint64_t;
  using Address = LoraMeshNetDevice::Address;
  using DevicePtr = Ptr<LoraMeshNetDevice>;
  using Nodes = std::vector<DevicePtr>;
  using AddressMap = LoraMeshNetDevice::AddressMap;
  using SentPacketMap = LoraMeshNetDevice::SentPacketMap;
  using PacketIDSet = LoraMeshNetDevice::IDAddressSet;

  std::vector<std::unique_ptr<Experiment::Base>> mExperiments;

private:
  bool DoExamples();
};

int main(int argc, char **argv) {
  LoraExample test;
  if (!test.Configure(argc, argv))
    NS_FATAL_ERROR("Configuration failed. Aborted.");

  test.Run();
  return 0;
}

//-----------------------------------------------------------------------------
LoraExample::LoraExample() {}

bool LoraExample::Configure(int argc, char **argv) {
  std::cout << "Configure: ";
  for (int i = 0; i < argc; ++i) {
    std::cout << argv[i];
  }
  std::cout << std::endl;
  Experiment::factory(mExperiments, argc, argv);
  return true;
}

void LoraExample::Run() { DoExamples(); }

bool LoraExample::DoExamples() {
  Experiment::fs::path dir;
  for (auto &experiment : mExperiments) {
    std::cout << "Running Experiments" << std::endl;
    auto numRuns = experiment->numRuns();
    for (auto r = 0; r < numRuns; ++r) {
      experiment->run(r);
      if (Experiment::getVerbosity() >= Experiment::Verbosity::Report) {
        experiment->report(std::cout, r);
      }
      experiment->exportResults(r);
      dir = experiment->getOutputDir();
    }
  }

  std::cout << "Simulation Done: Execute the following to generate graphs."
            << std::endl;
  std::stringstream s;
  s << "cd " << dir.string() << ";/usr/bin/python3 visualize.py";
  std::cout << s.str() << std::endl;

  // if (int result = std::system(s.str().c_str())) {
  // std::cout << "Done: " << result << std::endl;
  //}

  if (Experiment::getVerbosity() >= Experiment::Verbosity::Info) {
    // Packets need to be made into a virtual class instead of what we
    // have now:
    // Let's see how big each packet is.
    std::cout << "Packet Sizes" << std::endl;
    std::cout << "SimplePacketHeader: " << sizeof(SimplePacket) << std::endl;
    std::cout << "SimpleReceiptPacket: " << sizeof(SimpleReceiptPacket)
              << std::endl;
    std::cout << "SimplePayloadPacket: " << sizeof(SimplePayloadPacket)
              << std::endl;
    std::cout << "CombinedPayloadNodeReceipt: "
              << sizeof(CombinedPayloadNodeReceipt) << std::endl;
  }
  return false;
}
