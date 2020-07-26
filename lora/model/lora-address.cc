/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
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
 * Author: Leonard Tracy <lentracy@gmail.com>
 *         To Thanh Hai <tthhai@gmail.com>
 */

#include "lora-address.h"
#include "ns3/address.h"

namespace ns3 {

LoraAddress::LoraAddress() : m_address(kBroadcastAddress) {}

LoraAddress::LoraAddress(LoraAddressInt addr) : m_address(addr) {}

LoraAddress::~LoraAddress() {}

uint8_t LoraAddress::GetType(void) {
  static uint8_t type = Address::Register();
  return type;
}

Address LoraAddress::ConvertTo(void) const {
  return Address(GetType(), reinterpret_cast<const uint8_t *>(&m_address),
                 kAddressSize);
}

LoraAddress LoraAddress::ConvertFrom(const Address &address) {
  NS_ASSERT(IsMatchingType(address));
  LoraAddress uAddr;
  address.CopyTo(reinterpret_cast<uint8_t *>(&uAddr.m_address));
  return uAddr;
}

LoraAddress::LoraAddressInt LoraAddress::GetAsInt(void) const {
  return m_address;
}
bool LoraAddress::IsMatchingType(const Address &address) {
  return address.CheckCompatible(GetType(), kAddressSize);
}

LoraAddress::operator Address() const { return ConvertTo(); }

void LoraAddress::CopyFrom(const uint8_t *pBuffer) { 
  m_address = *reinterpret_cast<const LoraAddressInt*>(pBuffer); 
}

void LoraAddress::CopyTo(uint8_t *pBuffer) { 
  *reinterpret_cast<LoraAddressInt*>(pBuffer) = m_address;
}

LoraAddress LoraAddress::GetBroadcast() {
  return LoraAddress(kBroadcastAddress);
}
LoraAddress LoraAddress::Allocate() {
  static LoraAddressInt nextAllocated = 0;
  LoraAddressInt address = nextAllocated++;
  if (nextAllocated == kBroadcastAddress) {
    nextAllocated = 0;
  }

  return LoraAddress(address);
}

bool operator<(const LoraAddress &a, const LoraAddress &b) {
  return a.m_address < b.m_address;
}

bool operator==(const LoraAddress &a, const LoraAddress &b) {
  return a.m_address == b.m_address;
}

bool operator!=(const LoraAddress &a, const LoraAddress &b) {
  return !(a == b);
}

std::ostream &operator<<(std::ostream &os, const LoraAddress &address) {
  os << (int)address.m_address;
  return os;
}
std::istream &operator>>(std::istream &is, LoraAddress &address) {
  LoraAddress::LoraAddressInt x;
  is >> x;
  NS_ASSERT(0 <= x);
  NS_ASSERT(x <= LoraAddress::kBroadcastAddress);
  address.m_address = x;
  return is;
}

} // namespace ns3
