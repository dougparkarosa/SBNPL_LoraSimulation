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

#include "lora-transducer-hd.h"
#include "ns3/simulator.h"
#include "ns3/lora-prop-model.h"
#include "lora-phy.h"
#include "lora-channel.h"
#include "ns3/log.h"
#include "ns3/pointer.h"


namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("LoraTransducerHd");

NS_OBJECT_ENSURE_REGISTERED (LoraTransducerHd);
  
LoraTransducerHd::LoraTransducerHd ()
  : LoraTransducer (),
    m_state (RX),
    m_endTxTime (Seconds (0)),
    m_cleared (false)
{
}

LoraTransducerHd::~LoraTransducerHd ()
{
}

void
LoraTransducerHd::Clear ()
{
  if (m_cleared)
    {
      return;
    }
  m_cleared = true;
  if (m_channel)
    {
      m_channel->Clear ();
      m_channel = 0;
    }

  LoraPhyList::iterator it = m_phyList.begin ();
  for (; it != m_phyList.end (); it++)
    {
      if (*it)
        {
          (*it)->Clear ();
          *it = 0;
        }
    }
  ArrivalList::iterator ait = m_arrivalList.begin ();
  for (; ait != m_arrivalList.end (); ait++)
    {
      ait->GetPacket () = 0;
    }
  m_phyList.clear ();
  m_arrivalList.clear ();
  m_endTxEvent.Cancel ();
}

void
LoraTransducerHd::DoDispose ()
{
  Clear ();
  LoraTransducer::DoDispose ();
}
TypeId
LoraTransducerHd::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::LoraTransducerHd")
    .SetParent<Object> ()
    .SetGroupName ("Lora")
    .AddConstructor<LoraTransducerHd> ()
  ;
  return tid;
}

LoraTransducer::State
LoraTransducerHd::GetState () const
{
  return m_state;
}


bool
LoraTransducerHd::IsRx (void) const
{
  return m_state == RX;
}

bool
LoraTransducerHd::IsTx (void) const
{
  return m_state == TX;

}

const LoraTransducer::ArrivalList &
LoraTransducerHd::GetArrivalList (void) const
{
  return m_arrivalList;
}

void
LoraTransducerHd::Receive (Ptr<Packet> packet,
                          double rxPowerDb,
                          LoraTxMode txMode,
                          LoraPdp pdp)
{
  LoraPacketArrival arrival (packet,
                            rxPowerDb,
                            txMode,
                            pdp,
                            Simulator::Now ());

  m_arrivalList.push_back (arrival);
  Time txDelay = Seconds (packet->GetSize () * 8.0 / txMode.GetDataRateBps ());
  Simulator::Schedule (txDelay, &LoraTransducerHd::RemoveArrival, this, arrival);
  NS_LOG_DEBUG (Simulator::Now ().GetSeconds () << " Transducer in receive: packet = " << packet->GetUid());
  if (m_state == RX)
    {
      NS_LOG_DEBUG ("Transducer state = RX");
      LoraPhyList::const_iterator it = m_phyList.begin ();
      for (; it != m_phyList.end (); it++)
        {
          NS_LOG_DEBUG ("Calling StartRx: Packet=" << packet->GetUid());
          (*it)->StartRxPacket (packet, rxPowerDb, txMode, pdp);
        }
    }
}

void
LoraTransducerHd::Transmit (Ptr<LoraPhy> src,
                           Ptr<Packet> packet,
                           double txPowerDb,
                           LoraTxMode txMode)
{
  if (m_state == TX)
    {
      Simulator::Remove (m_endTxEvent);
      src->NotifyTxDrop(packet);           // traced source netanim
    }
  else
    {
      m_state = TX;
      src->NotifyTxBegin(packet);             // traced source netanim
    }


  Time delay = Seconds (packet->GetSize () * 8.0 / txMode.GetDataRateBps ());
  NS_LOG_DEBUG ("Transducer transmitting: "<< packet->GetUid() << " TX delay = "
                << delay.GetSeconds() << " seconds for packet size "
                << packet->GetSize () << " bytes and rate = "
                << txMode.GetDataRateBps () << " bps");
  LoraPhyList::const_iterator it = m_phyList.begin ();
  for (; it != m_phyList.end (); it++)
    {
      if (src != (*it))
        {
          (*it)->NotifyTransStartTx (packet, txPowerDb, txMode);
        }
    }
  m_channel->TxPacket (Ptr<LoraTransducer> (this), packet, txPowerDb, txMode);


  delay = std::max (delay, m_endTxTime - Simulator::Now ());

  m_endTxEvent = Simulator::Schedule (delay, &LoraTransducerHd::EndTx, this);
  m_endTxTime = Simulator::Now () + delay;
  Simulator::Schedule(delay, &LoraPhy::NotifyTxEnd, src, packet);    // traced source netanim
}

void
LoraTransducerHd::EndTx (void)
{
  NS_ASSERT (m_state == TX);
  m_state = RX;
  m_endTxTime = Seconds (0);
}
void
LoraTransducerHd::SetChannel (Ptr<LoraChannel> chan)
{
  NS_LOG_DEBUG ("Transducer setting channel");
  m_channel = chan;

}
Ptr<LoraChannel>
LoraTransducerHd::GetChannel (void) const
{
  return m_channel;
}
void
LoraTransducerHd::AddPhy (Ptr<LoraPhy> phy)
{
  m_phyList.push_back (phy);
}

const LoraTransducer::LoraPhyList &
LoraTransducerHd::GetPhyList (void) const
{
  return m_phyList;
}

void
LoraTransducerHd::RemoveArrival (LoraPacketArrival arrival)
{

  // Remove entry from arrival list
  ArrivalList::iterator it = m_arrivalList.begin ();
  for (; it != m_arrivalList.end (); it++)
    {
      if (it->GetPacket () == arrival.GetPacket ())
        {
          m_arrivalList.erase (it);
          break;
        }
    }
  LoraPhyList::const_iterator ait = m_phyList.begin ();
  for (; ait != m_phyList.end (); ait++)
    {
      (*ait)->NotifyIntChange ();
    }

}

} // namespace ns3
