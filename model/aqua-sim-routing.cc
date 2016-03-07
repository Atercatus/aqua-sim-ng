/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Connecticut
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
 * Author: Robert Martin <robert.martin@engr.uconn.edu>
 */

#include "ns3/log.h"
#include "ns3/attribute.h"
#include "ns3/simulator.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/pointer.h"

#include "aqua-sim-header.h"
#include "aqua-sim-routing.h"
//#include "aqua-sim-mac.h"

//Aqua Sim Routing

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("AquaSimRouting");
NS_OBJECT_ENSURE_REGISTERED(AquaSimRouting);


TypeId
AquaSimRouting::GetTypeId(void)
{
  static TypeId tid = TypeId("ns3::AquaSimRouting")
    .SetParent<Object>()
    .AddConstructor<AquaSimRouting> ()
    .AddAttribute("SetNetDevice", "The net device where this routing layer resides on",
      PointerValue (),
      MakePointerAccessor (&AquaSimRouting::m_device),
      MakePointerChecker<AquaSimRouting> ())
    //set my_addr, on-node lookup, add-ll, tracetarget, port-dmux
  ;
  return tid;
}

AquaSimRouting::AquaSimRouting()
{
  NS_LOG_FUNCTION(this);
  //m_tracetarget=NULL;		//to be implemented
  //ll(NULL), port_dmux(NULL)
}

AquaSimRouting::~AquaSimRouting()
{
  NS_LOG_FUNCTION(this);
}

void
AquaSimRouting::SetNetDevice(Ptr<AquaSimNetDevice> device)
{
  NS_LOG_FUNCTION(this);
  m_device = device;
}

void
AquaSimRouting::SetMac(Ptr<AquaSimMac> mac)
{
  NS_LOG_FUNCTION(this);
  m_mac = mac;
}

bool
AquaSimRouting::Recv(Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this << p << " : Currently not implemented");
  /*
   * TODO this should be implemented in inherited routing protocols
   * 		This may be redundant compared with sendup/senddown.
   */

if (IsDeadLoop(p))
  {
    NS_LOG_WARN("Packet(" << p << ") is dead loop on device(" << m_device << ")");
    return false;
  }
if (~AmINextHop(p))
  {
    NS_LOG_WARN("Device(" << m_device <<
		") is not the next hop, dropping packet(" << p << ")");
    return false;
  }
if (AmIDst(p))
  {
    NS_LOG_INFO("Destination(" << m_device << ") received packet(" << p << ")");
    return true;
  }

return SendUp(p);
}


/**
  * send packet p to the upper layer, i.e., port dmux
  *
  * @param p   a packet
  * */
bool
AquaSimRouting::SendUp(Ptr<Packet> p)
{
  //port_dmux->recv(p); // (Handler*)NULL
  NS_LOG_FUNCTION(this << p << " : currently a dummy sendup");
  /*TODO this needs to be fully implemented with the multiplexer
		  Or at least sent up for further processing
		  ie. Sync, Localization, Application driven
  */
  return true;
}

/**
  * send packet p to the lower layer
  *
  * @param p			a packet
  * @param next_hop	the next hop to route packet p
  * @param delay		packet p will be sent in time of delay
  * */
bool
AquaSimRouting::SendDown(Ptr<Packet> p, const Address &nextHop, Time delay)
{
  //cmh->uw_flag() = true;
  //cmh->addr_type() = NS_AF_INET;

  NS_LOG_FUNCTION(this << p << nextHop << delay);
  NS_ASSERT(p != NULL);

  AquaSimAddress next = AquaSimAddress::ConvertFrom(nextHop);
  //add header to packet
  AquaSimHeader header;
  p->RemoveHeader(header);

  //NS_LOG_DEBUG("Pktsize=" << header.GetSize());
  if(header.GetUId() == -1) header.SetUId(1);
  header.SetDirection(AquaSimHeader::DOWN);
  header.SetNextHop(next);
  p->AddHeader(header);

  //trace here...

  //send down after given delay
  NS_LOG_FUNCTION(this << " Currently a dummy send down. delay=" << delay << " p=" << p);

  NS_LOG_INFO("RoutingSendDown Dump: direction:" << header.GetDirection() <<
		", nexthop: " << header.GetNextHop() <<
		", pktsize: " << header.GetSize() <<
		", nosie: " << header.GetNoise() <<
		", freq: " << header.GetFreq() <<
		", modname: " << header.GetModName() <<
		", SAddr: " << header.GetSAddr() <<
		", txrange: " << header.GetTxRange() <<
		", txtime: " << header.GetTxTime() <<
		", device: " << m_device);

  /*Note this schedule will not work, should instead call internal function once
   * event is executed which will internal call Mac's function directly.
   * This should most likely be a callback.
  */
  //Simulator::Schedule(delay, &AquaSimMac::Recv, &p);

  Simulator::Schedule(delay, &AquaSimRouting::SendPacket, this, p);
  return true;
}

void
AquaSimRouting::SendPacket(Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this);
  if (!m_mac->Recv(p))
    NS_LOG_DEBUG(this << "Mac recv error");
}

/**
  * check if packet p is received because of a dead loop
  *
  * @param p		a packet
  * @return		true for p experienced a dead loop, false for not
  * */

bool
AquaSimRouting::IsDeadLoop(Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this);
  AquaSimHeader asHeader;
  p->PeekHeader(asHeader);
  NS_LOG_DEBUG ("SAddr=" << asHeader.GetSAddr());
  return (asHeader.GetSAddr()==m_myAddr) && (asHeader.GetNumForwards() > 0);
}

//TODO look into incorporating m_device->GetAddress() instead of within header here.

/**
  * check if this node is the source of packet p, i.e. p is generated by this node
  *
  * @param p		a packet
  * @return		true for yes, false for no
  * */
bool
AquaSimRouting::AmISrc(const Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this);
  AquaSimHeader asHeader;
  p->PeekHeader(asHeader);
  NS_LOG_DEBUG ("SAddr=" << asHeader.GetSAddr());
  return (asHeader.GetSAddr()==m_myAddr) && (asHeader.GetNumForwards() == 0);
}

/**
  * check if this node is the destination of packet p, i.e. p is destined to this node
  *
  * @param p		a packet
  * @return		true for yes, false for no
  * */
bool
AquaSimRouting::AmIDst(const Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this);
  AquaSimHeader asHeader;
  p->PeekHeader(asHeader);
  NS_LOG_DEBUG ("Direction=" << asHeader.GetDirection());
  return ((asHeader.GetDirection()==AquaSimHeader::UP) && (asHeader.GetDAddr() == m_myAddr));
}

/**
  * check if this node is the next hop of packetr p,
  * i.e., this node needs to forward p later on.
  *
  * @param p		a packet
  * @return		true for yes, false for no
  * */
bool
AquaSimRouting::AmINextHop(const Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this);
  AquaSimHeader asHeader;
  p->PeekHeader(asHeader);
  NS_LOG_DEBUG ("NextHop=" << asHeader.GetNextHop());
  return ((asHeader.GetNextHop() == m_myAddr)|| ( asHeader.GetNextHop() == AquaSimAddress::GetBroadcast() ));
}


}  //namespace ns3
