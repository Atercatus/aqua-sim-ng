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

#include "aqua-sim-mac-slotsf.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-mac.h"
#include "aqua-sim-pt-tag.h"
#include "aqua-sim-address.h"
#include "aqua-sim-header-routing.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/nstime.h"
#include "ns3/packet.h"
#include "ns3/integer.h"
#include <cmath>
namespace ns3 {

NS_LOG_COMPONENT_DEFINE("AquaSimSlotSF");
NS_OBJECT_ENSURE_REGISTERED(AquaSimSlotSF);


AquaSimSlotSF::AquaSimSlotSF(): SlotSFStatus(PASSIVE), m_NDPeriod(4.0), m_maxBurst(3),
		m_dataPktInterval(Seconds(0.0001)), m_estimateError(Seconds(0.0001)),m_dataPktSize(1600),
		m_neighborId(0), m_waitCTSTimer(Timer::CHECK_ON_DESTROY),//(Timer::CANCEL_ON_DESTROY),
		m_backoffTimer(Timer::CANCEL_ON_DESTROY), m_remoteTimer(Timer::CANCEL_ON_DESTROY),
		m_remoteExpireTime(-1), m_guardTime(0.0003), m_slotsfNDCounter(2)
		//, backoff_timer(this), status_handler(this), NDTimer(this),
		//WaitCTSTimer(this),DataBackoffTimer(this),RemoteTimer(this), CallBack_Handler(this)
{
	m_transmitDistance=3000.0;
  	m_maxPropDelay = Seconds(m_transmitDistance/1500.0);
  	m_RTSTxTime = m_maxPropDelay;
  	m_CTSTxTime = m_RTSTxTime + 2*m_maxPropDelay;
	m_dataRate = 10000.0;
	m_RTSCPTime = 2*m_maxPropDelay + Seconds(0.0030);
  	m_maxDataTxTime = MilliSeconds(m_dataPktSize/m_bitRate);  //1600bits/10kbps
	m_rctSlotTime = 0.0; 
	InitSlotLen();

  	m_rand = CreateObject<UniformRandomVariable> ();
  	Simulator::Schedule(Seconds(m_rand->GetValue(0.0,m_NDPeriod)+0.000001), &AquaSimSlotSF::NDTimerExpire, this);
}

AquaSimSlotSF::~AquaSimSlotSF()
{
}

TypeId
AquaSimSlotSF::GetTypeId(void)
{
  	static TypeId tid = TypeId("ns3::AquaSimSlotSF")
      				.SetParent<AquaSimMac>()
      				.AddConstructor<AquaSimSlotSF>()
      				.AddAttribute("MaxBurst", "The maximum number of packet burst. default is 1",
						IntegerValue(1),
						MakeIntegerAccessor (&AquaSimSlotSF::m_maxBurst),
						MakeIntegerChecker<int>());
  	return tid;
}

int64_t
AquaSimSlotSF::AssignStreams (int64_t stream)
{
  	NS_LOG_FUNCTION (this << stream);
  	m_rand->SetStream(stream);
  	return 1;
}

void
AquaSimSlotSF::NDTimerExpire()
{
	Ptr<Packet> pkt = MakeND();
	AquaSimHeader ash;
	pkt->PeekHeader(ash);
	Simulator::Schedule(ash.GetTimeStamp() - Simulator::Now(), &AquaSimSlotSF::SendPkt, this, pkt);
	//Simulator::Schedule(Seconds(0), &AquaSimSlotSF::SendPkt, this, pkt);
  	m_slotsfNDCounter--;

  	if (m_slotsfNDCounter > 0) {
    		Simulator::Schedule(Seconds(m_rand->GetValue(0.0,m_NDPeriod)), &AquaSimSlotSF::NDTimerExpire, this);
	
	}

  	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "ash.GetTimeStamp" << ash.GetTimeStamp() << "Now" << Simulator::Now());
}

// slot based

void
AquaSimSlotSF::InitSlotLen() 
{
	//NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	
	SlotSFHeader ssfh;
	m_slotLen = m_guardTime + m_maxPropDelay.ToDouble(Time::S) + 50 / m_dataRate; // 10000 is datarate(bps)
	NS_LOG_FUNCTION("m_slotLen" << m_slotLen << "m_guardTime" << m_guardTime <<"m_maxPropDelay" << m_maxPropDelay.ToDouble(Time::S) 
				<< "ssfh.GetSize()" << 50/10240.0);
}

double
AquaSimSlotSF::GetTime2ComingSlot(double t)
{
	double numElapseSlot = t/m_slotLen;
	//double result = (ceil(numElapseSlot) * m_slotLen) - t;
	double result = (ceil(numElapseSlot) * m_slotLen);
/*	
	if(m_rtcSlotTime == Seconds(0.0)) {
		m_rtcSlotTime = result;
		//return result;	
	}
*/	
	
	while(result < m_rctSlotTime + m_slotLen/5)
	{
		result += m_slotLen;
	}

/*	
	if((m_rctSlotTime - m_slotLen/3) <= result && result <= (m_rctSlotTime + m_slotLen/3)) {
		result += m_slotLen;
	}
*/
	m_rctSlotTime = result;
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress())  << "numElapseSlot" << numElapseSlot << "t" << t << "m_slotLen" << m_slotLen << "result" << result << "Seconds(result)" << Seconds(result));
	return result;
}



void
AquaSimSlotSF::SendPkt(Ptr<Packet> pkt)
{
  	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now());
  	AquaSimHeader asHeader;
  	pkt->RemoveHeader(asHeader);

  	asHeader.SetDirection(AquaSimHeader::DOWN);
	

  	Time txtime = asHeader.GetTxTime();
  	Time startTime;	
	switch( m_device->GetTransmissionStatus() ) {
    		case SLEEP:
      			PowerOn();
    		case NIDLE:
      			//for slot based
      			//startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
			//asHeader.etTimeStamp(startTime);
      			pkt->AddHeader(asHeader);
			//NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp" << asHeader.GetTimeStamp() << "startTime" << startTime);
      			//Simulator::Schedule(startTime, &AquaSimMac::SendDown, this, pkt);
			SendDown(pkt);
      			break;
    		case RECV:
      			NS_LOG_WARN("RECV-SEND Collision!!!!!");
      			pkt=0;
      			break;
    		default:
      			//status is SEND
     		 	pkt=0;
  }
  	return;
}

bool
AquaSimSlotSF::TxProcess(Ptr<Packet> pkt)
{
  	//callback to higher level, should be implemented differently
  	//Scheduler::instance().schedule(&CallBack_Handler, &m_callbackEvent, CALLBACK_DELAY);

  	//NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	if( NeighborList.empty() ) {
      		pkt=0;
      		return false;
  	}

  	//figure out how to cache the packet will be sent out!!!!!!!
  	AquaSimHeader asHeader;
	VBHeader vbh;
  	SlotSFHeader ssfh;
	MacHeader mach;
	AquaSimPtTag ptag;
 	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
  	pkt->RemoveHeader(ssfh);
	pkt->RemovePacketTag(ptag);

	asHeader.SetSize(m_dataPktSize);
  	asHeader.SetTxTime(m_maxDataTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
  	//UpperLayerPktType = ptag.GetPacketType();

  	asHeader.SetNextHop(NeighborList[m_neighborId]);
  	m_neighborId = (m_neighborId+1)%NeighborList.size();
	//m_neighborId = m_rand->GetValue(0, NeighborList.size());
/*
	while(AquaSimAddress::ConvertFrom(NeighborList[m_neighborId]) != 10)
	{
		m_neighborId = (m_neighborId+1) % NeighborList.size();	
	}
	//m_neighborId = 30;
*/
  	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "m_neighborId" << m_neighborId);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);

  	vbh.SetTargetAddr(asHeader.GetNextHop());

  	ssfh.SetPType(SlotSFHeader::DATA);
  	ssfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	ssfh.SetDA(asHeader.GetNextHop());

	pkt->AddHeader(ssfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	PktQ.push(pkt);

  	//fill the next hop when sending out the packet;
  	if( (PktQ.size() == 1) /*the pkt is the first one*/ && SlotSFStatus == PASSIVE ) {
      		if( CarrierDected() ) {
	 		//DoRemote(2*m_maxPropDelay+m_estimateError);
	 		DoRemote(2*Seconds(m_slotLen));;
      		}
      		else{
	 	//	SendRTS(2*m_maxPropDelay+m_CTSTxTime+m_RTSTxTime+m_estimateError);
      			SendRTS(4*Seconds(m_slotLen));
		}
  	}	
  	return true;
}


bool
AquaSimSlotSF::RecvProcess(Ptr<Packet> pkt)
{
  	AquaSimHeader asHeader;
	MacHeader mach;
  	SlotSFHeader ssfh;
	AquaSimPtTag ptag;
  	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
  	pkt->PeekHeader(ssfh);
	//ssfh.SetRecvProcessTime(Simulator::Now());
	pkt->PeekPacketTag(ptag);
	pkt->AddHeader(mach);
	pkt->AddHeader(asHeader);
  	AquaSimAddress dst = ssfh.GetDA();
 
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "SlotSFHeader" << ssfh << "asHeader" << asHeader << "Time Stamp" << asHeader.GetTimeStamp() 
			<< "Now" << Simulator::Now());
	 
	// collision
	if(isReceived == true) {
		NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "collision!!!!!!!!!!!!!" << "Now" << Simulator::Now());
		AfterRecvProcess(ssfh);
		return false;
	}

	isReceived = true; // recv mode

  	if( m_backoffTimer.IsRunning() ) {
      		m_backoffTimer.Cancel();
      		//DoRemote(2*m_maxPropDelay+m_estimateError);
		DoRemote(2*Seconds(m_slotLen));
  	} 
	else if( m_remoteTimer.IsRunning() ) {
      		m_remoteTimer.Cancel();
      		m_remoteExpireTime = Seconds(-1);
  	}
/*
  	if( ( ptag.GetPacketType() == AquaSimPtTag::PT_FAMA)&& (ssfh.GetPType()==SlotSFHeader::ND) ) {
      		ProcessND(ssfh.GetSA());
      		pkt=0;
      		return false;
  	}
*/
  	if( asHeader.GetErrorFlag() )
  	{
		pkt=0;

		DoRemote(2*Seconds(m_slotLen));
      		return false;
  	}

  	if( ptag.GetPacketType() == AquaSimPtTag::PT_FAMA ) {
		AfterRecvProcess(ssfh);		

      		switch( ssfh.GetPType() ) {
			case SlotSFHeader::ND:
				ProcessND(pkt);
				pkt = 0;
				return false;
				//break;
			case SlotSFHeader::RELAY_CTS:
				ProcessRELAY_CTS(ssfh);
				break;
			case SlotSFHeader::ND_ACK:
				//ProcessND_ACK(pkt);
				break;
			case SlotSFHeader::RTS:
				ProcessRTS(pkt);
				break;
			case SlotSFHeader::CTS:
				ProcessCTS(ssfh);
				break;
			default:
				//process Data packet
				if(ProcessDATA(pkt))
					return true;

      		}
  	}

  	pkt=0;
  	return true;
}

void
AquaSimSlotSF::SendDataPkt()
{
  	NS_LOG_FUNCTION("Call: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()));

  	int PktQ_Size = PktQ.size();
  	int SentPkt = 0;
	//Time StartTime = Simulator::Now();

  	AquaSimHeader asHeader;
  	PktQ.front()->PeekHeader(asHeader);
  	AquaSimAddress recver = asHeader.GetNextHop();
  	Ptr<Packet> tmp;
  	

  	for(int i=0; i<PktQ_Size && SentPkt<m_maxBurst; i++) {
    		tmp = PktQ.front();
    		tmp->PeekHeader(asHeader);
    		PktQ.pop();
    		if( asHeader.GetNextHop() == recver ) {
			SentPkt++;
		 	//Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
			//asHeader.SetTimeStamp(startTime);
			//NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp:" << asHeader.GetTimeStamp() << "startTime" << startTime);		
			//Simulator::Schedule(startTime, &AquaSimSlotSF::ProcessDataSendTimer, this, tmp);
			//tmp->AddHeader();
			Simulator::Schedule(Seconds(0), &AquaSimSlotSF::ProcessDataSendTimer, this, tmp);
        		//PktQ.front()->PeekHeader(asHeader);

/*
			if( !PktQ.empty() ) {
	  			StartTime += asHeader.GetTxTime() + m_dataPktInterval;
			}
			else {
	  			break;
			}
<*/		
			if(PktQ.empty()){
				break;
			}
    		}
    		else{
			PktQ.push(tmp);
    		}
  	}	

  	//SlotSFStatus = WAIT_DATA_FINISH;

  	//Simulator::Schedule(m_maxPropDelay+StartTime-Simulator::Now(),&AquaSimSlotSF::ProcessDataBackoffTimer,this);
  	Simulator::Schedule(m_maxPropDelay,&AquaSimSlotSF::ProcessDataBackoffTimer,this);
	
	//Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	//asHeader.SetTimeStamp(startTime);
	//NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp" << asHeader.GetTimeStamp() << "startTime" << startTime);
	//Simulator::Schedule(startTime, &AquaSimSlotSF::ProcessDataBackoffTimer, this);	
}

void
AquaSimSlotSF::ProcessDataSendTimer(Ptr<Packet> pkt)
{
  	AquaSimHeader asHeader;
	pkt->PeekHeader(asHeader);
	Time startTime = Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	pkt->AddHeader(asHeader);
	NS_LOG_FUNCTION("Call:" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now() << "TimeStamp:" << asHeader.GetTimeStamp() << "startTime" << startTime);		

	Simulator::Schedule(startTime - Simulator::Now(), &AquaSimSlotSF::SendPkt, this, pkt);
			
  	//SendPkt(pkt);
}


void
AquaSimSlotSF::ProcessDataBackoffTimer()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	if( !PktQ.empty() )
    		DoBackoff();
  	else
    		SlotSFStatus = PASSIVE;
}


Ptr<Packet>
AquaSimSlotSF::MakeND()
{
  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
  	SlotSFHeader ssfh;
	AquaSimPtTag ptag;

	asHeader.SetSize(2*sizeof(AquaSimAddress)+1);
  	asHeader.SetTxTime(GetTxTime(asHeader.GetSize()));
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(AquaSimAddress::GetBroadcast());
	Time startTime = Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

  	ssfh.SetPType(SlotSFHeader::ND);
  	ssfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	ssfh.SetDA(AquaSimAddress::GetBroadcast());
	//ssfh.SetSendTime(Seconds(0));
	//ssfh.SetReceiveTime(Seconds(0));
	//ssfh.SetReplyTime(Seconds(0));
	//ssfh.SetRecvProcessTime(Seconds(0));

	pkt->AddHeader(ssfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	
	return pkt;
}


void
AquaSimSlotSF::ProcessND(Ptr<Packet> pkt)
{
	/*
	AquaSimHeader asHeader;
	//MacHeader mach;
	//SlotSFHeader ssfh;
	Ptr<Packet> nd_ack = MakeND_ACK(pkt);	

	nd_ack->PeekHeader(asHeader);
	Simulator::Schedule(asHeader.GetTimeStamp() - Simulator::Now(), &AquaSimSlotSF::SendPkt, this, pkt);	
	SlotSFStatus = WAIT_ND_ACK;
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "delay" << asHeader.GetTimeStamp() - Simulator::Now());
	*/
	// timer set(re-send)
	//
	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
	pkt->PeekHeader(ssfh);
	NeighborList.push_back(ssfh.GetSA());	
}

Ptr<Packet>
AquaSimSlotSF::MakeND_ACK(Ptr<Packet> pkt)
{
	Ptr<Packet> nd_ack = Create<Packet>();

  	AquaSimHeader ack_asHeader;
	MacHeader ack_mach;
  	SlotSFHeader ack_ssfh;
	AquaSimPtTag ack_ptag;

	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	pkt->RemoveHeader(asHeader);  	
	pkt->RemoveHeader(mach);
	pkt->PeekHeader(ssfh);
	//pkt->AddHeader(mach);
	//pkt->AddHeader(asHeader);

  	ack_asHeader.SetSize(GetSizeByTxTime(m_RTSTxTime.ToDouble(Time::S)));
  	ack_asHeader.SetTxTime(m_RTSTxTime);
  	ack_asHeader.SetErrorFlag(false);
  	ack_asHeader.SetDirection(AquaSimHeader::DOWN);
	ack_ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	ack_asHeader.SetNextHop(ssfh.GetSA());
	Time startTime = Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	ack_asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now() << "TimeStamp: " << ack_asHeader.GetTimeStamp() << "startTime" << startTime);

  	ack_ssfh.SetPType(SlotSFHeader::ND_ACK);
 	ack_ssfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	ack_ssfh.SetDA(ssfh.GetSA());
	//ack_ssfh.SetSendTime(ssfh.GetSendTime());
	//ack_ssfh.SetReceiveTime(ssfh.GetRecvProcessTime());
	//ack_ssfh.SetReplyTime(startTime);
	//ack_ssfh.SetRecvProcessTime(Seconds(0));
	
	nd_ack->AddHeader(ack_ssfh);
	nd_ack->AddHeader(ack_mach);
  	nd_ack->AddHeader(ack_asHeader);
	nd_ack->AddPacketTag(ack_ptag);
  	
	return nd_ack;
}

void
AquaSimSlotSF::ProcessND_ACK(Ptr<Packet> pkt)
{
	AquaSimHeader asHeader;
	//MacHeader mach;
	SlotSFHeader ssfh;
	//pkt->RemoveHeader(asHeader);  	
	//pkt->RemoveHeader(mach);
	pkt->PeekHeader(ssfh);

	//Time propDelay = ((ssfh.GetRecvProcessTime() - ssfh.GetSendTime()) - (ssfh.GetReplyTime() - ssfh.GetReceiveTime())) / 2;
	//int quotient = ssfh.GetRecvProcessTime().ToDouble(Time::S) / m_slotLen;
	//double remainder = ssfh.GetRecvProcessTime().ToDouble(Time::S) - ((double)quotient * m_slotLen);
	//Time delay = propDelay - Seconds(remainder); 
	//NS_LOG_FUNCTION("RecvProcessTime" << ssfh.GetRecvProcessTime() << "ssfh.GetSendTime" << ssfh.GetSendTime() << "GetReplyTime" << ssfh.GetReplyTime() << "GetReceiveTime" << ssfh.GetReceiveTime()
	//		<< "propDelay" << propDelay << "delay" << delay);
	
	NeighborList.push_back(ssfh.GetSA());
	//DelayList.push_back(delay);
	//NeighborList.push_back(ssfh.GetSA());
}

Ptr<Packet>
AquaSimSlotSF::MakeRTS(AquaSimAddress Recver)
{
  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
  	SlotSFHeader ssfh;
	AquaSimPtTag ptag;

  	asHeader.SetSize(GetSizeByTxTime(m_RTSTxTime.ToDouble(Time::S)));
  	asHeader.SetTxTime(m_RTSTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(Recver);
	Time startTime = Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

  	ssfh.SetPType(SlotSFHeader::RTS);
 	ssfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	ssfh.SetDA(Recver);
	//ssfh.SetSendTime(Seconds(0));
	//ssfh.SetReceiveTime(Seconds(0));
	//ssfh.SetReplyTime(Seconds(0));
	//ssfh.SetRecvProcessTime(Seconds(0));

	pkt->AddHeader(ssfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	
	return pkt;
}


void
AquaSimSlotSF::SendRTS(Time DeltaTime)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	if(!m_waitCTSTimer.IsRunning()) {
		AquaSimHeader asHeader;
	  	PktQ.front()->PeekHeader(asHeader);
		
		Ptr<Packet> pkt = MakeRTS(asHeader.GetNextHop());
		pkt->PeekHeader(asHeader);
		Simulator::Schedule(asHeader.GetTimeStamp() - Simulator::Now(), &AquaSimSlotSF::SendPkt, this, pkt);	
		
	  	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "delay" << asHeader.GetTimeStamp() - Simulator::Now());
	  	
		SlotSFStatus = WAIT_CTS;
		m_waitCTSTimer.SetFunction(&AquaSimSlotSF::DoBackoff,this);
  		m_waitCTSTimer.Schedule(DeltaTime);

	}
//Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
}


void
AquaSimSlotSF::ProcessRTS(Ptr<Packet> pkt)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	pkt->RemoveHeader(asHeader);  	
	pkt->RemoveHeader(mach);
	pkt->PeekHeader(ssfh);
	pkt->AddHeader(mach);
	pkt->AddHeader(asHeader);

	if( ssfh.GetDA() == m_device->GetAddress() )
	{
		if( m_rtsQ.size() == 0 )
		{
			Ptr<Packet> pkt = MakeRELAY_CTS();
			pkt->PeekHeader(asHeader);
			Simulator::Schedule(asHeader.GetTimeStamp() - Simulator::Now(), &AquaSimSlotSF::SendPkt, this, pkt);	

			//SendPkt(MakeRELAY_CTS(ssfh.GetSA()));
			SlotSFStatus = WAIT_DATA;
			Simulator::Schedule(m_RTSCPTime, &AquaSimSlotSF::SendCTS, this);
		}	
		
		m_rtsQ.push(pkt);
	}  	

	DoRemote(5*m_maxPropDelay+m_estimateError+m_RTSCPTime);
}

void
AquaSimSlotSF::SendCTS()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	Time minTime = Seconds(999999);
	Time curTime = Seconds(0);
	Ptr<Packet> pkt;
	Ptr<Packet> earlyPkt;
	int rtsQSize = m_rtsQ.size();
	
	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	
	for(int i = 0; i < rtsQSize; i++)
	{
		pkt = m_rtsQ.front();
		pkt->RemoveHeader(asHeader);
		curTime = asHeader.GetTimeStamp();
		
		//test
		pkt->RemoveHeader(mach);
		pkt->RemoveHeader(ssfh);
		NS_LOG_FUNCTION("ssfh" << ssfh << "curTimr" << curTime << "asHeader" << asHeader);
		pkt->AddHeader(ssfh);
		pkt->AddHeader(mach);
		if(curTime < minTime){
			minTime = curTime;
			earlyPkt = pkt;
		}
		m_rtsQ.pop();
	} 
	earlyPkt->RemoveHeader(mach);
	earlyPkt->RemoveHeader(ssfh);
	
	//SendPkt(MakeCTS(ssfh.GetSA()));

	Ptr<Packet> CTS_pkt = MakeCTS(ssfh.GetSA());
	CTS_pkt->PeekHeader(asHeader);
	Simulator::Schedule(asHeader.GetTimeStamp() - Simulator::Now(), &AquaSimSlotSF::SendPkt, this, CTS_pkt);	
}

Ptr<Packet>
AquaSimSlotSF::MakeRELAY_CTS()
{
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()));

	Ptr<Packet> pkt = Create<Packet>();
	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	AquaSimPtTag ptag;

	asHeader.SetSize(GetSizeByTxTime(m_CTSTxTime.ToDouble(Time::S)));
	asHeader.SetTxTime(m_CTSTxTime);
	asHeader.SetErrorFlag(false);
	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
	asHeader.SetNextHop(AquaSimAddress::GetBroadcast());

	Time startTime = Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

	ssfh.SetPType(SlotSFHeader::RELAY_CTS);
	ssfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	ssfh.SetDA(AquaSimAddress::GetBroadcast());
       	//ssfh.SetSendTime(Seconds(0));
       	//ssfh.SetReceiveTime(Seconds(0));
       	//ssfh.SetReplyTime(Seconds(0));
       	//ssfh.SetRecvProcessTime(Seconds(0));


	pkt->AddHeader(ssfh);
	pkt->AddHeader(mach);
	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);

	return pkt;
}

Ptr<Packet>
AquaSimSlotSF::MakeCTS(AquaSimAddress RTS_Sender)
{
  	NS_LOG_FUNCTION("Call : " <<  AquaSimAddress::ConvertFrom (m_device->GetAddress()) << "RTS Sender : " << RTS_Sender);

  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	AquaSimPtTag ptag;

  	asHeader.SetSize(GetSizeByTxTime(m_CTSTxTime.ToDouble(Time::S)));
  	asHeader.SetTxTime(m_CTSTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(RTS_Sender);
	
	Time startTime = Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

  	ssfh.SetPType(SlotSFHeader::CTS);
  	ssfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	ssfh.SetDA(RTS_Sender);
	//ssfh.SetSendTime(Seconds(0));
       	//ssfh.SetReceiveTime(Seconds(0));
       	//ssfh.SetReplyTime(Seconds(0));
       	//ssfh.SetRecvProcessTime(Seconds(0));

	pkt->AddHeader(ssfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  
	return pkt;
}

void
AquaSimSlotSF::ProcessRELAY_CTS(SlotSFHeader ssfh)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	if(ssfh.GetDA() != m_device->GetAddress()) {
		DoRemote(m_maxPropDelay + m_estimateError + m_RTSCPTime);
	}
}

void
AquaSimSlotSF::ProcessCTS(SlotSFHeader ssfh)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	if(m_waitCTSTimer.IsRunning()) {
		m_waitCTSTimer.Cancel();

		if(ssfh.GetDA() == m_device->GetAddress()) {
			SendDataPkt();
		}
		else {
			DoBackoff();
		}
	}

	DoRemote(2*Seconds(m_slotLen));
}

bool
AquaSimSlotSF::ProcessDATA(Ptr<Packet> pkt)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	AquaSimHeader asHeader;
	MacHeader mach;
	SlotSFHeader ssfh;
	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
	pkt->PeekHeader(ssfh);
	pkt->AddHeader(mach);
	pkt->AddHeader(asHeader);

	if( ssfh.GetDA() == m_device->GetAddress() ) {
		SlotSFStatus = WAIT_DATA_FINISH;
		SendUp(pkt);
		return true;
	}
	else {
		DoRemote(Seconds(m_slotLen));
		return false;
	}
}

bool
AquaSimSlotSF::CarrierDected()
{
  	if( m_device->GetTransmissionStatus() == RECV || m_device->GetTransmissionStatus() == SEND )  {
	  	return true;
  	}
	
	return false;
}

void
AquaSimSlotSF::DoBackoff()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	Time backoffTime = MilliSeconds(m_rand->GetValue(0.0,10 * m_RTSTxTime.ToDouble(Time::MS)));
 
  	SlotSFStatus = BACKOFF;
  	if( m_backoffTimer.IsRunning() ) {
      		m_backoffTimer.Cancel();
  	}

  	//m_backoffTimer.SetDelay(backoffTime);
  	m_backoffTimer.SetFunction(&AquaSimSlotSF::BackoffTimerExpire,this);
  	m_backoffTimer.Schedule(backoffTime);
}


void
AquaSimSlotSF::DoRemote(Time DeltaTime)
{
  	SlotSFStatus = REMOTE;
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

  	if( Simulator::Now()+DeltaTime > m_remoteExpireTime ) {
      		m_remoteExpireTime = Simulator::Now()+DeltaTime;
      		
		if( m_remoteTimer.IsRunning() ) {
	  		m_remoteTimer.Cancel();
      		}
      		//m_remoteTimer.SetDelay(DeltaTime);
      		m_remoteTimer.SetFunction(&AquaSimSlotSF::ProcessRemoteTimer,this);
      		m_remoteTimer.Schedule(DeltaTime);
      		NS_LOG_FUNCTION("m_remoteTimer.GetDelay() : " << m_remoteTimer.GetDelayLeft());
  	}
}


void
AquaSimSlotSF::ProcessRemoteTimer()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	if( PktQ.empty() ) {
    		SlotSFStatus = PASSIVE;
  	}
  	else {
    		DoBackoff();
    		//SendRTS(2*m_maxPropDelay+m_CTSTxTime+m_RTSTxTime+m_estimateError);
  	}
}

void
AquaSimSlotSF::BackoffTimerExpire()
{
  	//SendRTS(2*m_maxPropDelay + m_RTSTxTime + m_CTSTxTime +m_estimateError);
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	SendRTS(4*Seconds(m_slotLen));
}

void AquaSimSlotSF::DoDispose()
{
	m_rand=0;
	while(!PktQ.empty()) {
		PktQ.front()=0;
		PktQ.pop();
	}
	AquaSimMac::DoDispose();
}

void
AquaSimSlotSF::EndRecv()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	isReceived = false;
}

void
AquaSimSlotSF::AfterRecvProcess(SlotSFHeader ssfh)
{
	Time delay = Seconds(ssfh.GetSize()/m_dataRate);
	Simulator::Schedule(delay, &AquaSimSlotSF::EndRecv, this);
	NS_LOG_FUNCTION("Call" << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now() << "Delay" << delay);
}

} // namespace ns3
