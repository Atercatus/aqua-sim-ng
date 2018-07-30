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


AquaSimSlotSF::AquaSimSlotSF(): SlotSFStatus(PASSIVE), m_NDPeriod(4.0), m_maxBurst(1),
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

  	m_maxDataTxTime = MilliSeconds(m_dataPktSize/m_bitRate);  //1600bits/10kbps

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
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	//Ptr<Packet> p = MakeND();
	//SendPkt(MakeND());
	Ptr<Packet> pkt = MakeND();
	AquaSimHeader ash;
	pkt->PeekHeader(ash);
	NS_LOG_FUNCTION("ash.GetTimeStamp" << ash.GetTimeStamp());
	NS_LOG_FUNCTION("Now" << Simulator::Now());
	Simulator::Schedule(ash.GetTimeStamp(), &AquaSimSlotSF::SendPkt, this, pkt);
	//Simulator::Schedule(Seconds(0), &AquaSimSlotSF::SendPkt, this, pkt);
  	m_slotsfNDCounter--;

  	if (m_slotsfNDCounter > 0) {
    		Simulator::Schedule(Seconds(m_rand->GetValue(0.0,m_NDPeriod)), &AquaSimSlotSF::NDTimerExpire, this);
	}
}

// slot based

void
AquaSimSlotSF::InitSlotLen() 
{
	//NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	
	SFamaHeader SFama;
	m_slotLen = m_guardTime + m_maxPropDelay.ToDouble(Time::S) + SFama.GetSize(SFamaHeader::SFAMA_CTS) / 10240.0; // 10240 is datarate(bps)
	NS_LOG_FUNCTION("m_slotLen" << m_slotLen << "m_guardTime" << m_guardTime <<"m_maxPropDelay" << m_maxPropDelay.ToDouble(Time::S) 
				<< "SFama.GetSize(FamaHeader::RTS)" << SFama.GetSize(SFamaHeader::SFAMA_CTS)/10240.0);
}

double
AquaSimSlotSF::GetTime2ComingSlot(double t)
{
	double numElapseSlot = t/m_slotLen;
	double result = (ceil(numElapseSlot) * m_slotLen) - t;
	NS_LOG_FUNCTION("numElapseSlot" << numElapseSlot << "t" << t << "m_slotLen" << m_slotLen << "result" << result << "Seconds(result)" << Seconds(result));
		
	return result;
}



void
AquaSimSlotSF::SendPkt(Ptr<Packet> pkt)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "Now" << Simulator::Now());
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
			//asHeader.SetTimeStamp(startTime);
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

/*
	if(m_device->GetTransmissionStatus()){
		PowerOn();
	}

	switch(SFStatus) {
		case PASSIVE:
			NS_LOG_WARN("SFStatus is PASSIVE !!!");
			break;
		case BACKOFF:
			NS_LOG_WARN("SFStatus is BACKOFF !!!");
			break;
		case WAIT_CTS:
			m_device->SetTransmissionStatus(SEND);
			pkt->AddHeader(asHeader);
			SendDown(pkt);
			break;
		case WAIT_DATA_FINISH:
			NS_LOG_WARN("SFStaus is WAIT_DATA_FINISH !!!");
			break;
		case WAIT_DATA:
			NS_LOG_WARN("Collision!!!!!");
			break;
		case REMOTE:
			break;

	}
*/

  	return;
}

bool
AquaSimSlotSF::TxProcess(Ptr<Packet> pkt)
{
  	//callback to higher level, should be implemented differently
  	//Scheduler::instance().schedule(&CallBack_Handler, &m_callbackEvent, CALLBACK_DELAY);

  	if( NeighborList.empty() ) {
      		pkt=0;
      		return false;
  	}

  	//figure out how to cache the packet will be sent out!!!!!!!
  	AquaSimHeader asHeader;
	VBHeader vbh;
  	FamaHeader FamaH;
	MacHeader mach;
	AquaSimPtTag ptag;
 	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
  	pkt->RemoveHeader(FamaH);
	pkt->RemovePacketTag(ptag);

	asHeader.SetSize(m_dataPktSize);
  	asHeader.SetTxTime(m_maxDataTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
  	//UpperLayerPktType = ptag.GetPacketType();

  	asHeader.SetNextHop(NeighborList[m_neighborId]);
  	m_neighborId = (m_neighborId+1)%NeighborList.size();
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);

  	vbh.SetTargetAddr(asHeader.GetNextHop());

  	FamaH.SetPType(FamaHeader::FAMA_DATA);
  	FamaH.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	FamaH.SetDA(asHeader.GetNextHop());

	pkt->AddHeader(FamaH);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	PktQ.push(pkt);

  	//fill the next hop when sending out the packet;
  	if( (PktQ.size() == 1) /*the pkt is the first one*/ && SlotSFStatus == PASSIVE ) {
      		if( CarrierDected() ) {
	 		//DoRemote(2*m_maxPropDelay+m_estimateError);
	 		DoRemote(2*Seconds(m_slotLen));
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
	NS_LOG_FUNCTION("Call: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "m_waitCTSTimer : " << m_waitCTSTimer.GetDelayLeft());


  	AquaSimHeader asHeader;
	MacHeader mach;
  	FamaHeader FamaH;
	AquaSimPtTag ptag;
  	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
  	pkt->PeekHeader(FamaH);
	pkt->PeekPacketTag(ptag);
	pkt->AddHeader(mach);
	pkt->AddHeader(asHeader);
  	AquaSimAddress dst = FamaH.GetDA();
  
  	if( m_backoffTimer.IsRunning() ) {
      		m_backoffTimer.Cancel();
      		//DoRemote(2*m_maxPropDelay+m_estimateError);
		DoRemote(2*Seconds(m_slotLen));
  	} 
	else if( m_remoteTimer.IsRunning() ) {
      		m_remoteTimer.Cancel();
      		m_remoteExpireTime = Seconds(-1);
  	}

 	 /*ND is not a part of AquaSimFama. We just want to use it to get next hop
  	 *So we do not care wether it collides with others
  	 */
  	if( ( ptag.GetPacketType() == AquaSimPtTag::PT_FAMA)&& (FamaH.GetPType()==FamaHeader::ND) ) {
      		ProcessND(FamaH.GetSA());
      		pkt=0;
      		return false;
  	}

  	if( asHeader.GetErrorFlag() )
  	{
      		//if(drop_)
		//drop_->recv(pkt,"Error/Collision");
    		//else
		pkt=0;

      		//DoRemote(2*m_maxPropDelay+m_estimateError);
		DoRemote(2*Seconds(m_slotLen));
      		return false;
  	}

  	if( ptag.GetPacketType() == AquaSimPtTag::PT_FAMA ) {
      		switch( FamaH.GetPType() ) {
			case FamaHeader::RTS:
          			if( dst == m_device->GetAddress() ) {
	      				ProcessRTS(FamaH.GetSA());
	  			}
	  			DoRemote(m_CTSTxTime+2*m_maxPropDelay+m_estimateError);
	  			break;

			case FamaHeader::CTS:
          			if(m_waitCTSTimer.IsRunning()) {
              				m_waitCTSTimer.Cancel();
            
              				if(dst == m_device->GetAddress()) {
                  				SendDataPkt();
              				}
              				else {
                  				DoBackoff();
              				} 
          			}		 

          			// this CTS must not be for this node
            			//DoRemote(2*m_maxPropDelay+m_estimateError);
				DoRemote(2*Seconds(m_slotLen));
          			break;
			default:
          			//process Data packet
	  			if( dst == m_device->GetAddress() ) {
	        			//ptag.SetPacketType(UpperLayerPktType);
                			//NS_LOG_INFO("Process Data Packet!!!!");
					SlotSFStatus = WAIT_DATA_FINISH;
      					m_device->SetTransmissionStatus(RECV);
	    				SendUp(pkt);
	    				return true;
	  			}
	  			else {
					//DoRemote(m_maxPropDelay+m_estimateError);
					DoRemote(Seconds(m_slotLen));
	  			}
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
  	NS_LOG_FUNCTION(this << pkt);

	
  	AquaSimHeader asHeader;
	pkt->PeekHeader(asHeader);
	Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	pkt->AddHeader(asHeader);
	NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp:" << asHeader.GetTimeStamp() << "startTime" << startTime);		
	Simulator::Schedule(startTime, &AquaSimSlotSF::SendPkt, this, pkt);
			
  	//SendPkt(pkt);
}


void
AquaSimSlotSF::ProcessDataBackoffTimer()
{
  	if( !PktQ.empty() )
    		DoBackoff();
  	else
    		SlotSFStatus = PASSIVE;
}


Ptr<Packet>
AquaSimSlotSF::MakeND()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
  	FamaHeader FamaH;
	AquaSimPtTag ptag;

	asHeader.SetSize(2*sizeof(AquaSimAddress)+1);
  	asHeader.SetTxTime(GetTxTime(asHeader.GetSize()));
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(AquaSimAddress::GetBroadcast());
	Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

  	FamaH.SetPType(FamaHeader::ND);
  	FamaH.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	FamaH.SetDA(AquaSimAddress::GetBroadcast());

	pkt->AddHeader(FamaH);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	
	return pkt;
}


void
AquaSimSlotSF::ProcessND(AquaSimAddress sa)
{
  	//FamaHeader FamaH;
  	//pkt->PeekHeader(FamaH);
	NeighborList.push_back(sa);
}


Ptr<Packet>
AquaSimSlotSF::MakeRTS(AquaSimAddress Recver)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
  	FamaHeader FamaH;
	AquaSimPtTag ptag;

  	asHeader.SetSize(GetSizeByTxTime(m_RTSTxTime.ToDouble(Time::S)));
  	asHeader.SetTxTime(m_RTSTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(Recver);
	Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

  	FamaH.SetPType(FamaHeader::RTS);
 	FamaH.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	FamaH.SetDA(Recver);

	pkt->AddHeader(FamaH);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	
	return pkt;
}


void
AquaSimSlotSF::SendRTS(Time DeltaTime)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	AquaSimHeader asHeader;
  	PktQ.front()->PeekHeader(asHeader);
  	SendPkt( MakeRTS(asHeader.GetNextHop()) );
  	NS_LOG_FUNCTION("After SendPkt");
  	SlotSFStatus = WAIT_CTS;
  	NS_LOG_FUNCTION("m_waitCTSTimer : " << m_waitCTSTimer.GetDelayLeft());
  	NS_LOG_FUNCTION("DeltaTIme : " << DeltaTime);
  	m_waitCTSTimer.SetFunction(&AquaSimSlotSF::DoBackoff,this);
  	m_waitCTSTimer.Schedule(DeltaTime);
}


void
AquaSimSlotSF::ProcessRTS(AquaSimAddress sa)
{
  	//FamaHeader FamaH;
  	//pkt->PeekHeader(FamaH);
  	SendPkt( MakeCTS(sa) );
  	SlotSFStatus = WAIT_DATA;
}



Ptr<Packet>
AquaSimSlotSF::MakeCTS(AquaSimAddress RTS_Sender)
{
  	NS_LOG_FUNCTION("Call : " <<  AquaSimAddress::ConvertFrom (m_device->GetAddress()) << "RTS Sender : " << RTS_Sender);

  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
	FamaHeader FamaH;
	AquaSimPtTag ptag;

  	asHeader.SetSize(GetSizeByTxTime(m_CTSTxTime.ToDouble(Time::S)));
  	asHeader.SetTxTime(m_CTSTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(RTS_Sender);
	
	Time startTime = Simulator::Now() + Seconds(GetTime2ComingSlot(Simulator::Now().ToDouble(Time::S)));
	asHeader.SetTimeStamp(startTime);
	NS_LOG_FUNCTION("Now" << Simulator::Now() << "TimeStamp: " << asHeader.GetTimeStamp() << "startTime" << startTime);

  	FamaH.SetPType(FamaHeader::CTS);
  	FamaH.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	FamaH.SetDA(RTS_Sender);

	pkt->AddHeader(FamaH);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  
	return pkt;
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

} // namespace ns2
