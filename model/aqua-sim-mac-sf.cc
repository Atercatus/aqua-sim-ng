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

#include "aqua-sim-mac-sf.h"
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

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("AquaSimSF");
NS_OBJECT_ENSURE_REGISTERED(AquaSimSF);


AquaSimSF::AquaSimSF(): SFStatus(PASSIVE), m_NDPeriod(4.0), m_maxBurst(1),
		m_dataPktInterval(0.0001), m_estimateError(0.001),m_dataPktSize(1600),
		m_neighborId(0), m_waitCTSTimer(Timer::CHECK_ON_DESTROY),//(Timer::CANCEL_ON_DESTROY),
		m_backoffTimer(Timer::CANCEL_ON_DESTROY), m_remoteTimer(Timer::CANCEL_ON_DESTROY),
		m_remoteExpireTime(-1), m_sfNDCounter(2)
		//, backoff_timer(this), status_handler(this), NDTimer(this),
		//WaitCTSTimer(this),DataBackoffTimer(this),RemoteTimer(this), CallBack_Handler(this)
{
	m_transmitDistance=3000.0;
  	m_maxPropDelay = Seconds(m_transmitDistance/1500.0);
  	m_RTSTxTime = m_maxPropDelay;
  	m_CTSTxTime = m_RTSTxTime + 2*m_maxPropDelay;

  	m_maxDataTxTime = MilliSeconds(m_dataPktSize/m_bitRate);  //1600bits/10kbps

  	m_rand = CreateObject<UniformRandomVariable> ();
  	Simulator::Schedule(Seconds(m_rand->GetValue(0.0,m_NDPeriod)+0.000001), &AquaSimSF::NDTimerExpire, this);
}

AquaSimSF::~AquaSimSF()
{
}

TypeId
AquaSimSF::GetTypeId(void)
{
  	static TypeId tid = TypeId("ns3::AquaSimSF")
      				.SetParent<AquaSimMac>()
      				.AddConstructor<AquaSimSF>()
      				.AddAttribute("MaxBurst", "The maximum number of packet burst. default is 1",
						IntegerValue(1),
						MakeIntegerAccessor (&AquaSimSF::m_maxBurst),
						MakeIntegerChecker<int>());
  	return tid;
}

int64_t
AquaSimSF::AssignStreams (int64_t stream)
{
  	NS_LOG_FUNCTION (this << stream);
  	m_rand->SetStream(stream);
  	return 1;
}

void
AquaSimSF::NDTimerExpire()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	SendPkt(MakeND());
  	m_sfNDCounter--;

  	if (m_sfNDCounter > 0) {
    		Simulator::Schedule(Seconds(m_rand->GetValue(0.0,m_NDPeriod)), &AquaSimSF::NDTimerExpire, this);
	}
}

void
AquaSimSF::SendPkt(Ptr<Packet> pkt)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

  	AquaSimHeader asHeader;
  	pkt->RemoveHeader(asHeader);

  	asHeader.SetDirection(AquaSimHeader::DOWN);


  	Time txtime = asHeader.GetTxTime();

  	switch( m_device->GetTransmissionStatus() ) {
    		case SLEEP:
      			PowerOn();
    		case NIDLE:
      			asHeader.SetTimeStamp(Simulator::Now());
      			pkt->AddHeader(asHeader);
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
AquaSimSF::TxProcess(Ptr<Packet> pkt)
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
  	SFHeader sfh;
	MacHeader mach;
	AquaSimPtTag ptag;
 	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
  	pkt->RemoveHeader(sfh);
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

  	sfh.SetPType(SFHeader::FAMA_DATA);
  	sfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	sfh.SetDA(asHeader.GetNextHop());

	pkt->AddHeader(sfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	PktQ.push(pkt);


  	//fill the next hop when sending out the packet;
  	if( (PktQ.size() == 1) /*the pkt is the first one*/ && SFStatus == PASSIVE ) {
      		if( CarrierDected() ) {
	 		DoRemote(2*m_maxPropDelay+m_estimateError);
      		}
      		else{
	 		SendRTS(2*m_maxPropDelay+m_CTSTxTime+m_RTSTxTime+m_estimateError);
      		}
  	}	
  	return true;
}


bool
AquaSimSF::RecvProcess(Ptr<Packet> pkt)
{
 

	NS_LOG_FUNCTION("Call: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "m_waitCTSTimer : " << m_waitCTSTimer.GetDelayLeft());

  	AquaSimHeader asHeader;
	MacHeader mach;
  	SFHeader sfh;
	AquaSimPtTag ptag;
  	pkt->RemoveHeader(asHeader);
	pkt->RemoveHeader(mach);
  	pkt->PeekHeader(sfh);
	pkt->PeekPacketTag(ptag);
	pkt->AddHeader(mach);
	pkt->AddHeader(asHeader);
  	AquaSimAddress dst = sfh.GetDA();
  
  	NS_LOG_FUNCTION("SFHeader: " << sfh);
  

  	if( m_backoffTimer.IsRunning() ) {
      		m_backoffTimer.Cancel();
      		DoRemote(2*m_maxPropDelay+m_estimateError);
  	} 
	else if( m_remoteTimer.IsRunning() ) {
      		m_remoteTimer.Cancel();
      		m_remoteExpireTime = Seconds(-1);
  	}

 	 /*ND is not a part of AquaSimFama. We just want to use it to get next hop
  	 *So we do not care wether it collides with others
  	 */
  	if( ( ptag.GetPacketType() == AquaSimPtTag::PT_FAMA)&& (sfh.GetPType()==SFHeader::ND) ) {
      		ProcessND(sfh.GetSA());
      		pkt=0;
      		return false;
  	}

  	if( asHeader.GetErrorFlag() )
  	{
      		//if(drop_)
		//drop_->recv(pkt,"Error/Collision");
    		//else
		pkt=0;

      		DoRemote(2*m_maxPropDelay+m_estimateError);
      		return false;
  	}

  	if( ptag.GetPacketType() == AquaSimPtTag::PT_FAMA ) {
      		switch( sfh.GetPType() ) {
			case SFHeader::RTS:
				ProcessRTS(sfh);
				break;
			case SFHeader::CTS:
				ProcessCTS(sfh);
          			break;
			default:
          			//process Data packet
          			if(ProcessDATA(sfh, pkt))
					return true;
          			
      		}
  	}

  	pkt=0;
  	return true;
}



void
AquaSimSF::SendDataPkt()
{
  	NS_LOG_FUNCTION("Call: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()));

  	int PktQ_Size = PktQ.size();
  	int SentPkt = 0;
  	Time StartTime = Simulator::Now();

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
        		NS_LOG_FUNCTION("StartTime: " << StartTime);
        		NS_LOG_FUNCTION("Simulator::Now(): " << Simulator::Now());
        		NS_LOG_FUNCTION("StartTime - Simulator::Now() => " << StartTime - Simulator::Now());
			Simulator::Schedule(StartTime-Simulator::Now(),&AquaSimSF::ProcessDataSendTimer, this, tmp);
			//Simulator::Schedule(Seconds(0), &AquaSimSF::ProcessDataSendTimer, this, tmp);
        		//PktQ.front()->PeekHeader(asHeader);
			if( !PktQ.empty() ) {
	  			StartTime += asHeader.GetTxTime() + m_dataPktInterval;
			}
			else {
	  			break;
			}
    		}
    		else{
			PktQ.push(tmp);
    		}
  	}	

  	//SFStatus = WAIT_DATA_FINISH;

  	Simulator::Schedule(m_maxPropDelay+StartTime-Simulator::Now(),&AquaSimSF::ProcessDataBackoffTimer,this);
}

void
AquaSimSF::ProcessDataSendTimer(Ptr<Packet> pkt)
{
  	NS_LOG_FUNCTION(this << pkt);
  	SendPkt(pkt);
}


void
AquaSimSF::ProcessDataBackoffTimer()
{
  	if( !PktQ.empty() )
    		DoBackoff();
  	else
    		SFStatus = PASSIVE;
}


Ptr<Packet>
AquaSimSF::MakeND()
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
  	SFHeader sfh;
	AquaSimPtTag ptag;

	asHeader.SetSize(2*sizeof(AquaSimAddress)+1);
  	asHeader.SetTxTime(GetTxTime(asHeader.GetSize()));
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(AquaSimAddress::GetBroadcast());

  	sfh.SetPType(SFHeader::ND);
  	sfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	sfh.SetDA(AquaSimAddress::GetBroadcast());

	pkt->AddHeader(sfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	
	return pkt;
}


void
AquaSimSF::ProcessND(AquaSimAddress sa)
{
  	//FamaHeader FamaH;
  	//pkt->PeekHeader(FamaH);
	NeighborList.push_back(sa);
}


Ptr<Packet>
AquaSimSF::MakeRTS(AquaSimAddress Recver)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
  	SFHeader sfh;
	AquaSimPtTag ptag;

  	asHeader.SetSize(GetSizeByTxTime(m_RTSTxTime.ToDouble(Time::S)));
  	asHeader.SetTxTime(m_RTSTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(Recver);

  	sfh.SetPType(SFHeader::RTS);
 	sfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	sfh.SetDA(Recver);

	pkt->AddHeader(sfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  	
	return pkt;
}


void
AquaSimSF::SendRTS(Time DeltaTime)
{
  	NS_LOG_FUNCTION(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	AquaSimHeader asHeader;
  	PktQ.front()->PeekHeader(asHeader);
  	SendPkt( MakeRTS(asHeader.GetNextHop()) );
  	NS_LOG_FUNCTION("After SendPkt");
  	SFStatus = WAIT_CTS;
  	NS_LOG_FUNCTION("m_waitCTSTimer : " << m_waitCTSTimer.GetDelayLeft());
  	NS_LOG_FUNCTION("DeltaTIme : " << DeltaTime);
  	m_waitCTSTimer.SetFunction(&AquaSimSF::DoBackoff,this);
  	m_waitCTSTimer.Schedule(DeltaTime);
}


void
AquaSimSF::ProcessRTS(SFHeader sfh)
{
  	if( sfh.GetDA() == m_device->GetAddress() ) 
	{
		SendPkt(MakeCTS(sfh.GetSA()));
  		SFStatus = WAIT_DATA;
	}

	DoRemote(m_CTSTxTime+2*m_maxPropDelay+m_estimateError);	 
}



Ptr<Packet>
AquaSimSF::MakeCTS(AquaSimAddress RTS_Sender)
{
  	NS_LOG_FUNCTION("Call : " <<  AquaSimAddress::ConvertFrom (m_device->GetAddress()) << "RTS Sender : " << RTS_Sender);

  	Ptr<Packet> pkt = Create<Packet>();
  	AquaSimHeader asHeader;
	MacHeader mach;
	SFHeader sfh;
	AquaSimPtTag ptag;

  	asHeader.SetSize(GetSizeByTxTime(m_CTSTxTime.ToDouble(Time::S)));
  	asHeader.SetTxTime(m_CTSTxTime);
  	asHeader.SetErrorFlag(false);
  	asHeader.SetDirection(AquaSimHeader::DOWN);
	ptag.SetPacketType(AquaSimPtTag::PT_FAMA);
  	asHeader.SetNextHop(RTS_Sender);

  	sfh.SetPType(SFHeader::CTS);
  	sfh.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
  	sfh.SetDA(RTS_Sender);

	pkt->AddHeader(sfh);
	pkt->AddHeader(mach);
  	pkt->AddHeader(asHeader);
	pkt->AddPacketTag(ptag);
  
	return pkt;
}

void
AquaSimSF::ProcessCTS(SFHeader sfh)
{
	if(m_waitCTSTimer.IsRunning()) {
        	m_waitCTSTimer.Cancel();
            
              	if(sfh.GetDA() == m_device->GetAddress()) {
                  	SendDataPkt();
              	}
              	else {
                  	DoBackoff();
            	} 
     	}		 

        // this CTS must not be for this node
        DoRemote(2*m_maxPropDelay+m_estimateError);
}

bool
AquaSimSF::ProcessDATA(SFHeader sfh, Ptr<Packet> pkt)
{	
	if( sfh.GetDA() == m_device->GetAddress() ) {
		//ptag.SetPacketType(UpperLayerPktType);
                NS_LOG_INFO("Process Data Packet!!!!");
	    	SFStatus = WAIT_DATA_FINISH;
		SendUp(pkt);
	    	return true;
	}
	else {
		DoRemote(m_maxPropDelay+m_estimateError);
		return false;
	}
}

bool
AquaSimSF::CarrierDected()
{
  	if( m_device->GetTransmissionStatus() == RECV || m_device->GetTransmissionStatus() == SEND )  {
	  	return true;
  	}
	
	return false;
}

void
AquaSimSF::DoBackoff()
{
  	Time backoffTime = MilliSeconds(m_rand->GetValue(0.0,10 * m_RTSTxTime.ToDouble(Time::MS)));
 
  	SFStatus = BACKOFF;
  	if( m_backoffTimer.IsRunning() ) {
      		m_backoffTimer.Cancel();
  	}

  	NS_LOG_FUNCTION("m_backoffTimer.GetDelay() : " << m_backoffTimer.GetDelayLeft());
  	m_backoffTimer.SetFunction(&AquaSimSF::BackoffTimerExpire,this);
  	m_backoffTimer.Schedule(backoffTime);
}


void
AquaSimSF::DoRemote(Time DeltaTime)
{
  	SFStatus = REMOTE;

  	if( Simulator::Now()+DeltaTime > m_remoteExpireTime ) {
      		m_remoteExpireTime = Simulator::Now()+DeltaTime;
      		
		if( m_remoteTimer.IsRunning() ) {
	  		m_remoteTimer.Cancel();
      		}
      		m_remoteTimer.SetFunction(&AquaSimSF::ProcessRemoteTimer,this);
      		m_remoteTimer.Schedule(DeltaTime);
      		NS_LOG_FUNCTION("m_remoteTimer.GetDelay() : " << m_remoteTimer.GetDelayLeft());
  	}
}


void
AquaSimSF::ProcessRemoteTimer()
{
  	if( PktQ.empty() ) {
    		SFStatus = PASSIVE;
  	}
  	else {
    		DoBackoff();
    		//SendRTS(2*m_maxPropDelay+m_CTSTxTime+m_RTSTxTime+m_estimateError);
  	}
}

void
AquaSimSF::BackoffTimerExpire()
{
  	SendRTS(2*m_maxPropDelay + m_RTSTxTime + m_CTSTxTime +m_estimateError);
}

void AquaSimSF::DoDispose()
{
	m_rand=0;
	while(!PktQ.empty()) {
		PktQ.front()=0;
		PktQ.pop();
	}
	AquaSimMac::DoDispose();
}

} // namespace ns2
