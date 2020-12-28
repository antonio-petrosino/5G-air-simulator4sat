/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 TELEMATICS LAB, Politecnico di Bari
 *
 * This file is part of 5G-air-simulator
 *
 * 5G-air-simulator is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation;
 *
 * 5G-air-simulator is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with 5G-air-simulator; if not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Telematics Lab <telematics-dev@poliba.it>
 * Author: Alessandro Grassi <alessandro.grassi@poliba.it>
 */


#include "gnb-phy.h"
#include "ue-phy.h"
#include "../device/NetworkNode.h"
#include "../channel/RadioChannel.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../phy/nbiot-simple-error-model.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../core/spectrum/transmitted-signal.h"
#include "../core/idealMessages/ideal-control-messages.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "interference.h"
#include "error-model.h"
#include "../channel/propagation-model/propagation-loss-model.h"
#include "../protocolStack/mac/AMCModule.h"
#include "../utility/eesm-effective-sinr.h"
#include "../utility/miesm-effective-sinr.h"
#include "../componentManagers/FrameManager.h"
#include "../protocolStack/mac/nb-AMCModule.h"


#define UL_INTERFERENCE 4

GnbPhy::GnbPhy()
{
  SetDevice(nullptr);
  SetDlChannel(nullptr);
  SetUlChannel(nullptr);
  SetBandwidthManager(nullptr);
  SetTxSignal (nullptr);
  SetErrorModel (nullptr);
  SetInterference (nullptr);
  SetTxPower(33); //dBm
  SetTxAntennas(1);
  SetRxAntennas(1);
  GetAntennaParameters ()->SetType(Phy::AntennaParameters::ANTENNA_TYPE_OMNIDIRECTIONAL);
  GetAntennaParameters ()->SetBearing(0);
  GetAntennaParameters ()->SetEtilt(15);
}

GnbPhy::~GnbPhy()
{
  Destroy ();
}

void
GnbPhy::DoSetBandwidthManager (void)
{
  BandwidthManager* s = GetBandwidthManager ();
  vector<double> channels = s->GetDlSubChannels ();

  TransmittedSignal* txSignal = new TransmittedSignal ();

  vector< vector<double> > values;
  values.resize( GetTxAntennas () );

  double powerTx = pow (10., (GetTxPower () - 30) / 10); // in natural unit

  double txPower = 10 * log10 (powerTx / channels.size () ); //in dB

  for (int i = 0; i < GetTxAntennas (); i++)
    {
      for (auto channel : channels)
        {
          values.at(i).push_back(txPower);
        }
    }
  txSignal->SetValues (values);
  //txSignal->SetBandwidthManager (s->Copy());

  SetTxSignal (txSignal);
}

void
GnbPhy::StartTx (shared_ptr<PacketBurst> p)
{
  //cout << "Node " << GetDevice()->GetIDNetworkNode () << " starts phy tx" << endl;

  if (FrameManager::Init()->MbsfnEnabled()==true && FrameManager::Init()->isMbsfnSubframe()==true)
    {
      GetDlMcChannel ()->StartTx (p, GetTxSignal (), GetDevice ());
    }
  else
    {
      GetDlChannel ()->StartTx (p, GetTxSignal (), GetDevice ());
    }
}

void
GnbPhy::StartRx (shared_ptr<PacketBurst> p, TransmittedSignal* txSignal)
{

//cout << "StartRX gnb-phy #113"<<endl;

DEBUG_LOG_START_1(SIM_ENV_TEST_DEVICE_ON_CHANNEL)
  cout << "Node " << GetDevice()->GetIDNetworkNode () << " starts phy rx" << endl;
DEBUG_LOG_END

  //COMPUTE THE SINR
  vector<double> measuredSinr;
  vector<int> channelsForRx;
  vector<double> rxSignalValues = txSignal->GetValues().at(0);

  double interference = 0;
  double noise_interference = 10. * log10 (pow(10., GetThermalNoise()/10) + interference); // dB - solo noise nel nostro caso
  //double noise_interference = 0;

  bool satScenario = GetDevice()-> GetPhy()->GetBandwidthManager()->GetNBIoTenabled();
  //double measuredSNR = 0.0;

	//if (satScenario == true){
		//UePhy* uePhy = (UePhy*) GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetPhy();
		//double ElAngle = uePhy->GetDevice()->GetMobilityModel ()->GetAbsolutePosition ()->GetElAngle (GetDevice()->GetMobilityModel()->GetAbsolutePosition());
		//measuredSNR = GetSNRfromElAngle_SAT(ElAngle);

	//}

  int chId = 0;
  for ( auto power : rxSignalValues ) // transmission power for the current sub channel [dB]
    {
      if (power != 0.)
        {
          channelsForRx.push_back (chId);
        }
      chId++;

      if(satScenario){
    	  measuredSinr.push_back(power - GetSatelliteNoisePowerDB());
      }
      else
      {
    	  measuredSinr.push_back (power - noise_interference - UL_INTERFERENCE);
      }
    }

  //CHECK FOR PHY ERROR
  bool phyError = false;

  if (GetErrorModel() != nullptr && p->GetNPackets() > 0)
    {
    vector<int> approxMCS;
    int MCS_ = FrameManager::Init()->GetMCSNBIoTSat ();
    //int RU_ = FrameManager::Init()->GetNRUNBIoTSat();
    int RU_ = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetNRUtoUE();

    UserEquipment* ue = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE();
    approxMCS.push_back(((NBIoTSimpleErrorModel*) GetErrorModel())-> GetRefMCS(MCS_,RU_));

    //phyError = GetErrorModel ()->CheckForPhysicalError (channelsForRx, approxMCS, measuredSinr);

    if(FrameManager::Init()->GetHARQ()){
    	//int nTxDone = GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx();

    	int TBS_ = GetDevice()->GetMacEntity()->GetNbAmcModule()->GetTBSizeFromMCS (MCS_, RU_);
    	int requiredRx = GetMaxRequiredHARQretx(measuredSinr.at(channelsForRx.at(0)));

    	// se il ritardo è superiore ad una certa soglia considero la trasmissione relativa al precedente satellite, discorso azzerato
    	double delayHARQ = Simulator::Init()->Now() - GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetLastHARQTimestamp();
    	if(delayHARQ > 400){
    		if(GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() > 0){
    			GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetHARQretx(0);
    		}
    	}

    	GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetLastHARQTimestamp(Simulator::Init()->Now());

    	if(GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1 < requiredRx){

    		phyError = true;
    		cout << "HARQ_ERR 1 RX_n: " << GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1 << " / " << requiredRx << endl;
    		GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetHARQretx(GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1);

    	}else{

    		//phyError = false;
    		phyError = ((NBIoTSimpleErrorModel*)GetErrorModel ())->CheckForPhysicalErrorHARQ (channelsForRx, measuredSinr);
    		cout << "HARQ_ERR "<< phyError <<" RX_n: " << GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->GetHARQretx() + 1 << " / " << requiredRx << endl;
    		GetDevice()->GetUserEquipmentRecord(p->GetPackets().front()->GetSourceID())->GetUE()->SetHARQretx(0);
    		//cout << "OK HARQ, non richiesta ritrasmissione." << endl;

    	}

	/*	cout << "PHY_RX SRC " << ue->GetIDNetworkNode()
		<< " DST " << GetDevice()->GetIDNetworkNode()
		<< " DISTANCE " << "0.0"
		<< " ELEVATION " << "0.0"
		<< " SNR " << measuredSinr.at(channelsForRx.at(0)) //TODO: THIS ONLY WORKS FOR SINGLE TONE
		<< " RU " << RU_
		<< " MCS " << MCS_
		<< " SIZE " << TBS_
		<< " ERR " << phyError
		<< " T " << Simulator::Init()->Now()
		<< endl;
		*/
    }else{

    	phyError = GetErrorModel ()->CheckForPhysicalError (channelsForRx, approxMCS, measuredSinr);
     }

    if (_PHY_TRACING_) {

                CartesianCoordinates* uePos = ue->GetMobilityModel()->GetAbsolutePosition();
                CartesianCoordinates* gnbPos = GetDevice()->GetMobilityModel()->GetAbsolutePosition();

                double DIST_ = uePos->GetDistance3D (gnbPos);


                int TBS_ = GetDevice()->GetMacEntity()->GetNbAmcModule()->GetTBSizeFromMCS (MCS_, RU_);
                double EL_ = 0.0;
                //double EL_ = ue->GetMobilityModel()->GetAbsolutePosition() ->GetElAngle(GetDevice()->GetMobilityModel()->GetAbsolutePosition());
            	if (GetDevice()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
            	{
            		EL_ =((SatelliteMovement*) GetDevice()->GetMobilityModel())->GetElAngle(ue->GetMobilityModel()->GetAbsolutePosition());
            	}

                cout << "PHY_RX SRC " << ue->GetIDNetworkNode()
                << " DST " << GetDevice()->GetIDNetworkNode()
                << " DISTANCE " << DIST_
                << " ELEVATION " << EL_
                << " SNR " << measuredSinr.at(channelsForRx.at(0)) //TODO: THIS ONLY WORKS FOR SINGLE TONE
                << " RU " << RU_
                << " MCS " << MCS_
                << " SIZE " << TBS_
                << " ERR " << phyError
                << " T " << Simulator::Init()->Now()
                << endl;
            }
   }


  if (!phyError && p->GetNPackets() > 0)
    {
      //FORWARD RECEIVED PACKETS TO THE DEVICE
      GetDevice()->ReceivePacketBurst(p);
    }

  delete txSignal;
}

void
GnbPhy::SendIdealControlMessage (IdealControlMessage *msg)
{
  GNodeB *gnb = GetDevice ();
  switch( msg->GetMessageType() )
    {
    case IdealControlMessage::ALLOCATION_MAP:
        {
          PdcchMapIdealControlMessage *pdcchMsg =  (PdcchMapIdealControlMessage*)msg;
          for (auto ue : pdcchMsg->GetTargetUEs() )
            {
              ue->GetPhy ()->ReceiveIdealControlMessage (msg);
            }
        }
      break;

    case IdealControlMessage::NB_IOT_ALLOCATION_MAP:
        {
          NbIoTMapIdealControlMessage *pdcchMsg =  (NbIoTMapIdealControlMessage*)msg;
          for (auto record : *pdcchMsg->GetMessage())
            {
              record.m_ue->GetPhy ()->ReceiveIdealControlMessage (msg);
            }
        }
      break;

    case IdealControlMessage::RA_RESPONSE:
    case IdealControlMessage::RA_CONNECTION_RESOLUTION:
      msg->GetDestinationDevice()->GetPhy ()->ReceiveIdealControlMessage (msg);
      break;

    default:
      cout << "Error in GnbPhy::SendIdealControlMessage: unknown message type (" << msg->GetMessageType() << ")" << endl;
      exit(1);
    }

  delete msg;
}

void
GnbPhy::ReceiveIdealControlMessage (IdealControlMessage *msg)
{
  GNodeB* gnb = GetDevice();
  gnb->GetMacEntity()->ReceiveIdealControlMessage(msg);
}

void
GnbPhy::ReceiveReferenceSymbols (NetworkNode* n, TransmittedSignal* s)
{
  GNodeB::UserEquipmentRecord* user = ((UserEquipment*) n)->GetTargetNodeRecord();
  ReceivedSignal* rxSignal;
  if (GetUlChannel ()->GetPropagationLossModel () != nullptr)
    {
      rxSignal = GetUlChannel ()->GetPropagationLossModel ()->
                 AddLossModel (n, GetDevice (), s);
    }
  else
    {
      rxSignal = s->Copy ();
    }
  AMCModule* amc = user->GetUE()->GetProtocolStack ()->GetMacEntity ()->GetAmcModule ();
  vector<double> ulQuality;
  vector<double> rxSignalValues = rxSignal->GetValues ().at(0);
  delete rxSignal;
  double noise_interference = 10. * log10 (pow(10., GetThermalNoise()/10)); // dB
  for (auto power : rxSignalValues)
    {
      ulQuality.push_back (power - noise_interference - UL_INTERFERENCE);
    }


DEBUG_LOG_START_1(SIM_ENV_TEST_UL_SINR)
  double effectiveSinr = GetMiesmEffectiveSinr (ulQuality);
  if (effectiveSinr > 40) effectiveSinr = 40;
  int mcs = amc->GetMCSFromCQI (amc->GetCQIFromSinr(effectiveSinr));
  cout << "UL_SINR " << n->GetIDNetworkNode () << " "
            << n->GetMobilityModel ()->GetAbsolutePosition()->GetCoordinateX () << " "
            << n->GetMobilityModel ()->GetAbsolutePosition()->GetCoordinateY () << " "
            << effectiveSinr << " " << mcs << endl;
DEBUG_LOG_END


  user->SetUplinkChannelStatusIndicator (ulQuality);
}


GNodeB*
GnbPhy::GetDevice(void)
{
  Phy* phy = (Phy*)this;
  return (GNodeB*)phy->GetDevice();
}
