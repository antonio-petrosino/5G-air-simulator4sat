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
 * Author: Sergio Martiradonna <sergio.martiradonna@poliba.it>
 */


#include "UserEquipment.h"
#include "NetworkNode.h"
#include "GNodeB.h"
#include "HeNodeB.h"
#include "Gateway.h"
#include "../phy/ue-phy.h"
#include "CqiManager/cqi-manager.h"
//#include "../mobility/Mobility.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../core/eventScheduler/simulator.h"
#include "../componentManagers/NetworkManager.h"
#include "../flows/radio-bearer.h"
#include "../flows/MacQueue.h"
#include "../protocolStack/rrc/ho/handover-entity.h"
#include "../protocolStack/rrc/ho/ho-manager.h"


UserEquipment::UserEquipment (int idElement,
                              double posx,
                              double posy,
                              Cell *cell,
                              GNodeB* target,
                              bool handover,
                              Mobility::MobilityModel model)
{
  SetIDNetworkNode (idElement);
  SetNodeType(NetworkNode::TYPE_UE);
  SetCell(cell);

  m_targetNode = target;

  ProtocolStack *stack = new ProtocolStack (this);
  SetProtocolStack (stack);

  Classifier *classifier = new Classifier ();
  classifier->SetDevice (this);
  SetClassifier (classifier);
  SetNodeState(STATE_IDLE);

  //Setup Mobility Model
  Mobility *m;
  if (model == Mobility::RANDOM_DIRECTION)
    {
      m = new RandomDirection ();
    }
  else if (model == Mobility::RANDOM_WALK)
    {
      m = new RandomWalk ();
    }
  else if (model == Mobility::RANDOM_WAYPOINT)
    {
      m = new RandomWaypoint ();
    }
  else if (model == Mobility::CONSTANT_POSITION)
    {
      m = new ConstantPosition ();
    }
  else if (model == Mobility::MANHATTAN)
    {
      m = new Manhattan ();
    }
  else if (model == Mobility::LINEAR_MOVEMENT)
    {
      m = new LinearMovement ();
    }
  else if (model == Mobility::SATELLITE)
    {
      m = new SatelliteMovement (1);
    }
  else if (model == Mobility::UE_SATELLITE)
    {
      m = new UeSatelliteMovement ();
    }
  else
    {
      cout << "ERROR: incorrect Mobility Model"<< endl;
      exit(1);
    }
  CartesianCoordinates *position = new CartesianCoordinates (posx, posy, 1.5);
  m->SetHandover (handover);
  m->SetAbsolutePosition (position);
  m->SetDevice (this);
  SetMobilityModel (m);

  m_timePositionUpdate = 0.001;
  Simulator::Init()->Schedule(m_timePositionUpdate,
                              &UserEquipment::UpdateUserPosition,
                              this,
                              Simulator::Init ()->Now());

  delete position;

  UePhy *phy = new UePhy ();
  phy->SetDevice(this);
  phy->SetBandwidthManager (target->GetPhy ()->GetBandwidthManager ());
  SetPhy(phy);

  m_cqiManager = nullptr;
  m_isIndoor = false;

  m_activityTimeout = 10;
  m_activityTimeoutEvent = NULL;
}

UserEquipment::UserEquipment (int idElement,
                              double posx,
                              double posy,
                              int speed,
                              double speedDirection,
                              Cell *cell,
                              GNodeB* target,
                              bool handover,
                              Mobility::MobilityModel model)
    : UserEquipment::UserEquipment (idElement,
                                    posx,
                                    posy,
                                    cell,
                                    target,
                                    handover,
                                    model)
{
  GetMobilityModel()->SetSpeed(speed);
  GetMobilityModel()->SetSpeedDirection(speedDirection);
}

UserEquipment::~UserEquipment()
{
  m_targetNode = nullptr;
  delete m_cqiManager;
  Destroy ();
}

void
UserEquipment::SetTargetNode (GNodeB* n)
{
  m_targetNode = n;
  SetCell (n->GetCell ());
}

GNodeB*
UserEquipment::GetTargetNode (void)
{
  return m_targetNode;
}

void
UserEquipment::SetTimePositionUpdate (double time)
{
  m_timePositionUpdate = time;
}

double
UserEquipment::GetTimePositionUpdate (void)
{
  return m_timePositionUpdate;
}

void
UserEquipment::UpdateUserPosition (double time)
{

	if(fmod(time-0.001, ((SatelliteMovement*) GetTargetNode ()->GetMobilityModel())->GetVisibilityPeriod()) < 150){

	GetMobilityModel ()->UpdatePosition (time);

DEBUG_LOG_START_1(SIM_ENV_HANDOVER_DEBUG)
  cout << "Aggiornamento posizione UE... time: "<< time <<endl;
DEBUG_LOG_END

  SetIndoorFlag(NetworkManager::Init()->CheckIndoorUsers(this));

  if (GetMobilityModel ()->GetHandover () == true)
    {
      GNodeB* targetNode = GetTargetNode ();

      if (targetNode->GetProtocolStack ()->GetRrcEntity ()->
          GetHandoverEntity ()->CheckHandoverNeed (this))
        {
          GNodeB* newTargetNode = targetNode->GetProtocolStack ()
                                       ->GetRrcEntity ()->GetHandoverEntity ()->GetHoManager ()->m_target;
DEBUG_LOG_START_1(SIM_ENV_HANDOVER_DEBUG)
          cout << "** HO ** \t time: " << time << " user " <<  GetIDNetworkNode () <<
                    " old gNB " << targetNode->GetIDNetworkNode () <<
                    " new gNB " << newTargetNode->GetIDNetworkNode () << endl;
DEBUG_LOG_END
          NetworkManager::Init()->HandoverProcedure(time, this, targetNode, newTargetNode);
        }
    }
  //blocco else che funzionerà solo per lo scenario NB-IoT in cui l'handover sarà sicuramente messo a falso
  else if (GetTargetNode()->GetPhy()->GetBandwidthManager()->GetNBIoTenabled() == true)
  {
	CartesianCoordinates* uePos = GetMobilityModel()->GetAbsolutePosition();
	CartesianCoordinates* gnbPos = GetTargetNode ()->GetMobilityModel()->GetAbsolutePosition();

	double distance = uePos->GetDistance3D (gnbPos);
	bool _attach = false;
	if (GetTargetNode()->GetMobilityModel()->GetMobilityModel() == Mobility::SATELLITE)
	{
		 _attach =((SatelliteMovement*) GetTargetNode()->GetMobilityModel()) ->GetAttachProcedure(uePos);
	}


	//if(distance > maxSatelliteRange) //600km 55° -> 547km 65°
	if(!_attach)
	{
		//if(GetNodeState() != UserEquipment::STATE_DETACHED){
		if(GetNodeState() == UserEquipment::STATE_IDLE){
DEBUG_LOG_START_1(SIM_ENV_HANDOVER_DEBUG)
cout<<"Procedura di !!!! DETACH !!!! avviata a tempo: "<< time << " UE id: "<< GetIDNetworkNode () << " distanza:" << distance <<endl;
Print();
DEBUG_LOG_END
			//cout<<"Procedura di !!!! DETACH !!!! avviata a tempo: "<< time << " UE id: "<< GetIDNetworkNode () << " distanza:" << distance <<endl;
			SetNodeState (UserEquipment::STATE_DETACHED);
            if (GetTargetNode()->GetAttachedUEs()>0)
                GetTargetNode ()->UpdateAttachedUEs(-1);
		}
	}else{

		if(GetNodeState() == UserEquipment::STATE_DETACHED){
DEBUG_LOG_START_1(SIM_ENV_HANDOVER_DEBUG)
cout<<"Procedura di !!!!  ATTACH  !!!! avviata a tempo: "<< time << " UE id: "<< GetIDNetworkNode () <<" distanza:" << distance << endl;
Print();
DEBUG_LOG_END
			//cout<<"Procedura di !!!!  ATTACH  !!!! avviata a tempo: "<< time << " UE id: "<< GetIDNetworkNode () <<" distanza:" << distance << endl;
			SetNodeState (UserEquipment::STATE_IDLE);
            GetTargetNode ()->UpdateAttachedUEs(1);

			bool needRAP = false;

			RrcEntity *rrc = GetProtocolStack ()->GetRrcEntity ();

			if (rrc->GetRadioBearerContainer ()->size() > 0){
				for (auto bearer : *rrc->GetRadioBearerContainer()){
					if (bearer->GetMacQueue()->GetNbDataPackets()>0){
						needRAP = true;
					}
				}
			}

			if(needRAP){
				Simulator::Init()->Schedule(0.0, &UeRandomAccess::StartRaProcedure, GetMacEntity()->GetRandomAccessManager());
			}
		}
	}
  }
}

  if (GetMobilityModel ()-> GetMobilityModel() != Mobility::CONSTANT_POSITION) {
    //schedule the new update after m_timePositionUpdate

	  //cout << "Periodicità: " << fmod(time, 3000.0) << "time: " << time << endl;

/*
	if(fmod(time, 3000.0) < 160.001){
    Simulator::Init()->Schedule(m_timePositionUpdate,
                              &UserEquipment::UpdateUserPosition,
                              this,
                              Simulator::Init ()->Now());
	}else{

	Simulator::Init()->Schedule(3000.001 - fmod(time,3000.00) ,
	                            &UserEquipment::UpdateUserPosition,
	                            this,
	                            Simulator::Init ()->Now() + 3000.001 - fmod(time,3000.00)); // chiamo up_us_pos con now()
								//now + 3000
	// se overload non passare niente, fino a this
	cout <<"Simulatore: schedulato fra " << 3000.001 - fmod(time,3000.0) << " sec." <<endl;
	}
*/
	  Simulator::Init()->Schedule(m_timePositionUpdate,
	                                &UserEquipment::UpdateUserPosition,
	                                this,
	                                Simulator::Init ()->Now());
  }
}

UeMacEntity*
UserEquipment::GetMacEntity(void) const
{
  return (UeMacEntity*)GetProtocolStack()->GetMacEntity();
}

void
UserEquipment::SetTargetNodeRecord (GNodeB::UserEquipmentRecord *r)
{
  m_targetNodeRecord = r;
}

GNodeB::UserEquipmentRecord*
UserEquipment::GetTargetNodeRecord (void)
{
  return m_targetNodeRecord;
}


void
UserEquipment::SetCqiManager (CqiManager *cm)
{
  m_cqiManager = cm;
}

CqiManager*
UserEquipment::GetCqiManager (void)
{
  return m_cqiManager;
}

void
UserEquipment::SetIndoorFlag ( bool flag)
{
  m_isIndoor = flag;
}

bool
UserEquipment::IsIndoor (void)
{
  return m_isIndoor;
}

void
UserEquipment::SetLastActivity()
{
  if (m_activityTimeoutEvent != NULL)
    {
      m_activityTimeoutEvent->MarkDeleted();
    }
  m_activityTimeoutEvent = Simulator::Init()->Schedule(m_activityTimeout, &UserEquipment::SetNodeState, this, NetworkNode::STATE_IDLE );
}

void
UserEquipment::SetActivityTimeout(double timeout)
{
  m_activityTimeout = timeout;
}

double
UserEquipment::GetActivityTimeout()
{
  return m_activityTimeout;
}

void UserEquipment::SetRandomAccessType(UeRandomAccess::RandomAccessType type)
{
  GetMacEntity ()->SetRandomAccessType(type);
}

//Debug
void
UserEquipment::Print (void)
{
  cout << " UserEquipment object:"
            "\n\t m_idNetworkNode = " << GetIDNetworkNode () <<
            "\n\t idCell = " << GetCell ()->GetIdCell () <<
            "\n\t idtargetNode = " << GetTargetNode ()->GetIDNetworkNode () <<
            "\n\t m_AbsolutePosition_X = " <<  GetMobilityModel ()->GetAbsolutePosition()->GetCoordinateX()<<
            "\n\t m_AbsolutePosition_Y = " << GetMobilityModel ()->GetAbsolutePosition()->GetCoordinateY()<<
            "\n\t m_speed = " << GetMobilityModel ()->GetSpeed () <<
            "\n\t m_speedDirection = " << GetMobilityModel ()->GetSpeedDirection () <<
            endl;
}
