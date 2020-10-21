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
 */

#include "Satellite.h"
#include "SatelliteCoordinate.h"
#include "../componentManagers/NetworkManager.h"
#include "../device/Gateway.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "../load-parameters.h"


SatelliteMovement::SatelliteMovement()
{
  SetMobilityModel(Mobility::SATELLITE);
  SetSpeed (0);
  SetSpeedDirection (0.0);
  SetPositionLastUpdate (0.0);
  SetHandover (false); //??
  SetLastHandoverTime (0.0);
  SetAbsolutePosition (nullptr);
  cout <<"Costruttore del movimento gNB satellitare avviato!" << endl;
}

SatelliteMovement::~SatelliteMovement()
{
  DeleteAbsolutePosition ();
}

void
SatelliteMovement::UpdatePosition (double time)
{
DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
  cout << "\t START MOBILITY MODEL for "<< GetDevice ()->GetIDNetworkNode() << endl;
DEBUG_LOG_END

  if (GetSpeed () == 0)
    {
DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
      cout << "\t\t speed = 0 --> position has not been updated!"<< endl;
DEBUG_LOG_END
      return;
    }

  double timeInterval = time - GetPositionLastUpdate ();
  if(timeInterval > 0.001){

  //UserEquipment *thisNode = (UserEquipment*)GetDevice();
  //Cell *thisCell = thisNode->GetCell ();
  //NetworkNode *targetNode = thisNode->GetTargetNode ();

  // movimento della GNodeB
	  GNodeB *thisNode = (GNodeB*)GetDevice();


	DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
	  cout << "MOBILITY_DEBUG: User ID: " << GetDevice ()->GetIDNetworkNode()
				<< "\n\t Cell ID " <<
				NetworkManager::Init()->GetCellIDFromPosition (GetAbsolutePosition()->GetCoordinateX(),
					GetAbsolutePosition()->GetCoordinateY())
				<< "\n\t Initial Position (X): " << GetAbsolutePosition()->GetCoordinateX()
				<< "\n\t Initial Position (Y): " << GetAbsolutePosition()->GetCoordinateY()
				<< "\n\t Speed: " << GetSpeed()
				<< "\n\t Speed Direction: " << GetSpeedDirection()
				<< "\n\t Time Last Update: " << GetPositionLastUpdate()
				<< "\n\t Time Interval: " << timeInterval
				<< endl;
	DEBUG_LOG_END
	DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG_TAB)
	  cout <<  GetDevice ()->GetIDNetworkNode() << ""
				<< GetAbsolutePosition()->GetCoordinateX() << " "
				<< GetAbsolutePosition()->GetCoordinateY() << " "
				<< GetSpeed() << " "
				<< GetSpeedDirection() << " "
				<< GetPositionLastUpdate() << " "
				<< timeInterval << " "
				<< endl;
	DEBUG_LOG_END

	  double shift = timeInterval * (GetSpeed()*(1000.0/3600.0));
      double x_pos = 0.0;
		// è espresso in metri al secondo se la velocita e data in km/h


// inserire modello di movimento
			//  double shift_y =
			//    shift * sin (GetSpeedDirection());
			//  double shift_x =
			//    shift * cos (GetSpeedDirection());
			//  double shift_z =
			//    shift * 0;

	x_pos = GetSatPosition(time);
// fine calcolo di movimento

	  CartesianCoordinates *newPosition =
		new CartesianCoordinates(x_pos,
								 GetAbsolutePosition()->GetCoordinateY(),
								 GetAbsolutePosition()->GetCoordinateZ());
	  newPosition->SetFloorHeight( GetAbsolutePosition()->GetFloorHeight() );
	  SetAbsolutePosition(newPosition);
	  SetPositionLastUpdate (time);

  //CartesianCoordinates origin(0,0);
  //SetAbsolutePosition( GetWrapAroundPosition(&origin) );

  //CartesianCoordinates *GNodeBPosition = targetNode->GetMobilityModel ()->GetAbsolutePosition ();

  //const double pi = M_PI;
  //double azimut = newPosition->GetPolarAzimut (GNodeBPosition);
  //double newDistanceFromTheGNodeB = newPosition->GetDistance (GNodeBPosition);

  // come aggiornare le distanze delle UE?



	DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
	  cout << "\n\t Final Position (X): " << GetAbsolutePosition()->GetCoordinateX()
				<< "\n\t Final Position (Y): " << GetAbsolutePosition()->GetCoordinateY()
				<< endl;
	DEBUG_LOG_END
	DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG_TAB)
	  cout << GetAbsolutePosition()->GetCoordinateX() << " "
				<< GetAbsolutePosition()->GetCoordinateY()
				<< endl;
	DEBUG_LOG_END

  	  delete newPosition;
  }
}
