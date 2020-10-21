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

#include "ue_Satellite.h"
#include "../componentManagers/NetworkManager.h"
#include "../device/Gateway.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "../load-parameters.h"

Ue_SatelliteMovement::Ue_SatelliteMovement()
{
  SetMobilityModel(Mobility::UE_SATELLITE);
  SetSpeed (0);
  SetSpeedDirection (0.0);
  SetPositionLastUpdate (0.0);
  SetHandover (false); //??
  SetLastHandoverTime (0.0);
  SetAbsolutePosition (nullptr);

  cout <<"Costruttore del movimento UE satellitare avviato!" << endl;
}

Ue_SatelliteMovement::~Ue_SatelliteMovement()
{
  DeleteAbsolutePosition ();
}

void
Ue_SatelliteMovement::UpdatePosition (double time)
{
	double timeInterval = time - GetPositionLastUpdate ();

	UserEquipment *thisNode = (UserEquipment*)GetDevice();
	Cell *thisCell = thisNode->GetCell ();

	NetworkNode *targetNode = thisNode->GetTargetNode ();

	CartesianCoordinates *newPosition =
	new CartesianCoordinates(GetAbsolutePosition()->GetCoordinateX(),
	                         GetAbsolutePosition()->GetCoordinateY(),
	                         GetAbsolutePosition()->GetCoordinateZ());
	newPosition->SetFloorHeight( GetAbsolutePosition()->GetFloorHeight() );
	//SetAbsolutePosition(newPosition);


  CartesianCoordinates *GNodeBPosition = targetNode->GetMobilityModel ()->GetAbsolutePosition ();

  double newDistanceFromTheGNodeB = newPosition->GetDistance3D (GNodeBPosition);

  // come aggiornare le distanze delle UE?

  SetPositionLastUpdate (time);

  delete newPosition;
}
