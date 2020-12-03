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
 * Author: Antonio Petrosino  <antonio.petrosino@poliba.it>
 * Author: Giancarlo Sciddurlo  <giancarlo.sciddurlo@poliba.it>
 */

#ifndef SATELLITEMOVEMENT_H_
#define SATELLITEMOVEMENT_H_

#include "Mobility.h"

class SatelliteMovement :public Mobility
{
public:
  SatelliteMovement(int nSat);
  virtual ~SatelliteMovement();

  	  enum AntennaType {
      	PATCH_ANTENNA,
  		PARABOLIC_REFLECTOR
  	  };

    void SetMCLthreshold(double SNRthreshold);
    double GetMCLthreshold(void);

    double GetSatPosition (double time);

    void SetNumberOfSatellitePerOrbit(int nSat);
    int GetNumberOfSatellitePerOrbit(void);

    void SetTimePositionUpdate(double _update);
    double GetTimePositionUpdate(void);

    void SetTimeOrbitPeriod(double _update);
    double GetTimeOrbitPeriod(void);

    void SetFixedAreaRadius(double radius);
    double GetFixedAreaRadius(void);

    //void SetSpotBeamRadius(double radius);
    double GetSpotBeamRadius(void);

    bool GetAttachProcedure(CartesianCoordinates* _uePos);
    double GetElAngle(CartesianCoordinates* _uePos);

    double GetVisibilityPeriod();

    SatelliteMovement::AntennaType
	GetAntennaType(void) const;
    void
    SetAntennaType(AntennaType model);
    
    double GetSatPositionFromElAngle(CartesianCoordinates *remoteObject, double elangle);
    double GetNextUsefulElevationAngle(double currentElAngle);
    double GetTimeNeededForDestination(double satPosition);
    double GetNextTimePositionUpdate(CartesianCoordinates *remoteObject);

  void
  UpdatePosition (double time);
    
private:
    int m_numSatellitePerOrbit;
    double m_gNBtimePositionUpdate;
    double m_timeOrbitPeriod;
    double m_SNRthreshold;
    double m_fixedAreaRadius;
    double m_spotBeamRadius;
    AntennaType m_AntennaType;
};

#endif /* SATELLITE_H_ */
