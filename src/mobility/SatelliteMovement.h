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
    
    void SetPeriod(double _period);
    double GetPeriod(void);
    double GetSatPosition (double time);
    void SetNumberOfSatellitePerOrbit(int nSat);
    int GetNumberOfSatellitePerOrbit(void);
    void SetTimePositionUpdate(double _update);
    double GetTimePositionUpdate(void);
    void SetTimeOrbitPeriod(double _update);
    double GetTimeOrbitPeriod(void);
    bool GetAttachProcedure(CartesianCoordinates* _uePos);
    double GetElAngle(CartesianCoordinates* _uePos);

  void
  UpdatePosition (double time);
    
private:
    int m_numSatellitePerOrbit;
    double m_gNBtimePositionUpdate;
    double timeOrbitPeriod;
};

#endif /* SATELLITE_H_ */
