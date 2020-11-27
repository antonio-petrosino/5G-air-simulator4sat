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

#include "SatelliteMovement.h"
#include "../componentManagers/NetworkManager.h"
#include "../device/Gateway.h"
#include "../device/GNodeB.h"
#include "../device/UserEquipment.h"
#include "../load-parameters.h"
//#include "../core/cartesianCoodrdinates/CartesianCoordinates.h"
#include "../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"


SatelliteMovement::SatelliteMovement(int nSat)
{
  SetMobilityModel(Mobility::SATELLITE);
  SetSpeed (7059.22);
  SetSpeedDirection (0.0);
  SetPositionLastUpdate (0.0);
  SetHandover (false);
  SetLastHandoverTime (0.0);
  SetAbsolutePosition (nullptr);
  SetTimePositionUpdate(0.05);
  SetNumberOfSatellitePerOrbit(nSat);
  SetTimeOrbitPeriod(5676.98); // about 94 min
  SetAntennaType(SatelliteMovement::PARABOLIC_REFLECTOR);
  cout << "Generating satellite with the following parameters:"
	   << "\n\t Speed: " << GetSpeed() << " meter/second"
	   << "\n\t Number of satellite per Orbit: " << GetNumberOfSatellitePerOrbit()
	   << "\n\t Elapsed Time between 2 sat: " << GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit() << " seconds"
	   << "\n ...Done!" << endl;
  Simulator::Init()->Schedule(m_gNBtimePositionUpdate,
                                &SatelliteMovement::UpdatePosition,
                                this,
                                Simulator::Init ()->Now());

}

SatelliteMovement::~SatelliteMovement()
{
  DeleteAbsolutePosition ();
}

void
SatelliteMovement::UpdatePosition (double time)
{
	//cout << "...t = "<<time <<" --> "<<endl;
DEBUG_LOG_START_1(SIM_ENV_MOBILITY_DEBUG)
    cout << "\t START MOBILITY MODEL for "<< GetDevice ()->GetIDNetworkNode() << endl;
	cout << "UpdatePosition gNB satellitare avviato."<<endl;
DEBUG_LOG_END

  double timeInterval = time - GetPositionLastUpdate ();

  //if(timeInterval > 0.1 || time == 0.0){

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

	double x_pos = GetSatPosition(time);
    // è espresso in metri al secondo se la velocita è data in km/h

	CartesianCoordinates *newPosition =
	new CartesianCoordinates(x_pos,
						   GetAbsolutePosition()->GetCoordinateY(),
						   GetAbsolutePosition()->GetCoordinateZ());
	newPosition->SetFloorHeight( GetAbsolutePosition()->GetFloorHeight() );

	SetAbsolutePosition(newPosition);
	SetPositionLastUpdate (time);

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

	Simulator::Init()->Schedule(m_gNBtimePositionUpdate,
                              &SatelliteMovement::UpdatePosition,
                              this,
                              Simulator::Init ()->Now());
  	delete newPosition;
  //}
}

int
SatelliteMovement::GetNumberOfSatellitePerOrbit(void){
    return m_numSatellitePerOrbit;
}

void
SatelliteMovement::SetNumberOfSatellitePerOrbit(int nSat){
    m_numSatellitePerOrbit = nSat;
}

double
SatelliteMovement::GetVisibilityPeriod()
{
	if (GetNumberOfSatellitePerOrbit() > 0){

		return GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit();
	}
	else{
		cout << "ERROR: Number of satellite < 0. " << endl;
		return 0.0;
	}

}

double
SatelliteMovement::GetTimeOrbitPeriod(){
    return m_timeOrbitPeriod;
}

void
SatelliteMovement::SetTimeOrbitPeriod(double _period){
	m_timeOrbitPeriod =  _period;
}


void
SatelliteMovement::SetTimePositionUpdate(double _time){
	m_gNBtimePositionUpdate = _time;
}

double
SatelliteMovement::GetTimePositionUpdate(void){
	return m_gNBtimePositionUpdate;
}

double
SatelliteMovement::GetSatPosition (double time)
{
    if(GetNumberOfSatellitePerOrbit()>0){
        // metà_raggio_satellite [COSTANTE] - radius [INPUT]+ (velocità_relativa [calcolata in base ai 2 minuti] * tempo_visibilità_satellite_[modulo3000]])
        // metri, metri, m/s, secondi
        // modello basato su:
        // area                 = 30 ettari
        // altezza satellite     = 500km
        // tempo visibilità        = 111 secondi
        // periodicità sat.        = 1 ogni 2838 secondi

        double mod =  GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit();
        double start_offset = 10000;
        double newPosition = -320000 -309 +(7059.22 * (fmod(time,mod))) - start_offset;
        //start_offset = 0;

        return newPosition;
    }else
    {
        cout <<"Number of satellite per orbit < 0." << endl;
        return 0;
    }
}

bool
SatelliteMovement::GetAttachProcedure(CartesianCoordinates* uePos){
	CartesianCoordinates* gnbPos = GetAbsolutePosition();

	double distance = uePos->GetDistance3D (gnbPos);
	//double maxSatelliteRange = GetTargetNode ()-> GetPhy ()->GetmaxSatelliteRange ();

	//double ElAngle = uePos->GetElAngle(gnbPos);
	double ElAngle = GetElAngle(uePos);
	double snr4attachProbability = GetSNRfromElAngle_SAT(ElAngle, 2, GetAntennaType()); //snr for downlink

	double measuredRSRP = snr4attachProbability + GetTermalNoisePowerDB();

	// if SNR < Soglia
	// return false;

	// Qui il discorso dovrebbe essere relativo all'MCL che in NB-IoT è 164 al massimo
	// 1 -> 144 dB
	// 2 -> 154 dB
	// 3 -> 164 dB
	// MCL = Ptx (33 dBm or 3 dB) - Prx [dB]
	// MCL = 3 dB - measuredRSRP
	double measuredCL = 3 - measuredRSRP;

	if(measuredCL > GetMCLthreshold()){
		return false;
	}

	double attachProbability = GetCellSelectionProb(snr4attachProbability);

	//std::uniform_int_distribution<> rndGen (0, 100);
	//extern std::mt19937 commonGen;
	//double randomNumber = rndGen(commonGen) / 100.;
	double randomNumber = (rand () %100 ) / 100.;

	bool _attach = false;

	if(randomNumber < attachProbability){
		_attach = true;
	}
	return _attach;
}

double
SatelliteMovement::GetElAngle(CartesianCoordinates *remoteObject)
{
	CartesianCoordinates* gnbPos = GetAbsolutePosition();

	double distance = sqrt (pow ((gnbPos->GetCoordinateX() - remoteObject->GetCoordinateX()),2) +
	          pow ( (gnbPos->GetCoordinateY() - remoteObject->GetCoordinateY()),2));

	//double satHeight = remoteObject->GetCoordinateZ();
	double satHeight = gnbPos->GetCoordinateZ();
	double elangle = 0.0;

	if(distance > 0){
		elangle = atan(satHeight / distance)* 180 / M_PI; // satHeight dovrebbe essere 500000 controllare
	}else if(distance == 0){
		elangle = 90;
	}
	// 2203970
	//cout << "Angolo elevazione calcolato: " << elangle << " - visibilità da 55° a 90°."<< endl;
	return elangle;

}

void
SatelliteMovement::SetMCLthreshold(double SNRthreshold){
	m_SNRthreshold = SNRthreshold;
}
double
SatelliteMovement::GetMCLthreshold(void){
	return m_SNRthreshold;
}

SatelliteMovement::AntennaType
SatelliteMovement::GetAntennaType(void) const{
	return m_AntennaType;
}
void
SatelliteMovement::SetAntennaType(AntennaType model){
	m_AntennaType = model;
}
