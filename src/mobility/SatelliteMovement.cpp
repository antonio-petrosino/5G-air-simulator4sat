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
  //SetAntennaType(SatelliteMovement::PARABOLIC_REFLECTOR);
  SetAntennaType(SatelliteMovement::PATCH_ANTENNA);
  //SetFixedAreaRadius(0);

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
        double startOffset = 700000;
        //double newPosition = -320000 - 309  +(7059.22 * (fmod(time,mod))) - start_offset;

        double newPosition = - GetSpotBeamRadius() - GetFixedAreaRadius() +  (7059.22 * (fmod(time,mod))) - startOffset;
        //start_offset = 0;

        return newPosition;
    }else
    {
        cout <<"Number of satellite per orbit < 1." << endl;
        exit(1);
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

	double txPower = GetDevice()->GetPhy()->GetTxPower();
	double measuredCL =  txPower - 30 - measuredRSRP;

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

double
SatelliteMovement::GetSatPositionFromElAngle(CartesianCoordinates *remoteObject, double elangle)
{
    CartesianCoordinates* gnbPos = GetAbsolutePosition();
    double satHeight = gnbPos->GetCoordinateZ();
    // formula inversa
    // elangle = atan(satHeight / distance)* 180 / M_PI;
    double distance = (satHeight) / (tan(elangle * M_PI / 180));
    //double satPosition = GetRightPosition(1, -2*(remoteObject->GetCoordinateX()), -((pow(distance,2)-pow((satHeight - remoteObject->GetCoordinateY()),2)-pow(remoteObject->GetCoordinateX(),2))));

    double root = sqrt(pow(distance,2) - pow(gnbPos->GetCoordinateY() - remoteObject->GetCoordinateY(),2));

    double satPosition = remoteObject->GetCoordinateX() - root;
    double sat2Position = remoteObject->GetCoordinateX() + root;

    return satPosition;
}

double
SatelliteMovement::GetTimeNeededForDestination(double satPosition)
{
    //BISOGNA PARTIRE DA QUESTA E TIRARE FUORI IL TIME!
    //double newPosition = - GetSpotBeamRadius() - GetFixedAreaRadius() +  (7059.22 * (fmod(time,mod))) - start_offset;
    double time = 0.0;
    double mod = GetTimeOrbitPeriod() / GetNumberOfSatellitePerOrbit();

    double gnbPosX = GetAbsolutePosition()->GetCoordinateX();
    double gnbPosXTarget = satPosition;
    double startOffset = 700000;
    double speed = 7059.22;
    
    double sMin = - GetSpotBeamRadius() - GetFixedAreaRadius() +  (speed * (fmod(0,mod))) - startOffset;
    double sMax = - GetSpotBeamRadius() - GetFixedAreaRadius() +  (speed * (fmod(mod-0.0001,mod))) - startOffset;
    
    double space;
    if(gnbPosX >= gnbPosXTarget){
        space = (sMax - gnbPosX) + (gnbPosXTarget - sMin);
    }
    else {
        space = gnbPosXTarget - gnbPosX;
    }
    
    time = space/speed;

    return time;
}

double
SatelliteMovement::GetNextTimePositionUpdate(CartesianCoordinates *uePos)
{
    double t;
    double ElAngle = GetElAngle(uePos);
    double minSNR = GetDevice()->GetPhy()->GetTxPower() - 30 - GetMCLthreshold() - GetTermalNoisePowerDB();
    double MinElAngle = GetMinElAngleFromSNR (minSNR, 2, GetAntennaType());

    if(ElAngle < MinElAngle){
    	t = GetTimeNeededForDestination(GetSatPositionFromElAngle(uePos, MinElAngle));

        if(t < 0.05)
        	t = 0.05;
    }else{

    	t = 0.05;
    }

    return t;
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

void
SatelliteMovement::SetFixedAreaRadius(double radius){
	 m_fixedAreaRadius = radius;
	 cout <<"Fixed area radius: " << m_fixedAreaRadius << " meters"<< endl;
}

double
SatelliteMovement::GetFixedAreaRadius(void){
	return m_fixedAreaRadius;
}

double
SatelliteMovement::GetSpotBeamRadius(void){

	if(m_AntennaType == SatelliteMovement::PARABOLIC_REFLECTOR){
		m_spotBeamRadius = 130000.0;

	}else if(m_AntennaType == SatelliteMovement::PATCH_ANTENNA){
		m_spotBeamRadius = 320000.0;
	}

	return m_spotBeamRadius;
}
