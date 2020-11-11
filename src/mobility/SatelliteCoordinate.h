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

#ifndef SATELLITE_COORDINATE_H_
#define SATELLITE_COORDINATE_H_

#include "iostream"
#include <cmath>

//double m_period = 2838.0;
//double m_visibility_time = 180.0;


static double
GetPeriod(){
	return 2838.0;
};

static double
GetVisibilityTime(){
	return 300.0;
};


static double
GetSatPosition (double time, int nSat)
{
	if(nSat>0){
		// metà_raggio_satellite [COSTANTE] - radius [INPUT]+ (velocità_relativa [calcolata in base ai 2 minuti] * tempo_visibilità_satellite_[modulo3000]])
		// metri, metri, m/s, secondi
		// modello basato su:
		// area 				= 30 ettari
		// altezza satellite 	= 500km
		// tempo visibilità		= 111 secondi
		// periodicità sat.		= 1 ogni 2838 secondi

		//double mod = 2830.0;
		double period = 5676.98;
		double mod =  period / nSat;
		//cout <<"Number of satellite per orbit: " << nSat<< endl;

		double newPosition = 0.0;
		double start_offset = 10000;
		//start_offset = 0;
		newPosition = -320000 -309 +(7059.22 * (fmod(time,mod))) - start_offset;

		return newPosition;
	}else
	{
		cout <<"Number of satellite per orbit < 0." << endl;
		return 0;
	}
};



#endif /* SATELLITE_COORDINATE_H_ */
