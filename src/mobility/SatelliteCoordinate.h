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

double
GetSatPosition (double time)
{
	// metà_raggio_satellite [COSTANTE] - radius [INPUT]+ (velocità_relativa [calcolata in base ai 2 minuti] * tempo_visibilità_satellite_[modulo3000]])
	// metri, metri, m/s, secondi

	// modello basato su:
	// area 				= 30 ettari
	// altezza satellite 	= 500km
	// tempo visibilità		= 120 secondi
	// periodicità sat.		= 1 ogni 48 min

	double mod = 3000.0; // dipende dal tempo di visibilità + prossimo passaggio satellite
	double newPosition = 0.0;
	double start_offset = 100000;
	newPosition = -300000 -309 +(5000 * (fmod(time,mod))) - start_offset;
	// da un bordo cella all'altro ci mette 125 ms circa
	return newPosition;
};

#endif /* SATELLITE_COORDINATE_H_ */
