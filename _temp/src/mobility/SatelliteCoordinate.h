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

//
//static int mat_cols = 2;
//static int mat_rows = 60;
//static double sat_position[60][2] =
//{
//// time, position_x
//   {0, 0},   /*  initializers for row indexed by 0 */
//   {0.5, 5},   /*  initializers for row indexed by 1 */
//   {1, 11},   /*  initializers for row indexed by 2 */
//   {1.5, 11},
////   {2, 11},
////   {2.5, 11},
////   {3, 11},
////   {3.5, 11},
////   {4, 11},
////   {4.5, 11},
////   {5, 11},
////   {5.5, 11},
////   {6, 11},
////   {6.5, 11},
////   {7, 11},
////   {7.5, 11},
////   {8, 11},
////   {8.5, 11},
////   {9, 11},
////   {9.5, 11},
////   {10, 11},
////   {10.5, 11},
////   {11, 11},
////   {11.5, 11},
////   {12, 11},
////   {12.5, 11},
////   {13, 11},
////   {13.5, 11},
////   {14, 11},
////   {14.5, 11},
////   {15, 11},
////   {15.5, 11},
////   {16, 11},
////   {16.5, 11},
////   {17, 11},
////   {17.5, 11},
////   {18, 11},
////   {18.5, 11},
////   {19, 11},
////   {19.5, 11},
////   {20, 11},
////   {21, 11},
////   {21.5, 11},
////   {22, 11},
////   {22.5, 11},
////   {23, 11},
////   {23.5, 11},
////   {24, 11},
////   {24.5, 11},
////   {25, 11},
////   {25.5, 11},
////   {26, 11},
////   {26.5, 11},
////   {27, 11},
////   {27.5, 11},
////   {28, 11},
////   {28.5, 11},
////   {29, 11},
////   {99, 9999999},
////   {99999,9999999}
//};



double
GetSatPosition (double time)
{
	// metà_raggio_satellite [COSTANTE] - radius [INPUT]+ (velocità_relativa [calcolata in base ai 2 minuti] * tempo_visibilità_satellite_[modulo3000]])
	// metri, metri, m/s, secondi

	// modello basato su:
	// area 				= 30 ettari
	// altezza satellite 	= 500km
	// tempo visibilità		= 120 secondi

	double mod = 3000.0;
	double newPosition = 0.0;

	newPosition = -300000 -309 +(5000 * (fmod(time,mod)));
	//cout <<"newPosition = " << newPosition << endl;
	//cout <<"Modulo da verificare: " << fmod(time,mod)<<endl << "time: "<< time << "mod: " << mod << endl;
	if(fmod(time,mod) > 120 && fmod(time,mod) < 3000){
		newPosition = 9999999;
		//cout << "gNB fuori visibilità." << endl;
	}

	return newPosition;

	// vecchio approccio con interpolazione da tabella
	//double newPosition = 0.0, x_1 = 0.0, t_1 =  0.0, x_2 = 0.0, t_2 = 0.0;

	//[X_1,Y_1,Z_1] = GetPosition (time_0); // t = 0 sec X = 5
	//[X_2,Y_2,Z_2] = GetPosition (time_1); // t = 1 sec X = 37
	// t_x = 0.75 // sec

	// X_I = (X_2 - X_1) * (t_x) / (time_1 - time_0)
	// Y_I = Y_1 = Y_2
	// Z_I = Z_1 = Z_2

	// come interpolare fra 2 posizioni?


//	for(int i=0;i<=mat_rows;i++){
//		//for(int j=0;j<=mat_cols;j++){
//
//			if(sat_position[i][0] >= time){
//				//interpolazione solo su x
//				x_1 = sat_position[i-1][1];
//				t_1 = sat_position[i-1][0];
//
//				x_2 = sat_position[i][1];
//				t_2 = sat_position[i][0];
//
//				newPosition = x_1 + (((x_2 - x_1) * (time-t_1)) / (t_2 - t_1));
//
//				return newPosition;
//			}
//
//		//}
//
//	}
};


#endif /* SATELLITE_COORDINATE_H_ */
