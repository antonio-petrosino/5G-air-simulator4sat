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



#ifndef BLERVSSINR_NBIoT_SAT_H
#define BLERVSSINR_NBIoT_SAT_H

#include "../../componentManagers/FrameManager.h"
#include "../../protocolStack/mac/AMCModule.h"
#include "../../load-parameters.h"

#include "iostream"

// i valori non rispecchiano il nostro scenario
static double BLER_NBIoT_SAT [24][21] =
{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // iMCS = 0, NRep  = 1;
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
		{1, 1, 1, 1, 1, 1, 1, 0.9, 0.3, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 , 0.0 }, // 3
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // iMCS = 0, NRep  = 2;
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
		{1, 1, 1, 1, 1, 0.7, 0.15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // iMCS = 0, NRep  = 4;
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
		{1, 1, 1, 0.7, 0.15, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // 3
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7

};

// i valori non rispecchiano il nostro scenario
static double SINR_NBIoT_SAT [24][21] =
{
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // iMCS = 0, NRep  = 1;
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
		{-10.0, -9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0 }, // 3
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // iMCS = 0, NRep  = 2;
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
		{-10.0, -9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0}, // 3
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // iMCS = 0, NRep  = 4;
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 1
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 2
		{-10.0, -9.0, -8.0, -7.0, -6.0, -5.0, -4.0, -3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0}, // 3
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 4
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 5
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 6
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, // 7

};

static double LOSS_NBIoT_SAT[584] =

	{   -22.300648, -22.016501, -21.740108, -21.471049, -21.208941, -20.953430, -20.704190, -20.460920, -20.223341, -19.991195, -19.764242,
		-19.542257, -19.325032, -19.112372, -18.904095, -18.700030, -18.500017, -18.303907, -18.111557, -17.922835, -17.737616, -17.555780,
		-17.377217, -17.201821, -17.029492, -16.860136, -16.693662, -16.529985, -16.369026, -16.210706, -16.054953, -15.901697, -15.750873,
		-15.602417, -15.456268, -15.312370, -15.170667, -15.031107, -14.893641, -14.758219, -14.624797, -14.493329, -14.363775, -14.236094,
		-14.110247, -13.986198, -13.863909, -13.743348, -13.624482, -13.507279, -13.391708, -13.277742, -13.165351, -13.054508, -12.945189,
		-12.837367, -12.731019, -12.626122, -12.522654, -12.420593, -12.319918, -12.220610, -12.122649, -12.026017, -11.930697, -11.836671,
		-11.743922, -11.652434, -11.562193, -11.473183, -11.385391, -11.298802, -11.213403, -11.129181, -11.046124, -10.964221, -10.883459,
		-10.803829, -10.725318, -10.647918, -10.571618, -10.496408, -10.422281, -10.349226, -10.277237, -10.206304, -10.136420, -10.067578,
		-9.999770, -9.932990, -9.867232, -9.802489, -9.738755, -9.676025, -9.614294, -9.553556, -9.493806, -9.435041, -9.377255, -9.320445,
		-9.264607, -9.209738, -9.155833, -9.102891, -9.050908, -8.999882, -8.949810, -8.900690, -8.852520, -8.805299, -8.759025, -8.713697,
		-8.669313, -8.625873, -8.583377, -8.541824, -8.501213, -8.461545, -8.422820, -8.385039, -8.348202, -8.312310, -8.277365, -8.243367,
		-8.210318, -8.178221, -8.147076, -8.116887, -8.087655, -8.059384, -8.032076, -8.005735, -7.980363, -7.955965, -7.932545, -7.910106,
		-7.888653, -7.868191, -7.848724, -7.830258, -7.812798, -7.796349, -7.780919, -7.766513, -7.753137, -7.740799, -7.729506, -7.719265,
		-7.710085, -7.701972, -7.694936, -7.688986, -7.684131, -7.680380, -7.677744, -7.676232, -7.675856, -7.676627, -7.678556, -7.681655,
		-7.685937, -7.691415, -7.698103, -7.706013, -7.715162, -7.725563, -7.737233, -7.750188, -7.764444, -7.780019, -7.796931, -7.815199,
		-7.834842, -7.855881, -7.878336, -7.902230, -7.927584, -7.954424, -7.982772, -8.012654, -8.044097, -8.077129, -8.111777, -8.148071,
		-8.186043, -8.225724, -8.267147, -8.310348, -8.355364, -8.402230, -8.450988, -8.501678, -8.554344, -8.609030, -8.665782, -8.724651,
		-8.785686, -8.848942, -8.914476, -8.982344, -9.052610, -9.125338, -9.200595, -9.278454, -9.358988, -9.442276, -9.528402, -9.617452,
		-9.709520, -9.804701, -9.903099, -10.004822, -10.109985, -10.218710, -10.331126, -10.447370, -10.567587, -10.691933, -10.820572,
		-10.953682, -11.091450, -11.234077, -11.381781, -11.534793, -11.693361, -11.857755, -12.028264, -12.205202, -12.388908, -12.579750,
		-12.778129, -12.984483, -13.199290, -13.423074, -13.656413, -13.899944, -14.154370, -14.420475, -14.699131, -14.991317, -15.298130,
		-15.620813, -15.960777, -16.319633, -16.699233, -17.101721, -17.529595, -17.985793, -18.473801, -18.997797, -19.562850, -20.175189,
		-20.842593, -21.574945, -22.385068, -23.290025, -24.313219, -25.487980, -26.864129, -28.521035, -30.596892, -33.366586, -37.515834,
		-45.900963, -49.728393, -38.647221, -33.893705, -30.811618, -28.520382, -26.692149, -25.168379, -23.860214, -22.712809, -21.689927,
		-20.766364, -19.923884, -19.148863, -18.430856, -17.761678, -17.134796, -16.544912, -15.987669, -15.459443, -14.957186, -14.478308,
		-14.020593, -13.582129, -13.161254, -12.756516, -12.366635, -11.990483, -11.627055, -11.275457, -10.934885, -10.604617, -10.284002,
		-9.972448, -9.669419, -9.374425, -9.087020, -8.806792, -8.533367, -8.266396, -8.005560, -7.750563, -7.501130, -7.257008, -7.017958,
		-6.783761, -6.554210, -6.329113, -6.108290, -5.891571, -5.678798, -5.469823, -5.264504, -5.062709, -4.864315, -4.669202, -4.477261,
		-4.288384, -4.102473, -3.919433, -3.739174, -3.561611, -3.386661, -3.214249, -3.044301, -2.876746, -2.711518, -2.548552, -2.387788,
		-2.229167, -2.072634, -1.918135, -1.765620, -1.615038, -1.466343, -1.319491, -1.174437, -1.031140, -0.889561, -0.749660, -0.611400,
		-0.474747, -0.339666, -0.206123, -0.074088, 0.056471, 0.185584, 0.313278, 0.439583, 0.564523, 0.688125, 0.810414, 0.931414, 1.051149,
		1.169640, 1.286910, 1.402980, 1.517870, 1.631600, 1.744190, 1.855658, 1.966022, 2.075300, 2.183508, 2.290664, 2.396784, 2.501883,
		2.605976, 2.709078, 2.811203, 2.912366, 3.012579, 3.111857, 3.210212, 3.307656, 3.404202, 3.499862, 3.594646, 3.688567, 3.781635,
		3.873862, 3.965256, 4.055829, 4.145591, 4.234551, 4.322719, 4.410103, 4.496714, 4.582559, 4.667647, 4.751986, 4.835586, 4.918453,
		5.000596, 5.082022, 5.162738, 5.242752, 5.322072, 5.400703, 5.478654, 5.555929, 5.632537, 5.708483, 5.783774, 5.858415, 5.932414,
		6.005774, 6.078504, 6.150607, 6.222089, 6.292957, 6.363214, 6.432867, 6.501920, 6.570378, 6.638247, 6.705530, 6.772233, 6.838361,
		6.903917, 6.968906, 7.033332, 7.097201, 7.160515, 7.223278, 7.285496, 7.347172, 7.408309, 7.468912, 7.528984, 7.588529, 7.647550,
		7.706051, 7.764035, 7.821506, 7.878467, 7.934921, 7.990872, 8.046323, 8.101276, 8.155735, 8.209702, 8.263182, 8.316176, 8.368687,
		8.420718, 8.472272, 8.523352, 8.573960, 8.624098, 8.673770, 8.722978, 8.771724, 8.820010, 8.867840, 8.915215, 8.962137, 9.008610,
		9.054634, 9.100213, 9.145349, 9.190042, 9.234297, 9.278114, 9.321496, 9.364444, 9.406961, 9.449048, 9.490708, 9.531941, 9.572751,
		9.613138, 9.653105, 9.692653, 9.731784, 9.770500, 9.808801, 9.846691, 9.884170, 9.921239, 9.957902, 9.994158, 10.030009, 10.065458,
		10.100505, 10.135151, 10.169399, 10.203249, 10.236702, 10.269761, 10.302426, 10.334699, 10.366581, 10.398073, 10.429176, 10.459892,
		10.490221, 10.520166, 10.549726, 10.578903, 10.607698, 10.636113, 10.664148, 10.691804, 10.719082, 10.745984, 10.772509, 10.798660,
		10.824438, 10.849842, 10.874874, 10.899535, 10.923826, 10.947747, 10.971300, 10.994485, 11.017303, 11.039755, 11.061841, 11.083563,
		11.104921, 11.125916, 11.146548, 11.166819, 11.186728, 11.206278, 11.225467, 11.244298, 11.262770, 11.280884, 11.298641, 11.316041,
		11.333085, 11.349774, 11.366108, 11.382087, 11.397713, 11.412985, 11.427905, 11.442471, 11.456686, 11.470550, 11.484063, 11.497225,
		11.510036, 11.522498, 11.534611, 11.546375, 11.557790, 11.568857, 11.579576, 11.589948, 11.599972, 11.609650, 11.618981, 11.627965,
		11.636604, 11.644897, 11.652844, 11.660446, 11.667703, 11.674616, 11.681184, 11.687407, 11.693287, 11.698822, 11.704014, 11.708862,
		11.713366, 11.717527, 11.721345, 11.724820, 11.727951, 11.730740, 11.733186, 11.735290, 11.737050, 11.738468, 11.739543, 11.740276,
		11.740668 };


static double ELANGLE4LOSS_NBIoT_SAT[584] =

	{ 55.000000, 55.060000, 55.120000, 55.180000, 55.240000, 55.300000, 55.360000, 55.420000, 55.480000, 55.540000, 55.600000,
			55.660000, 55.720000, 55.780000, 55.840000, 55.900000, 55.960000, 56.020000, 56.080000, 56.140000, 56.200000,
			56.260000, 56.320000, 56.380000, 56.440000, 56.500000, 56.560000, 56.620000, 56.680000, 56.740000, 56.800000,
			56.860000, 56.920000, 56.980000, 57.040000, 57.100000, 57.160000, 57.220000, 57.280000, 57.340000, 57.400000,
			57.460000, 57.520000, 57.580000, 57.640000, 57.700000, 57.760000, 57.820000, 57.880000, 57.940000, 58.000000,
			58.060000, 58.120000, 58.180000, 58.240000, 58.300000, 58.360000, 58.420000, 58.480000, 58.540000, 58.600000,
			58.660000, 58.720000, 58.780000, 58.840000, 58.900000, 58.960000, 59.020000, 59.080000, 59.140000, 59.200000,
			59.260000, 59.320000, 59.380000, 59.440000, 59.500000, 59.560000, 59.620000, 59.680000, 59.740000, 59.800000,
			59.860000, 59.920000, 59.980000, 60.040000, 60.100000, 60.160000, 60.220000, 60.280000, 60.340000, 60.400000,
			60.460000, 60.520000, 60.580000, 60.640000, 60.700000, 60.760000, 60.820000, 60.880000, 60.940000, 61.000000,
			61.060000, 61.120000, 61.180000, 61.240000, 61.300000, 61.360000, 61.420000, 61.480000, 61.540000, 61.600000,
			61.660000, 61.720000, 61.780000, 61.840000, 61.900000, 61.960000, 62.020000, 62.080000, 62.140000, 62.200000,
			62.260000, 62.320000, 62.380000, 62.440000, 62.500000, 62.560000, 62.620000, 62.680000, 62.740000, 62.800000,
			62.860000, 62.920000, 62.980000, 63.040000, 63.100000, 63.160000, 63.220000, 63.280000, 63.340000, 63.400000,
			63.460000, 63.520000, 63.580000, 63.640000, 63.700000, 63.760000, 63.820000, 63.880000, 63.940000, 64.000000,
			64.060000, 64.120000, 64.180000, 64.240000, 64.300000, 64.360000, 64.420000, 64.480000, 64.540000, 64.600000,
			64.660000, 64.720000, 64.780000, 64.840000, 64.900000, 64.960000, 65.020000, 65.080000, 65.140000, 65.200000,
			65.260000, 65.320000, 65.380000, 65.440000, 65.500000, 65.560000, 65.620000, 65.680000, 65.740000, 65.800000,
			65.860000, 65.920000, 65.980000, 66.040000, 66.100000, 66.160000, 66.220000, 66.280000, 66.340000, 66.400000,
			66.460000, 66.520000, 66.580000, 66.640000, 66.700000, 66.760000, 66.820000, 66.880000, 66.940000, 67.000000,
			67.060000, 67.120000, 67.180000, 67.240000, 67.300000, 67.360000, 67.420000, 67.480000, 67.540000, 67.600000,
			67.660000, 67.720000, 67.780000, 67.840000, 67.900000, 67.960000, 68.020000, 68.080000, 68.140000, 68.200000,
			68.260000, 68.320000, 68.380000, 68.440000, 68.500000, 68.560000, 68.620000, 68.680000, 68.740000, 68.800000,
			68.860000, 68.920000, 68.980000, 69.040000, 69.100000, 69.160000, 69.220000, 69.280000, 69.340000, 69.400000,
			69.460000, 69.520000, 69.580000, 69.640000, 69.700000, 69.760000, 69.820000, 69.880000, 69.940000, 70.000000,
			70.060000, 70.120000, 70.180000, 70.240000, 70.300000, 70.360000, 70.420000, 70.480000, 70.540000, 70.600000,
			70.660000, 70.720000, 70.780000, 70.840000, 70.900000, 70.960000, 71.020000, 71.080000, 71.140000, 71.200000,
			71.260000, 71.320000, 71.380000, 71.440000, 71.500000, 71.560000, 71.620000, 71.680000, 71.740000, 71.800000,
			71.860000, 71.920000, 71.980000, 72.040000, 72.100000, 72.160000, 72.220000, 72.280000, 72.340000, 72.400000,
			72.460000, 72.520000, 72.580000, 72.640000, 72.700000, 72.760000, 72.820000, 72.880000, 72.940000, 73.000000,
			73.060000, 73.120000, 73.180000, 73.240000, 73.300000, 73.360000, 73.420000, 73.480000, 73.540000, 73.600000,
			73.660000, 73.720000, 73.780000, 73.840000, 73.900000, 73.960000, 74.020000, 74.080000, 74.140000, 74.200000,
			74.260000, 74.320000, 74.380000, 74.440000, 74.500000, 74.560000, 74.620000, 74.680000, 74.740000, 74.800000,
			74.860000, 74.920000, 74.980000, 75.040000, 75.100000, 75.160000, 75.220000, 75.280000, 75.340000, 75.400000,
			75.460000, 75.520000, 75.580000, 75.640000, 75.700000, 75.760000, 75.820000, 75.880000, 75.940000, 76.000000,
			76.060000, 76.120000, 76.180000, 76.240000, 76.300000, 76.360000, 76.420000, 76.480000, 76.540000, 76.600000,
			76.660000, 76.720000, 76.780000, 76.840000, 76.900000, 76.960000, 77.020000, 77.080000, 77.140000, 77.200000,
			77.260000, 77.320000, 77.380000, 77.440000, 77.500000, 77.560000, 77.620000, 77.680000, 77.740000, 77.800000,
			77.860000, 77.920000, 77.980000, 78.040000, 78.100000, 78.160000, 78.220000, 78.280000, 78.340000, 78.400000,
			78.460000, 78.520000, 78.580000, 78.640000, 78.700000, 78.760000, 78.820000, 78.880000, 78.940000, 79.000000,
			79.060000, 79.120000, 79.180000, 79.240000, 79.300000, 79.360000, 79.420000, 79.480000, 79.540000, 79.600000,
			79.660000, 79.720000, 79.780000, 79.840000, 79.900000, 79.960000, 80.020000, 80.080000, 80.140000, 80.200000,
			80.260000, 80.320000, 80.380000, 80.440000, 80.500000, 80.560000, 80.620000, 80.680000, 80.740000, 80.800000,
			80.860000, 80.920000, 80.980000, 81.040000, 81.100000, 81.160000, 81.220000, 81.280000, 81.340000, 81.400000,
			81.460000, 81.520000, 81.580000, 81.640000, 81.700000, 81.760000, 81.820000, 81.880000, 81.940000, 82.000000,
			82.060000, 82.120000, 82.180000, 82.240000, 82.300000, 82.360000, 82.420000, 82.480000, 82.540000, 82.600000,
			82.660000, 82.720000, 82.780000, 82.840000, 82.900000, 82.960000, 83.020000, 83.080000, 83.140000, 83.200000,
			83.260000, 83.320000, 83.380000, 83.440000, 83.500000, 83.560000, 83.620000, 83.680000, 83.740000, 83.800000,
			83.860000, 83.920000, 83.980000, 84.040000, 84.100000, 84.160000, 84.220000, 84.280000, 84.340000, 84.400000,
			84.460000, 84.520000, 84.580000, 84.640000, 84.700000, 84.760000, 84.820000, 84.880000, 84.940000, 85.000000,
			85.060000, 85.120000, 85.180000, 85.240000, 85.300000, 85.360000, 85.420000, 85.480000, 85.540000, 85.600000,
			85.660000, 85.720000, 85.780000, 85.840000, 85.900000, 85.960000, 86.020000, 86.080000, 86.140000, 86.200000,
			86.260000, 86.320000, 86.380000, 86.440000, 86.500000, 86.560000, 86.620000, 86.680000, 86.740000, 86.800000,
			86.860000, 86.920000, 86.980000, 87.040000, 87.100000, 87.160000, 87.220000, 87.280000, 87.340000, 87.400000,
			87.460000, 87.520000, 87.580000, 87.640000, 87.700000, 87.760000, 87.820000, 87.880000, 87.940000, 88.000000,
			88.060000, 88.120000, 88.180000, 88.240000, 88.300000, 88.360000, 88.420000, 88.480000, 88.540000, 88.600000,
			88.660000, 88.720000, 88.780000, 88.840000, 88.900000, 88.960000, 89.020000, 89.080000, 89.140000, 89.200000,
			89.260000, 89.320000, 89.380000, 89.440000, 89.500000, 89.560000, 89.620000, 89.680000, 89.740000, 89.800000,
			89.860000, 89.920000, 89.980000 };


static double
GetBLER_SAT (double SINR, int MCS)
{
  int index = -1;
  double BLER = 0.0;
  int MCS_ = MCS;
  double R = 0.0;

  //cout << "SINR in input: " << SINR << "MCS: " << MCS << endl;

  int _NRep = FrameManager::Init()->GetNRep();

  if(_NRep == 2 ){

	  MCS_ = MCS_ + 8;

  }else if(_NRep == 4){

	  MCS_ = MCS_ + 16;

  }

  if ( SINR <= SINR_NBIoT_SAT [MCS_-1] [0] )
    {
      BLER = 1.0;
    }
  else if (SINR >= SINR_NBIoT_SAT [MCS_-1] [15])
    {
      BLER = 0.0;
    }
  else
    {
      for (int i=0; i<15; i++)
        {
          if (SINR >= SINR_NBIoT_SAT [MCS_-1] [i] && SINR < SINR_NBIoT_SAT [MCS_-1] [i+1])
            {
              index = i;
              //cout << SINR_15_CQI_TU [MCS_-1] [i] << " - " << SINR_15_CQI_TU [MCS_-1] [i+1] << endl;
            }
        }
    }

  if (index != -1)
    {
      R = (SINR - SINR_NBIoT_SAT [MCS_-1] [index]) / ( SINR_NBIoT_SAT [MCS_-1] [index + 1] - SINR_NBIoT_SAT [MCS_-1] [index] );

      BLER = BLER_NBIoT_SAT [MCS_-1] [index] + R * ( BLER_NBIoT_SAT [MCS_-1] [index + 1] - BLER_NBIoT_SAT [MCS_-1] [index] );
    }



  if (BLER >= 0.1)
    {
DEBUG_LOG_START_1(SIM_ENV_BLER_DEBUG)
      cout << "SINR " << SINR << " "
                << "CQI " << MCS_ << " "
                << "SINRprec " << SINR_NBIoT_SAT [MCS_-1] [index] << " "
                << "SINRsucc " << SINR_NBIoT_SAT [MCS_-1] [index+1] << " "
                << "BLERprec " << BLER_NBIoT_SAT [MCS_-1] [index] << " "
                << "BLERsucc " << BLER_NBIoT_SAT [MCS_-1] [index + 1] << " "
                << "R " << R << " "
                << "BLER " << BLER << " "
                << endl;
DEBUG_LOG_END
    }
  //cout << "BLER: " << BLER<<endl;
  return BLER;
}

static double
GetLOSS_SAT (double elangle)
{
  int index = -1;
  double R = 0.0;
  double LOSS = 0.0;

  if ( elangle <= ELANGLE4LOSS_NBIoT_SAT [0] )
    {
      LOSS = 999999.0; // comunicazione impossibile
    }
  else if (elangle >= ELANGLE4LOSS_NBIoT_SAT [583])
    {
	  LOSS = ELANGLE4LOSS_NBIoT_SAT [583];
    }
  else
    {
      for (int i=0; i<583; i++)
        {
          if (elangle >= ELANGLE4LOSS_NBIoT_SAT [i] && elangle < ELANGLE4LOSS_NBIoT_SAT [i+1])
            {
              index = i;
            }
        }
    }

  if (index != -1)
    {
      R = (elangle - ELANGLE4LOSS_NBIoT_SAT [index]) / ( ELANGLE4LOSS_NBIoT_SAT [index + 1] - ELANGLE4LOSS_NBIoT_SAT [index] );

      LOSS = LOSS_NBIoT_SAT [index] + R * ( LOSS_NBIoT_SAT [index + 1] - LOSS_NBIoT_SAT [index] );
    }
  //cout<< "LOSS relativa: " << LOSS << endl;
  return LOSS;
}


#endif /* BLERVSSINR_NBIoT_SAT_H */
