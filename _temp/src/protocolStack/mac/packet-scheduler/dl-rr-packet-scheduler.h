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
 * Author: Alessandro Grassi <alessandro.grassi@poliba.it>
 */


#ifndef DLRRPACKETSCHEDULER_H_
#define DLRRPACKETSCHEDULER_H_

#include "downlink-packet-scheduler.h"

class DL_RR_PacketScheduler : public DownlinkPacketScheduler
{
public:
  DL_RR_PacketScheduler();
  virtual ~DL_RR_PacketScheduler();

  virtual double ComputeSchedulingMetric (RadioBearer *bearer, double spectralEfficiency, int subChannel);

private:
  int m_selectedFlowIndex;
  double m_lastUpdate;
};

#endif /* DLRRPACKETSCHEDULER_H_ */
