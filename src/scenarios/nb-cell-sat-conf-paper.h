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
 
#include "../channel/RadioChannel.h"
#include "../phy/gnb-phy.h"
#include "../phy/ue-phy.h"
#include "../core/spectrum/bandwidth-manager.h"
#include "../networkTopology/Cell.h"
#include "../protocolStack/packet/packet-burst.h"
#include "../protocolStack/packet/Packet.h"
#include "../core/eventScheduler/simulator.h"
#include "../flows/application/InfiniteBuffer.h"
#include "../flows/application/VoIP.h"
#include "../flows/application/CBR.h"
#include "../flows/application/TraceBased.h"
#include "../device/IPClassifier/ClassifierParameters.h"
#include "../flows/QoS/QoSParameters.h"
#include "../flows/QoS/QoSForEXP.h"
#include "../flows/QoS/QoSForFLS.h"
#include "../flows/QoS/QoSForM_LWDF.h"
#include "../componentManagers/FrameManager.h"
#include "../utility/RandomVariable.h"
#include "../channel/propagation-model/channel-realization.h"
#include "../phy/wideband-cqi-eesm-error-model.h"
#include "../phy/nbiot-simple-error-model.h"
#include "../phy/BLERTrace/BLERvsSINR_NBIoT_SAT.h"
#include "../load-parameters.h"

#include "../device/UserEquipment.h"
#include "../device/NetworkNode.h"
#include "../device/NetworkNode.h"
#include "../device/GNodeB.h"
#include "../device/CqiManager/cqi-manager.h"
#include "../device/CqiManager/fullband-cqi-manager.h"
#include "../device/CqiManager/wideband-cqi-manager.h"
#include "../channel/propagation-model/propagation-loss-model.h"

#include "../protocolStack/mac/random-access/ue-random-access.h"
#include "../protocolStack/mac/random-access/gnb-random-access.h"
#include "../protocolStack/mac/random-access/enb-nb-iot-random-access.h"
#include "../protocolStack/mac/random-access/ue-nb-iot-random-access.h"
#include "../protocolStack/mac/nb-AMCModule.h"

#include <iostream>
#include <queue>
#include <fstream>
#include <stdlib.h>
#include <cstring>
#include <math.h>

static void nbCell_Satellite_Conf_Paper (int argc, char *argv[])
{
	// ./5G-air-simulator nbCell-Sat-Conf-Paper 2 1 3 4 128 1024 10
	//						[1]                [2-3-4-5] [6] [7] [8]


	std::map<double, int> ceProb;
    std::map<int, int> maxPreambleTx;
    std::map<int, int> preambleRep;
    std::map<int, int> rarWindow;
    std::map<int, int> nbPre;
    std::map<int, int> rachPeriod;
    std::map<int, int> rachOffset;
    std::map<int, int> boWindow;

    int nSatellitePerOrbit = atoi(argv[2]); // numero di satelliti per orbita
	int carriers = atoi(argv[3]);  // 1 carrier -> solo 48 preambolo
	int MCS = atoi(argv[4]);
	int NRep = atoi(argv[5]);
	int RAOPeriod = atoi(argv[6]);
	int backOffRACH = atoi(argv[7]);

	//string environment(argv[2]); // "suburban" or "rural"
	string environment("sat");
    //int schedUL = atoi(argv[4]);
	int schedUL = 1;
    //double dur = atoi(argv[5]); // [s]
	double dur = 86400.0; // [s] 86400 sec = 1 day
    //double radius = atof(argv[6]); // [km]
	double radius = 0.309; // [km]
    //int nbUE = atoi(argv[7]);
	int nbUE = 3000;
    //double bandwidth = atof(argv[8]); // [MHz] max 15MHz
	double bandwidth = 30; // [MHz] max 15MHz
    double spacing = 15; // 15 kHz or 3.75 kHz
    int tones = 1; // 1,3,12
	int NRU = 5; // sarà costante nel nostro scenario???
    //int CBR_interval = 21600; // scenario agricoltura 1 ogni 4/6 ore [s]
    int CBR_size = 19; // da D1 268 [byte] (2144 bit) che include gli header di alto livello
    int totPreambleTx = 10; // tentativi di RACH procedure
    int nbCE = 1; // numero coverage class

    // al varariare della coverage class
    for (int i=0; i<nbCE; i++)
    {
    //if (i==nbCE-1)
      ceProb.insert(std::make_pair(1, i));
    //else if (i==0)
      //ceProb.insert(std::make_pair(atof(argv[19+i])/100,i));
    //else
      //ceProb.insert(std::make_pair((atof(argv[19+i])/100) + (atof(argv[18+i])/100), i));

    maxPreambleTx.insert(std::make_pair(i, 10));
    preambleRep.insert(std::make_pair(i, 4));
    rarWindow.insert(std::make_pair(i, 8));
    nbPre.insert(std::make_pair(i, 48 * carriers)); // nP
    rachPeriod.insert(std::make_pair(i,RAOPeriod ));
    rachOffset.insert(std::make_pair(i, 12)); // dall inizio della visibilita del sat fino alla fine del cell search??
    boWindow.insert(std::make_pair(i, backOffRACH)); // backoff window larga
   }
    
int seed;
  if (argc==9)
    {
      seed = atoi(argv[8]);
    }
  else
    {
      seed = -1;
    }

  	cout << "Visualizza prob." << endl;
    for(auto it = ceProb.cbegin(); it != ceProb.cend(); ++it)
    {
        std::cout << it->first << " " << it->second << "\n";
    }    

    double carrierFreq; //MHz
    double txPower = 33; // dBm
    double antennaHeight = 500000; // m
    double antennaGain = 8; // 8 dBi
    double UENoiseFigure = 6; // dB
    
    int channelModel = 1;
    double BSNoiseFigure = 3; // dB
    bool handover = false;
    double BSFeederLoss = 0; // 2
    
    if(environment=="suburban")
    {        
        channelModel = 4;
        carrierFreq = 2000;
    }
    else if(environment=="rural")
    {        
        channelModel = 4;
        carrierFreq = 2000;
    }
    else if(environment == "sat"){
    	channelModel = 5;
    	carrierFreq = 2000;
    }
    else
    {
        cout << "ERROR: invalid environment selected" << endl;
    }
    
    // define channel model
    ChannelRealization::ChannelModel model;

    switch(channelModel)
    {
        case 0:
          model = ChannelRealization::CHANNEL_MODEL_MACROCELL_URBAN_IMT;
          break;
        case 1:
          model = ChannelRealization::CHANNEL_MODEL_MACROCELL_RURAL_IMT;
          break;
        case 2:
          model = ChannelRealization::CHANNEL_MODEL_3GPP_CASE1;
          break;
        case 3:
          model = ChannelRealization::CHANNEL_MODEL_MACROCELL_RURAL;
          break;
        case 4:
          model = ChannelRealization::CHANNEL_MODEL_SATELLITE;
          cout << "Modello scelto: NB-IoT SATELLITARE " << endl;
          break;
        case 5:
          model = ChannelRealization::CHANNEL_MODEL_MACROCELL_URBAN_IMT_3D;
          break;
        default:
          model = ChannelRealization::CHANNEL_MODEL_MACROCELL_URBAN;
          break;
    }

    // define simulation times
    double duration = dur; //+ 1;
    double flow_duration = duration;
    
    UeRandomAccess::RandomAccessType m_UeRandomAccessType = UeRandomAccess::RA_TYPE_NB_IOT;
    GnbRandomAccess::RandomAccessType m_GnbRandomAccessType = GnbRandomAccess::RA_TYPE_NB_IOT;

    // CREATE COMPONENT MANAGER
    Simulator *simulator = Simulator::Init();
    FrameManager *frameManager = FrameManager::Init();
    NetworkManager* networkManager = NetworkManager::Init();

    // CONFIGURE SEED
    if (seed >= 0)
    {
        srand (seed);
    }
    else
    {
        srand (time(NULL));
    }
    
    std::mt19937 gen(seed>=0 ? seed : time(NULL));

  cout << "Simulation with SEED = " << seed << endl;
  cout << "Duration: " << duration << " flow: " << flow_duration << endl;

  // SET FRAME STRUCTURE
  frameManager->SetFrameStructure(FrameManager::FRAME_STRUCTURE_FDD);
  frameManager->SetNRep(NRep);
  frameManager->SetMCSNBIoTSat(MCS);
  frameManager->SetNRUNBIoTSat(NRU);

  //Define Application Container
  CBR CBRApplication[nbUE];
  int cbrApplication = 0;
  int destinationPort = 101;
  int applicationID = 0;

  int class30min = 0;
  int class1hour = 0;
  int class2hour = 0;
  int class1day = 0;

  // CREATE CELL
  Cell *cell = new Cell (0, radius, 0.005, 0, 0);

  networkManager->GetCellContainer ()->push_back (cell);

  // CREATE CHANNELS and propagation loss model
  RadioChannel *dlCh = new RadioChannel ();
  RadioChannel *ulCh = new RadioChannel ();

  // CREATE SPECTRUM
  BandwidthManager* spectrum = new BandwidthManager (bandwidth, bandwidth, 0, 0);
  spectrum->CreateNbIoTspectrum(carriers, spacing, tones);

    cout << "TTI Length: ";
    frameManager->setTTILength(tones, spacing);
    cout << frameManager->getTTILength() << "ms " << endl;

    DEBUG_LOG_START_1(SIM_ENV_SCHEDULER_DEBUG_NB)
    spectrum->Print();
    DEBUG_LOG_END

    GNodeB::ULSchedulerType uplink_scheduler_type;
    switch (schedUL)
    {
        
    case 0:
      uplink_scheduler_type = GNodeB::ULScheduler_TYPE_NB_IOT_FIFO;
      cout << "Scheduler NB FIFO "<< endl;
      break;
      
    case 1:
      uplink_scheduler_type = GNodeB::ULScheduler_TYPE_NB_IOT_ROUNDROBIN;
      cout << "Scheduler NB RR "<< endl;
      break;
      
    default:
      uplink_scheduler_type = GNodeB::ULScheduler_TYPE_NB_IOT_FIFO;
      cout << "Scheduler NB FIFO "<< endl;
      break;
    }

    // The simulator handles only UPLINK case
    
    //Create GNodeB
    NBIoTSimpleErrorModel *errorModel = new NBIoTSimpleErrorModel();
    GNodeB* gnb = new GNodeB (1, cell, -300310, 0, 500000, "sat", nSatellitePerOrbit); // scenario satellitare a 500km
    gnb->SetRandomAccessType(m_GnbRandomAccessType);
    gnb->GetPhy ()->SetDlChannel (dlCh);
    gnb->GetPhy ()->SetUlChannel (ulCh);
    gnb->GetPhy ()->SetNoiseFigure(BSNoiseFigure);
    gnb->GetPhy ()->SetCarrierFrequency(carrierFreq);
    gnb->GetPhy ()->SetBandwidthManager (spectrum);
    gnb->GetPhy ()->SetHeight(antennaHeight);
    gnb->GetPhy ()->SetmaxSatelliteRange(GetMinDistance4CellSelection()); // in base all'angolo di visibilità scelto con SNR > 0
    //gnb->SetNumberOfSatellitePerOrbit(nSatellitePerOrbit);

    //gnb->GetPhy ()->SetmaxSatelliteRange(510000);
    gnb->GetPhy ()->SetErrorModel (errorModel);
    ulCh->AddDevice (gnb);
    //gnb->SetDLScheduler (GNodeB::DLScheduler_TYPE_PROPORTIONAL_FAIR);
    gnb->SetULScheduler(uplink_scheduler_type);


    networkManager->GetGNodeBContainer ()->push_back (gnb);
    //cout << "Created gNB - id 1 position (0;0)"<< endl;
    cout << "Created gNB - id 1 - Out of visibility"<< endl;

    GnbNbIoTRandomAccess* gnbRam = (GnbNbIoTRandomAccess*) gnb->GetMacEntity()->GetRandomAccessManager();

    gnbRam->SetMaxPreambleTx(maxPreambleTx);
    gnbRam->SetPreambleRep(preambleRep);
    gnbRam->SetRarWindow(rarWindow);
    gnbRam->SetSubcarriers(nbPre);
    gnbRam->SetResPeriodicity(rachPeriod);
    gnbRam->SetTimeOffset(rachOffset);
    gnbRam->SetBackoff(boWindow);

    //Create UEs
    int idUE = 2;
    double speedDirection = 0; // How can I simulate gNB movement

    double posX = 0;
    double posY = 0;

    int nbOfZones; // divisione in livelli di MCS visto che non e' presente un feedback del CQI
    if (tones==1)
    {
        nbOfZones=11;
    }
    else
    {
        nbOfZones=14;
    }
    
    int zone; // l'intera cella viene divisa in porzioni
    double zoneWidth = radius*1000 / nbOfZones;
    double edges[nbOfZones+1];
    for (int i=0; i <= nbOfZones; i++)
    {
        edges[i]= i*zoneWidth;
    }
    
    int high, low, sign;
    double distance;

    std::uniform_real_distribution<> spaDis(0.0, zoneWidth);
    std::uniform_int_distribution<> zoneDis(0, nbOfZones-1);
    std::uniform_int_distribution<> sig(1, 2);

    //double rndSpaDis =  (rand () % zoneWidth );

    //double rndSpaDis_temp = (double)rand() / RAND_MAX;
    //double rndSpaDis =  0 + rndSpaDis_temp * (zoneWidth - 0);

	//int rndZoneDis = (rand () % (nbOfZones-1) );
	//int rndSig = (rand () % 1 ) + 1;

    for (int i = 0; i < nbUE; i++)
    {

    zone = zoneDis(gen); // le UE vengono assegnate in modo uniforme a tutte le zone
    //zone = rndZoneDis;
    cout << "ZONE " << zone;
    low = edges[nbOfZones - 1 - zone];
    cout << " LOW EDGE " << low;

    distance = spaDis(gen) + (double) low;
    //distance = rndSpaDis + (double) low;

    cout << " DISTANCE " << distance;
    cout << endl;

    sign = (sig(gen) % 2) * 2 - 1;
    //sign = (rndSig % 2) * 2 - 1;
    posX=distance / sqrt(2) * sign;
    sign = (sig(gen) % 2) * 2 - 1;
    //sign = (rndSig % 2) * 2 - 1;
    posY=distance / sqrt(2) * sign;

    UserEquipment* ue = new UserEquipment (idUE,
                                     posX, posY, 0, speedDirection,
                                     cell,
                                     gnb,
                                     0, //handover false!
                                     Mobility::UE_SATELLITE);

    cout << "Created UE - id " << idUE << " position " << posX << " " << posY << endl;

    ue->SetRandomAccessType(m_UeRandomAccessType);

    ue->SetTimePositionUpdate (0.1); // trigger per la mobilità

    ue->GetPhy ()->SetDlChannel (dlCh);
    ue->GetPhy ()->SetUlChannel (ulCh);

    double propDist = distance/(1000*radius);
    std::map<double, int>::iterator it;
    it = ceProb.upper_bound(propDist);
    UeMacEntity* ueMac = (UeMacEntity*) ue->GetProtocolStack()->GetMacEntity();
    UeNbIoTRandomAccess* ueRam =(UeNbIoTRandomAccess*) ueMac->GetRandomAccessManager();
    ueRam->SetCEClassStatic(it->second);
    ueRam->SetCEClassDynamic(it->second);
    ueRam->SetMaxFailedAttempts(totPreambleTx);

    DEBUG_LOG_START_1(SIM_ENV_CE_CLASS_LOG)
    cout << "CE Class is " << ueRam->GetCEClassStatic()<< " and proportional dist is " << propDist << endl;
    DEBUG_LOG_END

    networkManager->GetUserEquipmentContainer ()->push_back (ue);

    // register ue to the gnb
    gnb->RegisterUserEquipment (ue);

    ////////////////////////////
// ERROR model
        ue->GetPhy ()->SetErrorModel (errorModel);
        ChannelRealization* c_ul = new ChannelRealization (ue, gnb, model);
        //c_ul->disableFastFading();
        gnb->GetPhy ()->GetUlChannel ()->GetPropagationLossModel ()->AddChannelRealization (c_ul);
    ////////////////////////////

    DEBUG_LOG_START_1(SIM_ENV_SCHEDULER_DEBUG_LOG)
    CartesianCoordinates* userPosition = ue->GetMobilityModel()->GetAbsolutePosition();
    CartesianCoordinates* cellPosition = cell->GetCellCenterPosition();

    double distance = userPosition->GetDistance(cellPosition)/1000;

    double zoneWidth = (double) (radius/nbOfZones);
    double edges[nbOfZones+1];
    for (int i=0; i <= nbOfZones; i++)
    {
    	edges[i]= i*zoneWidth;
    }
    if (distance >= edges[nbOfZones])
    {
    	zone = 0;
    }
    for (int i= 0; i < nbOfZones; i++)
    {
		if (distance >= edges[i] && distance <= edges[i+1])
		{
		  zone=nbOfZones-1-i;
		}
    }
cout << "LOG_ZONE UE " << idUE
<< " DISTANCE " << distance
<< " WIDTH " << zoneWidth
<< " ZONE " << zone
<< endl;
DEBUG_LOG_END


    //PERIODIC INTER-ARRIVAL TIME SPLIT
	//  5% 	-> 30 minutes 	-> 1800 sec
	// 15% 	-> 60 minutes 	-> 3600 sec
	// 40% 	-> 120 minutes 	-> 7200 sec
	// 40% 	-> 1 day 		-> 86400 sec

    //double randomNumber = (rand () %100 ) / 100.;
	std::uniform_int_distribution<> rndCBR (0, 100);
	double randomNumber = rndCBR(gen) / 100.;
    double _cbrInterval = 99999.0;

    if(randomNumber <= 0.05){
    	_cbrInterval = 1800.0;
    	class30min++;
    }else if(randomNumber > 0.05 && randomNumber <= 0.20){
    	_cbrInterval = 3600.0;
    	class1hour++;
    }else if(randomNumber > 0.20 && randomNumber <= 0.60){
    	_cbrInterval = 7200.0;
    	class2hour++;
    }else if(randomNumber > 0.60){
    	_cbrInterval = 86400.0;
    	class1day++;
    }
    cout << "CBR interval is " << _cbrInterval << endl;

    //CREATE UPLINK APPLICATION FOR THIS UE
	std::uniform_real_distribution<> timeDis(0.0, (double) _cbrInterval); // intervallo all interno del quale l UE trasmette
	double start_time = .001 + timeDis(gen); // 1 ms + un numero da 0 a CBR_interval
	double duration_time = flow_duration - 0.001;

    //double rndTime = (rand () % _cbrInterval );
    //double rndTime_temp = (double)rand() / RAND_MAX;
    //double rndTime =  0 + rndTime_temp * (_cbrInterval - 0);

    // *** cbr application
    // create application
    CBRApplication[cbrApplication].SetSource (ue);
    CBRApplication[cbrApplication].SetDestination (gnb);
    CBRApplication[cbrApplication].SetApplicationID (applicationID);
    CBRApplication[cbrApplication].SetStartTime(start_time);
    CBRApplication[cbrApplication].SetStopTime(duration_time);

    //CBRApplication[cbrApplication].SetInterval ((double) CBR_interval);
    CBRApplication[cbrApplication].SetInterval (_cbrInterval);
    CBRApplication[cbrApplication].SetSize (CBR_size);

    //-------------------------------------------------------------------
    //------------------------------------------ create qos parameters????

    QoSParameters *qosParameters = new QoSParameters ();
    qosParameters->SetMaxDelay (flow_duration);
    CBRApplication[cbrApplication].SetQoSParameters (qosParameters);


    //create classifier parameters
    ClassifierParameters *cp = new ClassifierParameters (ue->GetIDNetworkNode(),
                                                   gnb->GetIDNetworkNode(),
                                                   0,
                                                   destinationPort,
                                                   TransportProtocol::TRANSPORT_PROTOCOL_TYPE_UDP);
    CBRApplication[cbrApplication].SetClassifierParameters (cp);

    cout << "CREATED CBR APPLICATION, ID " << applicationID << endl;

    //update counter
    //destinationPort++;
    applicationID++;
    cbrApplication++;
    idUE++;
    }

    simulator->SetStop(duration);
    simulator->Run ();

        
}
