
#include "../include/io_control.h"
#include "../include/common.h"

#include <string>
#include <thread>
#include <chrono>
#include <memory>

#include <iostream>
#include <cmath>
#include <thread>
#include <tway_msgs/SensorState.h>


extern tway_msgs::SensorState gSnsStsMsg;
uint8_t	gSendMsgData = 0x00;
uint8_t flag = 0x00;

namespace tway {
	IOControl::IOControl() {
	}
	int IOControl::LoadDevice(kaco::Master* pCanMaster) {

		m_pCanMaster = pCanMaster;
		m_nDeviceNo = m_pCanMaster->num_devices();
		std::cout << "[Infom]device number is " << m_nDeviceNo<<endl;
 		for( int i=0; i<m_nDeviceNo; ++i) {
 			m_pNodeDevice[i] = &(m_pCanMaster->get_device(i));
			m_pNodeDevice[i]->start();
			COMMON_SLEEP_MILI(500)
			m_pNodeDevice[i]->load_dictionary_from_library();
			m_nProfileNo[i] = m_pNodeDevice[i]->get_device_profile_number();
			m_pNodeDevice[i]->print_dictionary();
			DUMP(m_pNodeDevice[i]->get_entry("Manufacturer device name"));
			m_nNodeId[i] = m_pNodeDevice[i]->get_node_id();
      COMMON_SLEEP_MILI(500)

 		}
		return RETURN_SUCCESS;//1
	}
	int IOControl::SetPDOMapping() {
		for( int nCanCh=0; nCanCh<m_nDeviceNo; ++nCanCh) {
			m_pNodeDevice[nCanCh]->add_receive_pdo_mapping ( 0x180 + m_nNodeId[nCanCh], "Read input 8-bit/Digital Inputs 1-8",  0);          //offset 0,
			m_pNodeDevice[nCanCh]->add_receive_pdo_mapping ( 0x180 + m_nNodeId[nCanCh], "Read input 8-bit/Digital Inputs 9-16", 1);          //offset 1
			m_pNodeDevice[nCanCh]->add_transmit_pdo_mapping( 0x200 + m_nNodeId[nCanCh], { {"Write output 8-bit/Digital Outputs 1-8", 0} } ); //offset 0
			m_pNodeDevice[nCanCh]->add_transmit_pdo_mapping( 0x200 + m_nNodeId[nCanCh], { {"Write output 8-bit/Digital Outputs 9-16",1} } ); //offset 1

			m_pNodeDevice[nCanCh]->set_entry(0x1601, 00, (uint8_t) 0x00);
			m_pNodeDevice[nCanCh]->set_entry(0x1A01, 00, (uint8_t) 0x00);
			m_pNodeDevice[nCanCh]->set_entry(0x1400, 02, (uint8_t) 0x01); //Slave->RxPDO Set sync
			m_pNodeDevice[nCanCh]->set_entry(0x1800, 02, (uint8_t) 0x01); //Slave->TxPDO Set
		}
		return RETURN_SUCCESS;//1
	}

	void IOControl::GetIOStatusDIO() {
		for( int nCanCh=0; nCanCh<m_nDeviceNo; ++nCanCh) {
			SetSynch();
			COMMON_SLEEP_MILI(10)  // from 100 -> 30
			//std::cout <<"Status[0]==>"<< m_pNodeDevice[nCanCh]->get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache) << std::endl;//Read Status
			//std::cout <<"Status[1]==>"<< m_pNodeDevice[nCanCh]->get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache) << std::endl;//Read Statu
			uint8_t iData1 = (uint8_t)m_pNodeDevice[nCanCh]->get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache);
			uint8_t iData2 = (uint8_t)m_pNodeDevice[nCanCh]->get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache);
			//std::cout << "First "<<iData1<<std::endl;
			//std::cout << "Second "<<iData2<<std::endl;
			FindBitStatusCh1(iData1);
			FindBitStatusCh2(iData2);
		}
	}
	void IOControl::SetSynch() {
		m_pNodeDevice[WHEEL_LEFT]->set_synch();
	}
	void IOControl::FindBitStatusCh1(uint8_t iData) {
		uint8_t iMaskBit = 0x01;
		uint8_t iDataCpy = iData;
		for( int i=0; i<7 ; ++i ) {
			char cRes = iDataCpy & (iMaskBit << i);
			switch(i) {
				case 0://Battery Charge Status
					if( cRes ) {
						gSnsStsMsg.nBatteryChargeSts = 1; //Battery Charge Status
					}else {
						gSnsStsMsg.nBatteryChargeSts = 0; //Battery Charge Status
					}
					break;
				case 1://Power Off
					if( cRes ) {
						gSnsStsMsg.nPowerOffSts 	  = 1;  //Power Off Status

					}else {
						gSnsStsMsg.nPowerOffSts 	  = 0;  //Power Off Status
					}
					break;
				case 2://EMS
					if( cRes ) {
						gSnsStsMsg.nEMSBtnSts 		  = 1;  //EMS Button On Status
					}else {
						gSnsStsMsg.nEMSBtnSts 		  = 0;  //EMS
					}
          break;
				case 3://Bumper Status
					if( cRes ) {
						gSnsStsMsg.nBumperSts 		  = 1;  //Bumper On
					}else {
						gSnsStsMsg.nBumperSts 		  = 0;  //EMS Button On Status
					}
					break;
			}
		}
	}

	void IOControl::FindBitStatusCh2(uint8_t iData) {
		uint8_t iMaskBit = 0x01;
		uint8_t iDataCpy = iData;
		for( int i=0; i<7 ; ++i ) {
			char cRes = iDataCpy & (iMaskBit << i);
			switch(i) {
				case 0://Home Status
					if( cRes ) {
						gSnsStsMsg.nLiftHomeSts 		= 1; //
					}else {
						gSnsStsMsg.nLiftHomeSts 		= 0; //
					}
					break;
				case 1://Up Limit Status
					if( cRes ) {
						gSnsStsMsg.nLiftUpLimitSts 	= 1; //
					}else {
						gSnsStsMsg.nLiftUpLimitSts 	= 0; //
					}
					break;
				case 2://Down Limit Status
					if( cRes ) {
						gSnsStsMsg.nLiftDownLimitSts 	= 1; //
					}else {
						gSnsStsMsg.nLiftDownLimitSts 	= 0; //
					}
					break;
				case 3://Position Status
					if( cRes ) {
						gSnsStsMsg.nLiftPosiSts 		= 1; //
					}else {
						gSnsStsMsg.nLiftPosiSts 		= 0; //
					}
					break;
			}
		}
	}

	 void IOControl::SetSignalTower(uint8_t nColor) {
        std::cout << "IOControl::SetSignalTower "<<nColor<<std::endl;
        uint8_t    iTemp = 0x00; uint8_t    iTemp2 = 0x00;
        uint8_t iMaskBit = 0x01;

        ros::Timer m_nTimer;
        ros::NodeHandle nh;
        m_nTimer = nh.createTimer(ros::Duration(0.3), &IOControl::CallBack_Oblinking,this);
        switch(nColor) {
            case SIGNAL_TOWER_RED: //1
                iTemp = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_GREEN);    //
                iTemp2 = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_GREEN_L); //
                if( iTemp || iTemp2 ){
                    gSendMsgData &= 0xF9;
                    //SendData(gSendMsgData);
                }
                gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_RED); //
            break;

            case SIGNAL_TOWER_RED_BLINKING: //2
                          m_nTimer = nh.createTimer(ros::Duration(0.3), &IOControl::CallBack_Rblinking,this);
            break;

            case SIGNAL_TOWER_GREEN: //3p
                iTemp = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_RED);      //0
                iTemp2 = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_GREEN_L); //0
                if( iTemp || iTemp2 ){
                    gSendMsgData &= 0xFA;
                    //SendData(gSendMsgData);
                }
                gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_GREEN); //
            break;

            case SIGNAL_TOWER_GREEN_BLINKING:
                m_nTimer = nh.createTimer(ros::Duration(0.3), &IOControl::CallBack_Gblinking,this);
            break;

            case SIGNAL_TOWER_ORANGE: //5
                iTemp = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_GREEN);   //
                if( iTemp ){
                    gSendMsgData &= 0xFD;
                    //SendData(gSendMsgData);
                }
                gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_GREEN_L); //
                gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_SIGNALTOWER_RED);
            break;

            case SIGNAL_TOWER_OFF:
                gSendMsgData &= 0xF8;
            break;

            case SIGNAL_TOWER_ORANGE_BLINKING:
              m_nTimer = nh.createTimer(ros::Duration(0.3), &IOControl::CallBack_Oblinking,this);
            break;
        }
        //SendData(gSendMsgData);
    }
	void IOControl::SetBuzzer(uint8_t nSet) {
		std::cout << "IOControl::SetBuzzer "<<nSet<<std::endl;
		if(nSet) {
			uint8_t iMaskBit = 0x01;
			gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_BUZZER);
		}else {
			gSendMsgData &= 0xF7;
		}
		//SendData(gSendMsgData);
	}
	void IOControl::SetMotorPowerOn(uint8_t nSet) {
		std::cout << "IOControl::SetMotorPowerOn "<<nSet<<std::endl;
		if(nSet){
			uint8_t iMaskBit = 0x01;
			gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_POWER_ON);
		}else {
			gSendMsgData &= 0xEF;
		}
		//SendData(gSendMsgData);
	}
	void IOControl::SetBatteryChargeModeOn(uint8_t nSet) {
		std::cout << "IOControl::SetBatteryChargeModeOn "<<nSet<<std::endl;
		if( nSet) {
			uint8_t iMaskBit = 0x01;
			gSendMsgData = gSendMsgData | (iMaskBit<<DO_SHIFT_CHARGE_ON);
			//SendData(gSendMsgData);
		}else {
			gSendMsgData &= 0xDF;
		}
	}
	void IOControl::SendData(uint8_t iSetMsg) {
		for( int nCanCh=0; nCanCh<m_nDeviceNo; ++nCanCh) {
			m_pNodeDevice[nCanCh]->set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t)iSetMsg, kaco::WriteAccessMethod::cache);
			SetSynch();
		}
	}
	void IOControl::SendData() {
		for( int nCanCh=0; nCanCh<m_nDeviceNo; ++nCanCh) {
			m_pNodeDevice[nCanCh]->set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t)gSendMsgData, kaco::WriteAccessMethod::cache);
			SetSynch();
		}
	}
  void IOControl::CallBack_Rblinking(const ros::TimerEvent & e){
    flag=0x01;
    if(flag&&gSendMsgData){
      gSendMsgData &= 0xF8;
    }
    else gSendMsgData = 0x01;

  }
  void IOControl::CallBack_Gblinking(const ros::TimerEvent & e){

    flag=0x02;
    if(flag&&gSendMsgData){
      gSendMsgData &= 0xF8;
    }
    else gSendMsgData = 0x02;
  }
  void IOControl::CallBack_Oblinking(const ros::TimerEvent & e){
    flag=0x07;
    if((flag&&gSendMsgData)){
      gSendMsgData &= 0xF8;
    }
    else gSendMsgData = 0x05;
  }
}
