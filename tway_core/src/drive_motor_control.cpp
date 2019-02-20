/*
1Hz -> 1/1=1초
2Hz ->1/2=0.5초
10Hz ->1/10=0.1초
100Hz ->1/100=0.01초
1000Hz ->1/1000=0.001초
*/

#include "../include/common.h"

#include "../../tway_com/include/bridge.h"
#include "../../tway_com/include/entry_publisher.h"
#include "../../tway_com/include/entry_subscriber.h"

#include <string>
#include <thread>
#include <chrono>
#include <memory>

#include <iostream>
#include <cmath>
#include <thread>
#include "../include/drive_motor_control.h"

namespace tway {
	DriveMotorControl::DriveMotorControl() {
	}
	DriveMotorControl::~DriveMotorControl() {
		for( int i=0; i<m_nDeviceNo; ++i ) 	{
			std::cout << "[Infom(destructor)]device no is " << i <<endl;
			m_pCanMaster->core.nmt.send_nmt_message( m_nNodeId[i], kaco::NMT::Command::reset_node); //
			COMMON_SLEEP_MILI(500)
		}
		m_pCanMaster->stop();
		COMMON_SLEEP_MILI(500)
	}
	int DriveMotorControl::LoadDevice(kaco::Master* pCanMaster) {
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
	int DriveMotorControl::SetPDOMapping() {
		for( int nCanCh=0; nCanCh<m_nDeviceNo; ++nCanCh) {
			m_pNodeDevice[nCanCh]->add_receive_pdo_mapping ( 0x1A0 + m_nNodeId[nCanCh], "Statusword",             0  );  //TxPDO1 offset 0,
			m_pNodeDevice[nCanCh]->add_receive_pdo_mapping ( 0x2A0 + m_nNodeId[nCanCh], "Position actual value",  0  );  //TxPDO2 offset 0,
			m_pNodeDevice[nCanCh]->add_receive_pdo_mapping ( 0x3A0 + m_nNodeId[nCanCh], "Velocity actual value",  0  );  //TxPDO3 offset 0,
			m_pNodeDevice[nCanCh]->add_receive_pdo_mapping ( 0x4A0 + m_nNodeId[nCanCh], "Torque actual value",    0  );  //TxPDO4 offset 0,

			m_pNodeDevice[nCanCh]->add_transmit_pdo_mapping( 0x220 + m_nNodeId[nCanCh],{ {"Modes of operation", 0} }); //RxPDO1[offset 0]  1byte
			m_pNodeDevice[nCanCh]->add_transmit_pdo_mapping( 0x320 + m_nNodeId[nCanCh],{ {"Controlword",        0} }); //RxPDO2[offset 0]  2byte
			m_pNodeDevice[nCanCh]->add_transmit_pdo_mapping( 0x420 + m_nNodeId[nCanCh],{ {"Target velocity",    0} }); //RxPDO3[offset 0]  4byte
			Set_Synch();
		}
		return RETURN_SUCCESS;
	}
	void DriveMotorControl::SetProfile( int nSetMode ) {
    int8_t nSetVal = 0;

		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			switch(nSetMode) {
				case MOTOR_PROFILE_VELOCITY:
					nSetVal = (int8_t)0x03;
					break;
				case MOTOR_PROFILE_POSITION:
					nSetVal = (int8_t)0x01;
					break;
			}
			m_pNodeDevice[nSide]->set_entry("Modes of operation", nSetVal, kaco::WriteAccessMethod::cache ); //1byte
			//DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Modes of operation",kaco::ReadAccessMethod::cache));
		}
	}
	int DriveMotorControl::Control( int nSetCon ) {
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			switch(nSetCon) {
				case MOTOR_CONTROL_STOP:
					m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x010F, kaco::WriteAccessMethod::cache ); //2byte
					//DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Controlword",kaco::ReadAccessMethod::cache));
					break;
				case MOTOR_CONTROL_START:
					m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x0F, kaco::WriteAccessMethod::cache ); //2byte
          //std::cout<<"step_start"<<std::endl;
          //DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Controlword",kaco::ReadAccessMethod::cache));
					break;
				case MOTOR_CONTROL_EQ_STOP:
					m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x000B, kaco::WriteAccessMethod::cache ); //2byte
          //std::cout<<"step_stop"<<std::endl;
          //DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Controlword",kaco::ReadAccessMethod::cache));
					break;
				case MOTOR_CONTROL_SHUTDOWN:
					m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x0006, kaco::WriteAccessMethod::cache ); //2byte
					//DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Controlword",kaco::ReadAccessMethod::cache));
					break;
				case MOTOR_CONTROL_CLEAR:
          m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x80, kaco::WriteAccessMethod::pdo ); //2byte
          //std::cout<<"step_clear"<<std::endl;
			}
		}
	}
	void DriveMotorControl::ControlLoad() {
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x0006, kaco::WriteAccessMethod::cache ); //2byte
			//DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Controlword",kaco::ReadAccessMethod::cache));
		}
		Set_Synch();
		COMMON_SLEEP_MILI(500)
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			m_pNodeDevice[nSide]->set_entry("Controlword", (uint16_t)0x0F, kaco::WriteAccessMethod::cache ); //2byte
			//DUMP_HEX(m_pNodeDevice[nSide]->get_entry("Controlword",kaco::ReadAccessMethod::cache));
			break;
		}
		Set_Synch();
	}
	int DriveMotorControl::SetVelocity( int32_t* pSetArr, int nLength ) {
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			//std::cout << "[Infom] SetVelocity() is " << nSide <<":"<<(int32_t)pSetArr[nSide] <<endl;
			m_pNodeDevice[nSide]->set_entry("Target velocity", (int32_t)pSetArr[nSide], kaco::WriteAccessMethod::cache );//4byte
		}
	}
	void DriveMotorControl::Check_Status(uint16_t* pGetStatus, int nLength) {
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			pGetStatus[nSide] = m_pNodeDevice[nSide]->get_entry("Statusword",kaco::ReadAccessMethod::cache);
		}
	}
	void DriveMotorControl::Set_Synch() {
		m_pNodeDevice[WHEEL_LEFT]->set_synch();
	}
	double DriveMotorControl::GetVelocity( int32_t* pGetArr, int nLength ) {
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			pGetArr[nSide] = m_pNodeDevice[nSide]->get_entry("Velocity actual value", kaco::ReadAccessMethod::cache);
		}
	}
	double DriveMotorControl::GetPosition( int32_t* pGetArr, int nLength) {
		for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
			pGetArr[nSide] = m_pNodeDevice[nSide]->get_entry("Position actual value", kaco::ReadAccessMethod::cache);
		}
	}
	double DriveMotorControl::GetTorque( int32_t* pGetArr, int nLength) {
	for( int nSide=0; nSide< m_nDeviceNo ; ++nSide) {
		pGetArr[nSide] = m_pNodeDevice[nSide]->get_entry("Torque actual value", kaco::ReadAccessMethod::cache);
	}
	}
	void DriveMotorControl::motor_ready() {
		SetProfile(MOTOR_PROFILE_VELOCITY);
		ControlLoad();
	}
	void DriveMotorControl::shut_down() {
		Control(MOTOR_CONTROL_STOP);
		Set_Synch();
		Control(MOTOR_CONTROL_SHUTDOWN);
		Set_Synch();
	}
	void DriveMotorControl::motor_start() {
		Control(MOTOR_CONTROL_START);
	}
	void DriveMotorControl::motor_stop() {
		Control(MOTOR_CONTROL_STOP);
	}
	void DriveMotorControl::clear() {
			Control(MOTOR_CONTROL_CLEAR);
  }
	void DriveMotorControl::motor_test() {
		SetProfile(MOTOR_PROFILE_VELOCITY);
		ControlLoad();
		COMMON_SLEEP(1)
		int32_t SetVel[1]={2500};
		SetVelocity(SetVel, sizeof(SetVel)/sizeof(int32_t));
		Control(MOTOR_CONTROL_START);
		Set_Synch();
		COMMON_SLEEP(8)
		shut_down();
	}
}
