
#ifndef TWAY_CORE_INCLUDE_DRIVE_MOTOR_CONTROL_H_
#define TWAY_CORE_INCLUDE_DRIVE_MOTOR_CONTROL_H_

#include "logger.h"
#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include "../include/control.h"

#include "../../tway_com/include/bridge.h"
#include "../../tway_com/include/entry_publisher.h"
#include "../../tway_com/include/entry_subscriber.h"

using namespace std;

namespace tway {
	class DriveMotorControl : public Control {
		public:
			DriveMotorControl();
			~DriveMotorControl();
			int LoadDevice(kaco::Master* pCanMaster);
			int SetPDOMapping() override;

		private:
			void SetProfile(int nSetMode);
			int Control( int nSetCon);
			void ControlLoad();

		public:
			void Set_Synch();
			void motor_ready();
			void shut_down();
			void motor_start();
			void motor_stop();
			void clear();
			void motor_test();

		public:
			int SetVelocity( int32_t* pSetArr, int nLength );
			double GetVelocity( int32_t* pGetArr, int nLength );
			double GetPosition( int32_t* pGetArr, int nLength);
			double GetTorque( int32_t* pGetArr, int nLength);

			void Check_Status(uint16_t* pGetStatus, int nLength);

		public:
			kaco::Master* m_pCanMaster;
			size_t m_nDeviceNo;
			uint16_t m_nProfileNo[2];
			kaco::Device* m_pNodeDevice[2];
			uint16_t m_nNodeId[2];
		private:
	};
}

#endif /* TWAY_CORE_INCLUDE_DRIVE_MOTOR_CONTROL_H_ */
