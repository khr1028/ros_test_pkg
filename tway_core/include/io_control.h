
#ifndef TWAY_CORE_INCLUDE_IO_CONTROL_H_
#define TWAY_CORE_INCLUDE_IO_CONTROL_H_

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
	class IOControl : public Control {
		public:
			IOControl();
			int LoadDevice(kaco::Master* pCanMaster);
			int SetPDOMapping() override;
		public:
			void GetIOStatusDIO();
			void SetSynch();

			void FindBitStatusCh1(uint8_t nData);
			void FindBitStatusCh2(uint8_t nData);

			void SetSignalTower(uint8_t nColor);
			void SetBuzzer(uint8_t nSet);
			void SetMotorPowerOn(uint8_t nSet);
			void SetBatteryChargeModeOn(uint8_t nSet);

			void SendData(uint8_t nSet);
			void SendData();
      void CallBack_Rblinking(const ros::TimerEvent &);
      void CallBack_Gblinking(const ros::TimerEvent &);
      void CallBack_Oblinking(const ros::TimerEvent &);





		public:
			kaco::Master* m_pCanMaster;
			size_t m_nDeviceNo;
			uint16_t m_nProfileNo[2];
			kaco::Device* m_pNodeDevice[2];
      uint16_t m_nNodeId[2];
	};
}
#endif /* TWAY_CORE_INCLUDE_IO_CONTROL_H_ */
