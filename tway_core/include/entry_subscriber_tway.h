
#ifndef TWAY_CORE_INCLUDE_ENTRY_SUBSCRIBER_TWAY_H_
#define TWAY_CORE_INCLUDE_ENTRY_SUBSCRIBER_TWAY_H_


#include "device.h"
#include "ros/ros.h"

#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "geometry_msgs/Twist.h"

#include <string>
#include <thread>
#include "../../tway_com/include/subscriber.h"
#include "../include/main_core.h"

class CMainCore;

namespace tway {
	class EntrySubscriberTway : public kaco::Subscriber {
		public:
			EntrySubscriberTway(std::string msg_name ,  CMainCore* pMainCore);
			void advertise() override;
			void commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg);
			void commandMasterMsgCallback(const tway_msgs::MasterMsg& master_msg);

		private:
			static const bool debug = false;
			static const unsigned queue_size = 10000;

			ros::NodeHandle m_NodeHandle;
			ros::Subscriber m_SubScriber;

		private:
			std::string m_strMsgName;
			CMainCore* m_pMainCore;
	};
}

#endif /* TWAY_CORE_INCLUDE_ENTRY_SUBSCRIBER_TWAY_H_ */






