
#ifndef SOURCE_DIRECTORY__KACANOPEN_TWAY_CORE_INCLUDE_ENTRY_PUBLISHER_TWAY_H_
#define SOURCE_DIRECTORY__KACANOPEN_TWAY_CORE_INCLUDE_ENTRY_PUBLISHER_TWAY_H_

#include "device.h"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/BatteryState.h"
//#include "sensor_msgs/Imu.h"
//tony 20180608
#include "tf2_msgs/TFMessage.h"

#include <string>
#include <tway_msgs/SensorState.h>

#include "../../tway_com/include/publisher.h"
#include "../include/main_core.h"

class CMainCore;

namespace tway {
	class EntryPublisherTway : public kaco::Publisher {
		public:

			EntryPublisherTway(std::string pb_name);
			EntryPublisherTway(std::string pb_name, CMainCore* p_main_core);

			void advertise() override;
			void publish() override;

		private:
			static const bool debug = false;
			static const unsigned queue_size = 100;

			ros::Publisher m_publisher;
			ros::NodeHandle m_NodeHandle;
			std::string m_strTopicName;
			int m_nUser;

		private:
			CMainCore* m_pMainCore;

		private:
			sensor_msgs::Imu   m_ImuMsg;
			nav_msgs::Odometry m_OdometryMsg;
			sensor_msgs::JointState m_JointStateMsg;
			//geometry_msgs::TransformStamped m_OdomTFMsg;
			// tony 20180608
			tf2_msgs::TFMessage m_OdomTFMsg;
			//
			sensor_msgs::BatteryState m_BatSndMsg;
			tway_msgs::SensorState m_SndSnsStsMsg; //Send SensorState Message
	};
} // end namespace tway




#endif /* SOURCE_DIRECTORY__KACANOPEN_TWAY_CORE_INCLUDE_ENTRY_PUBLISHER_TWAY_H_ */
