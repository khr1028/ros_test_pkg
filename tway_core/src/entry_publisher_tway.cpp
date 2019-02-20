#include "../../tway_core/include/entry_publisher_tway.h"

#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include <string>

extern double gSetLinearVelocity ;
extern double gSetAngularVelocity ;
extern unsigned long prev_update_time;
extern float odom_pose[3];
extern double odom_vel[3];
extern tway_msgs::SensorState gSnsStsMsg;
extern nav_msgs::Odometry odom;

namespace tway {
	EntryPublisherTway::EntryPublisherTway(std::string pb_name)
		: m_strTopicName(pb_name) {
		m_OdomTFMsg.transforms.resize(1);
	}
	EntryPublisherTway::EntryPublisherTway(std::string pb_name, CMainCore* pMainCore)
		: m_strTopicName(pb_name), m_pMainCore(pMainCore) {
			m_OdomTFMsg.transforms.resize(1);

	}
	void EntryPublisherTway::advertise() {
		//std::string topic = "get_" + m_strTopicName;
		//tony 20180608
		std::string topic = m_strTopicName;
		DEBUG_LOG("Advertising "<<topic);
		std::cout << topic <<endl;
		DUMP("[Debug]void EntryPublisher::advertise_user() in..");
		if( !m_strTopicName.compare("imu") ) {
			m_publisher = m_NodeHandle.advertise<sensor_msgs::Imu>(topic, queue_size);
		}else 
		if( !m_strTopicName.compare("odom") ) {
			m_publisher = m_NodeHandle.advertise<nav_msgs::Odometry>(topic, queue_size);
		}
		else if( !m_strTopicName.compare("joint_states") ) {
			m_publisher = m_NodeHandle.advertise<sensor_msgs::JointState>(topic, queue_size);
			
		//}
		//else if( !m_strTopicName.compare("tf") ) {
			//m_publisher = m_NodeHandle.advertise<geometry_msgs::TransformStamped>(topic, queue_size);
			//tony 20180608
		//	m_publisher = m_NodeHandle.advertise<tf2_msgs::TFMessage>(topic, queue_size);
			
		}
		else if( !m_strTopicName.compare("SensorState") ) {
			m_publisher = m_NodeHandle.advertise<tway_msgs::SensorState>(topic, queue_size);
			
		}
		else if( !m_strTopicName.compare("BatteryState") ) {
			m_publisher = m_NodeHandle.advertise<sensor_msgs::BatteryState>(topic, queue_size);
			
		}
	}
	void EntryPublisherTway::publish() {
		try {
			if(!m_strTopicName.compare("imu") ) {
				if( m_pMainCore != NULL )
					m_pMainCore->MakePublishImuMsg(m_ImuMsg, m_NodeHandle);
				m_publisher.publish(m_ImuMsg);
			}else 
			if(!m_strTopicName.compare("odom") ) {
				if( m_pMainCore != NULL )
					m_pMainCore->PublishOdometry(m_OdometryMsg, m_NodeHandle);
				m_publisher.publish(m_OdometryMsg);

			}else if(!m_strTopicName.compare("joint_states") ) {
				if( m_pMainCore != NULL )
					m_pMainCore->PublishJointState(m_JointStateMsg);
				m_publisher.publish(m_JointStateMsg);
			//}else if(!m_strTopicName.compare("tf") ) {
			//	if( m_pMainCore != NULL )
			//		m_pMainCore->PublishTF(m_OdomTFMsg);
			//	m_publisher.publish(m_OdomTFMsg);
			}else if(!m_strTopicName.compare("BatteryState") ) {
				if( m_pMainCore != NULL )
					m_pMainCore->PublishBatteryMsg(m_BatSndMsg);
				m_publisher.publish(m_BatSndMsg);
			}else if(!m_strTopicName.compare("SensorState") ) {
				if( m_pMainCore != NULL )
					m_pMainCore->PublishSensorState(m_SndSnsStsMsg);
				m_publisher.publish(m_SndSnsStsMsg);
			}

		} catch (...) {
			// TODO: only catch timeouts?
			ERROR("Exception in EntryPublisher::publish()");
		}
	}
}
