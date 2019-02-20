
#include "../../tway_core/include/entry_subscriber_tway.h"

#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include <string>
#include <cstdio>

#include <iostream>
#include <cmath>
#include <thread>
using namespace std;

extern tway_msgs::MasterMsg gMasterMsg;

namespace tway
{
	EntrySubscriberTway::EntrySubscriberTway(std::string msg_name, CMainCore* pMainCore)
		: m_strMsgName(msg_name),m_pMainCore(pMainCore)
	{
	}

	void EntrySubscriberTway::advertise() {
		//std::string topic = m_device_prefix+"set_"+m_message_name;
		std::string topic = m_strMsgName;
		DEBUG_LOG("Advertising "<<topic);

		if( !m_strMsgName.compare("cmd_vel") ) {
			m_SubScriber = m_NodeHandle.subscribe(topic, queue_size, &EntrySubscriberTway::commandVelocityCallback, this);

		}else if( !m_strMsgName.compare("MasterMsg") ) {
			m_SubScriber = m_NodeHandle.subscribe(topic, queue_size, &EntrySubscriberTway::commandMasterMsgCallback, this);
		}
	}

	void EntrySubscriberTway::commandVelocityCallback(const geometry_msgs::Twist& cmd_vel_msg) {
		//std::cout<<"[Debug]commandVelocityCallback(goal_linear_velocity)-->"<<cmd_vel_msg.linear.x<<endl;
		//std::cout<<"[Debug]commandVelocityCallback(goal_angular_velocity)-->"<<cmd_vel_msg.angular.z<<endl;
		m_pMainCore->SubscribeVelocity( cmd_vel_msg.linear.x, cmd_vel_msg.angular.z);
	}
	void EntrySubscriberTway::commandMasterMsgCallback(const tway_msgs::MasterMsg& master_msg) {
		m_pMainCore->SubscribeMasterMsg(master_msg);
	}

	//New Register
}






