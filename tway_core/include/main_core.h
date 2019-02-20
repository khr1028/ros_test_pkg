 
#ifndef MAIN_CORE_H
#define MAIN_CORE_H

#include "logger.h"
#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TransformStamped.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//tony 20180608
#include "tf2_msgs/TFMessage.h"

#include "../../tway_com/include/bridge.h"
#include "../../tway_com/include/entry_publisher.h"
#include "../../tway_com/include/entry_subscriber.h"

#include "../include/control.h"
#include "../include/io_control.h"
#include "../include/drive_motor_control.h"
#include "../include/lift_motor_control.h"

#include "../include/ahrs.h"


#include <tway_msgs/SensorState.h>
#include <tway_msgs/MasterMsg.h>

using namespace std;


class DriveMotorControl;
class IOControl;

namespace tway {
	class CMainCore 	{
		public:
			CMainCore(int argc, char* argv[]);
			CMainCore(int argc, char* argv[], int ndummy );
			~CMainCore();
		protected:
			void MainProcess();
			int LoadEachDevice();
			void StandByMotor();
			void StandByIO();
			void RosRun();
		public:
			void reg_publisher(); 			//publisher
			void reg_subscriber();
			void reg_mainloop();
		public:
			void MakePublishImuMsg(sensor_msgs::Imu& imu_msg, ros::NodeHandle& NodeHandle);
			void PublishOdometry(nav_msgs::Odometry& OdomStsMsg, ros::NodeHandle& NodeHandle);
			void PublishJointState(sensor_msgs::JointState& JointStsMsg);

			//void PublishTF(geometry_msgs::TransformStamped& OdomTFMsg);
			// tony 20180608
			void PublishTF(tf2_msgs::TFMessage& tfMsg);
			
			void PublishBatteryMsg(sensor_msgs::BatteryState& battery);
			void PublishSensorState(tway_msgs::SensorState& SndSnsStsMsg);


			void SubscribeVelocity(double dLinearVel, double dAngularVel); 			//subscriber
			void SubscribeMasterMsg(tway_msgs::MasterMsg MasterMsg);
		public:
			void controlMotorSpeed(double dLinearSetVel, double dAngularSetVel);
			void controlMotorSpeed();
		private:
			kaco::Master m_CanMaster[2];
			kaco::Bridge m_Bridge;
			std::string m_strPCAN[2];
			const std::string m_strBaudrate = "500K";
		private:
			int m_nUser;

			


		public:
			DriveMotorControl 	m_ctrlDriveMotor;
			CLiftMotorControl	m_ctrlLiftMotor;
			IOControl 			m_ctrlIO;
			CAhrs				m_ctrlIMU;

			
	};
}
#endif // MAIN_CORE_H_
