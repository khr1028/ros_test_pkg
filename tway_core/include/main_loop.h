
#ifndef TWAY_CORE_INCLUDE_MAIN_LOOP_H_
#define TWAY_CORE_INCLUDE_MAIN_LOOP_H_

#include "logger.h"
#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include <ros/time.h>
#include "sdo_error.h"

#include "../../test_component/src/test_component.hpp"
#include "../include/control.h"
#include "../include/loop.h"
#include "../include/drive_motor_control.h"
#include "../include/io_control.h"
#include "../include/main_core.h"
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

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <tway_msgs/SensorState.h>





using namespace std;

namespace tway {
	class CMainLoop : public TestComponent 	{
		public:
			CMainLoop(CMainCore* pMaincore);
			//~CMainLoop();
		public:
			void ControlLoop();
			void PublishDriveInfomation();
			void PublishSensorStateMsg();

			void UpdateTime();
			ros::Time RosNowTime();
			
			void InitTime();
			void InitOdom();
			void InitJointStates();

			void UpdateOdometry();
			void UpdateIMUMsg();
			
			double last_velocity[WHEEL_NUM] = {0.0, 0.0};
			void UpdateJointStates();
			void UpdateTF(geometry_msgs::TransformStamped& odom_tf, geometry_msgs::TransformStamped& imu_tf);

		public:
			void ControlMotorBySpeed();
			void DisplayVelocity();
			void DisplayPosition();
			void DisplayStatus();

		private:
			CMainCore* m_pMainCore;
			unsigned long program_start_time;


		public:
		//tony 20180608
			int CalcOdometry(double diff_time);
			void UpdateMotorInfo(double _dstLeft, double _dstRight);
			/*******************************************************************************
			* Calculation for odometry
			*******************************************************************************/
			int32_t last_diff_tick[WHEEL_NUM] = {0, 0};
			double  last_rad[WHEEL_NUM]       = {0.0, 0.0};
			double  last_raw_theta = 0;

			/*******************************************************************************
			* create odom_tf
			*******************************************************************************/
			
			tf::TransformBroadcaster tf_broadcaster;


		//tony 20180820
			bool init_ = false;
			bool init_imu = false;
			float yaw_offset = 0;
			float delta_yaw_imu  = 0;
			float pre_yaw_imu = 0;
			float final_yaw = 0;
		//tony 20180829 add imu flag
		int imu_counter_check[IMU_COUNTER_CHECK_NUM] = {0,};
	};
}
#endif /* TWAY_CORE_INCLUDE_MAIN_LOOP_H_ */
