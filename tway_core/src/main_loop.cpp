
#include "../include/common.h"
#include "../include/main_loop.h"



#include "../../tway_com/include/bridge.h"
#include "../../tway_com/include/entry_publisher.h"
#include "../../tway_com/include/entry_subscriber.h"



using namespace std;
string 	tf_prefix ="robot_1/";
//string 	tf_prefix ="";



// These variables were declared in main_core.cpp
/******************************************************************/
extern double gSetLinearVelocity;
extern double gSetAngularVelocity;
extern double gMovDstnWhLeft;
extern double gMovDstnWhRight;
/******************************************************************
* Declaration for SLAM and Navigation
******************************************************************/
extern unsigned long prev_update_time;
extern ros::Time prevUpdateTime;

extern float odom_pose[3];
extern double odom_vel[3];

extern sensor_msgs::Imu gImuMsg;
extern nav_msgs::Odometry gOdomMsg;
extern sensor_msgs::JointState gJntSts;
extern geometry_msgs::TransformStamped gTnsFomStmOdom;
extern geometry_msgs::TransformStamped imu_tf;
double theta=0;
int    imu_compensator_counter = 0;
/******************************************************************/
extern tway_msgs::SensorState 	gSnsStsMsg; //Sensor Status

/******************************************************************
* Calculation for odometry
******************************************************************/
bool init_encoder = true;
double last_diff_dist[WHEEL_NUM] = {0, 0};
double  last_rad[WHEEL_NUM]       = {0.0, 0.0};
double last_velocity[WHEEL_NUM]   = {0.0, 0.0};

/*******************************************************************************
* SoftwareTimer of Tway
*******************************************************************************/
static ros::Time tTime[5];

/*******************************************************************************
* Declaration for Battery
*******************************************************************************/
/******************************************************************
*IMU data
******************************************************************/
extern double imuAngle;
extern sensor_msgs::Imu _mImuData;
/*****************************************************************/

bool setup_end        = false;
uint8_t battery_state = 0;

namespace tway {
	CMainLoop::CMainLoop(CMainCore* pMaincore) : TestComponent("MAIN_LOOP"){
		m_pMainCore = pMaincore;
		InitTime();
		InitOdom();
		InitJointStates();
	}
	void CMainLoop::ControlLoop() {
		//ros::Time now_time = ros::Time::now();
		//std::cout << "[Infom-control_loop()]ros::Time now_time is " << now_time <<endl;
		ControlMotorBySpeed();
		//PublishSensorStateMsg();	
		PublishDriveInfomation();
		DisplayStatus();
		UpdateIMUMsg();
	}
	void CMainLoop::InitTime() {
		ros::Time stamp_now = RosNowTime();
		prevUpdateTime = stamp_now;
	}
	//=============================================================================================//
	// 20180607 Tony: update Odom variables
	void CMainLoop::InitOdom() {
		init_encoder = true;
		for (int index = 0; index < 3; index++) {
			odom_pose[index] = 0.0;
			odom_vel[index]  = 0.0;
		}
		gOdomMsg.pose.pose.position.x = 0.0;
		gOdomMsg.pose.pose.position.y = 0.0;
		gOdomMsg.pose.pose.position.z = 0.0;
		gOdomMsg.pose.pose.orientation.x = 0.0;
		gOdomMsg.pose.pose.orientation.y = 0.0;
		gOdomMsg.pose.pose.orientation.z = 0.0;
		gOdomMsg.pose.pose.orientation.w = 0.0;
		gOdomMsg.twist.twist.linear.x  = 0.0;
		gOdomMsg.twist.twist.angular.z = 0.0;

		gOdomMsg.pose.covariance[0] = 1e-5;
		gOdomMsg.pose.covariance[7] = 1e-5;
		gOdomMsg.pose.covariance[14] = 1e10;
		gOdomMsg.pose.covariance[21] = 1e10;
		gOdomMsg.pose.covariance[28] = 1e10;
		gOdomMsg.pose.covariance[35] = 1e-2;
	
	}
	//=============================================================================================//
	// 20180607 Tony: Init Joint States
	void CMainLoop::InitJointStates() {
		gJntSts.header.frame_id = tf_prefix + "base_link";
		gJntSts.name            = {"",""};
		gJntSts.name[WHEEL_LEFT]     = "wheel_left_joint";
		gJntSts.name[WHEEL_RIGHT]    = "wheel_right_joint";
		gJntSts.position = {0.0, 0.0};
		gJntSts.velocity = {0.0, 0.0};
		gJntSts.effort = {0.0, 0.0};

		
	}
	//=============================================================================================//
	// 20181211 Tony: update IMU
	void CMainLoop::UpdateIMUMsg()
	{
		
		ros::Time time_before = ros::Time::now();
		Euler imu_data_temp = m_pMainCore->m_ctrlIMU.GetEulerData();
		if(!init_imu)
		{
			yaw_offset = imu_data_temp.yaw;
			init_imu = true;
			std::cout << "Yaw Offset " << yaw_offset <<endl;
		}
		float current_yaw = (imu_data_temp.yaw - yaw_offset)*0.01745329251;  // degree to rad

		// change of yaw. Used for odom cal
		delta_yaw_imu = current_yaw - pre_yaw_imu;
		// smooth
		delta_yaw_imu = delta_yaw_imu <= -PI ? delta_yaw_imu + 2*PI : delta_yaw_imu >= PI ? delta_yaw_imu - 2*PI : delta_yaw_imu;
		//update yaw
		pre_yaw_imu = current_yaw;
		//calculate yaw by sum of change
		final_yaw += delta_yaw_imu;
		std::cout << "Yaw: " << final_yaw <<endl;


		gImuMsg.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0, imu_data_temp.yaw);
		gImuMsg.header.stamp = ros::Time::now();
		gImuMsg.header.frame_id = "robot_1/imu_link";
	}
	//=============================================================================================//
	// 20180607 Tony: update odom, tf, jointstates
	void CMainLoop::PublishDriveInfomation() {
		ros::Time curUpdateTime = ros::Time::now();
		ros::Duration duration = curUpdateTime - prevUpdateTime;
		prevUpdateTime = curUpdateTime;
		double step_time = duration.toSec();
		//std::cout << "[Infom-PublishDriveInfomation()Duration] is " << step_time <<endl;

		// calculate odometry
		CalcOdometry(step_time);

		// odometry
		gOdomMsg.header.stamp = curUpdateTime;		
		UpdateOdometry();

  		// odometry tf
  		gTnsFomStmOdom.header.stamp = curUpdateTime;		  
  		UpdateTF(gTnsFomStmOdom, imu_tf);
		tf_broadcaster.sendTransform(gTnsFomStmOdom);
		//imu_tf.header.stamp = curUpdateTime;
		//tf_broadcaster.sendTransform(imu_tf);


		// joint states
		gJntSts.header.stamp = curUpdateTime;
		UpdateJointStates();		
	}
	//=============================================================================================//
	void CMainLoop::PublishSensorStateMsg() {
		gSnsStsMsg.header.stamp = RosNowTime();
		m_pMainCore->m_ctrlIO.GetIOStatusDIO();
		// 20180607 by Tony
		UpdateMotorInfo(gMovDstnWhLeft , gMovDstnWhRight);
		// tony 20180615 . Check operation of IMU
		
		//std::cout << "IMU yaw: " << m_pMainCore->m_pctrlImu->filter_.zeta_ <<endl;


	}
	ros::Time CMainLoop::RosNowTime() {
		ros::Time now_time = ros::Time::now();
		return now_time;
	}
	
	void CMainLoop::UpdateTime() {
	}
	//=============================================================================================//
	// By Tony 20180608
	void CMainLoop::UpdateOdometry(){

	
		gOdomMsg.header.frame_id = tf_prefix + "odom";
		gOdomMsg.child_frame_id  = tf_prefix + "base_footprint";

		gOdomMsg.pose.pose.position.x = odom_pose[0];
		gOdomMsg.pose.pose.position.y = odom_pose[1];
		gOdomMsg.pose.pose.position.z = 0;

		//gOdomMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose[2]);
		gOdomMsg.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,odom_pose[2]);
		

		//geometry_msgs::Quaternion geoMtrQuat;
		//tf::Quaternion tfQuat = tf::createQuaternionFromYaw(odom_pose[2]);
		//tf::quaternionTFToMsg( tfQuat, geoMtrQuat ); //Covert tf::Quaternion to geometry_msg::Quaternion
		//gOdomMsg.pose.pose.orientation = geoMtrQuat;

		gOdomMsg.twist.twist.linear.x  = odom_vel[0];
		gOdomMsg.twist.twist.angular.z = odom_vel[2];
		//gOdomMsg.twist.twist.linear.y = odom_pose[2];
		//gOdomMsg.twist.twist.linear.z = theta;


	}
	//=============================================================================================//
	//=============================================================================================//
	// by tony 20180608
	void CMainLoop::UpdateJointStates(){
		
		static float joint_states_pos[WHEEL_NUM] = {0.0, 0.0};
		static float joint_states_vel[WHEEL_NUM] = {0.0, 0.0};
		//static float joint_states_eff[WHEEL_NUM] = {0.0, 0.0};
		

		joint_states_pos[WHEEL_LEFT]  = last_rad[WHEEL_LEFT];
		joint_states_pos[WHEEL_RIGHT] = last_rad[WHEEL_RIGHT];

		joint_states_vel[WHEEL_LEFT]  = last_velocity[WHEEL_LEFT];
		joint_states_vel[WHEEL_RIGHT] = last_velocity[WHEEL_RIGHT];

		gJntSts.position[WHEEL_LEFT]  = joint_states_pos[WHEEL_LEFT];
		gJntSts.position[WHEEL_RIGHT]  = joint_states_pos[WHEEL_RIGHT];
		

		gJntSts.velocity[WHEEL_LEFT]  = joint_states_vel[WHEEL_LEFT] ;
		gJntSts.velocity[WHEEL_RIGHT]  = joint_states_vel[WHEEL_RIGHT] ;	 

		gJntSts.effort[WHEEL_LEFT] = odom_pose[2];
		gJntSts.effort[WHEEL_LEFT] = odom_pose[2];

		
	}
	//=============================================================================================//
	//=============================================================================================//
	// by tony 20180608
	void CMainLoop::UpdateTF(geometry_msgs::TransformStamped& gTnsFomStmOdom, geometry_msgs::TransformStamped& imu_tf){
	  gTnsFomStmOdom.header = gOdomMsg.header;
	  gTnsFomStmOdom.child_frame_id = tf_prefix + "base_footprint";
	  gTnsFomStmOdom.transform.translation.x = gOdomMsg.pose.pose.position.x;
	  //gTnsFomStmOdom.transform.translation.y = gOdomMsg.pose.pose.position.y;
	  gTnsFomStmOdom.transform.translation.y = gOdomMsg.pose.pose.position.y;
	  gTnsFomStmOdom.transform.translation.z = gOdomMsg.pose.pose.position.z;
	  gTnsFomStmOdom.transform.rotation      = gOdomMsg.pose.pose.orientation;

      // PUBLISH IMU      
      
      imu_tf.header.frame_id = tf_prefix + "base_link";
      imu_tf.child_frame_id = tf_prefix + "imu_link";
      imu_tf.transform.rotation.w = gImuMsg.orientation.w;	
      imu_tf.transform.rotation.x = gImuMsg.orientation.x;
      imu_tf.transform.rotation.y = gImuMsg.orientation.y;
      imu_tf.transform.rotation.z = gImuMsg.orientation.z;

	}
	//=============================================================================================//
  void CMainLoop::ControlMotorBySpeed(){
		double TempVel[2]={0,0}; double TempVel2[2]={0,0}; int32_t SetRPM[2]={0,0};
		//std::cout << "[Infom-controlMotorSpeed()]Set gSetLinearVelocity(m/s) is " << gSetLinearVelocity <<endl;
		//std::cout << "[Infom-controlMotorSpeed()]Set gSetAngularVelocity(rad/s) is " << gSetAngularVelocity <<endl;

		TempVel[WHEEL_LEFT]  = gSetLinearVelocity - (gSetAngularVelocity * WHEEL_SEPARATION / 2);
		TempVel[WHEEL_RIGHT] = gSetLinearVelocity + (gSetAngularVelocity * WHEEL_SEPARATION / 2);
		//std::cout << "[Infom-controlMotorSpeed()]TempVel[WHEEL_LEFT] is " << TempVel[WHEEL_LEFT] <<endl;
		//std::cout << "[Infom-controlMotorSpeed()]TempVel[WHEEL_RIGHT] is " << TempVel[WHEEL_RIGHT] <<endl;
		TempVel2[WHEEL_LEFT] = TempVel[WHEEL_LEFT] * VALOCITY_CONSTANT_VALUE;
		TempVel2[WHEEL_RIGHT] = TempVel[WHEEL_RIGHT] * VALOCITY_CONSTANT_VALUE;
		//std::cout << "[Infom-controlMotorSpeed()]TempVel2[WHEEL_LEFT] is " << TempVel2[WHEEL_LEFT] <<endl;
		//std::cout << "[Infom-controlMotorSpeed()]TempVel2[WHEEL_RIGHT] is " << TempVel2[WHEEL_RIGHT] <<endl;

		SetRPM[WHEEL_LEFT]  = -((TempVel2[WHEEL_LEFT] * 60) / (2 * PI * WHEEL_RADIUS)) * REDUCER_RATE;
		SetRPM[WHEEL_RIGHT] = ((TempVel2[WHEEL_RIGHT] * 60) / (2 * PI * WHEEL_RADIUS)) * REDUCER_RATE;
	    //std::cout <<. "[Infom-controlMotorSpeed()]SetRPM[WHEEL_LEFT] is " << SetRPM[WHEEL_LEFT] <<endl;
    //std::cout <<. "[Infom-controlMotorSpeed()]SetRPM[WHEEL_RIGHT] is " << SetRPM[WHEEL_RIGHT] <<endl;

		m_pMainCore->m_ctrlDriveMotor.SetVelocity(SetRPM, sizeof(SetRPM)/sizeof(int32_t));
		m_pMainCore->m_ctrlDriveMotor.motor_start();
    	m_pMainCore->m_ctrlDriveMotor.Set_Synch();
		DisplayVelocity();
		DisplayPosition();
		UpdateMotorInfo(gMovDstnWhLeft , gMovDstnWhRight);
  }
	void CMainLoop::DisplayVelocity() {
		int32_t nGetVel[2]={0,0};
		m_pMainCore->m_ctrlDriveMotor.GetVelocity( nGetVel, sizeof(nGetVel)/sizeof(int32_t) );
    //std::cout <<. "[Infom-display_velocity()]Left_Get_Velocity(rpm) is "<< std::dec << nGetVel[0]<<endl;
    //std::cout <<. "[Infom-display_velocity()]Right_Get_Velocity(rpm) is "<< std::dec << nGetVel[1]<<endl;
		gSnsStsMsg.nLeftRPM  = nGetVel[0];
		gSnsStsMsg.nRightRPM = nGetVel[1];
	}
	void CMainLoop::DisplayPosition() {
		int32_t GetPos[2]={0,0};
		m_pMainCore->m_ctrlDriveMotor.GetPosition(GetPos, sizeof(GetPos)/sizeof(int32_t));
    //std::cout <<. "[Infom-display_position()]Left_Get_Position( ) is "<< std::dec << GetPos[0]<<endl;
    //std::cout <<. "[Infom-display_position()]Right_Get_Position( ) is "<< std::dec << GetPos[1]<<endl;
		gSnsStsMsg.nLeftEncoder  = GetPos[0];
		gSnsStsMsg.nRightEncoder = GetPos[1];
		//모터 1회전당 펄스수 * 감속비 --> 휠1회전 모터 펄스수 --> 휠 1회전당 모터 펄스수는 2000 * 50(감속비) = 100000 inc
		//휠1회전당 이동거리 = 2 * pi * 0.11(m) = 0.69113(m)
		//1pulse 당 이동거리  0.000006911(m)
		//double dDstnPls = ( 2 * PI * WHEEL_RADIUS ) / ( PER_PULSE_400W * REDUCER_RATE); //1pulse당 이동거리(m)
		gMovDstnWhLeft = GetPos[0] * DIS_PER_PULSE; //left
		gMovDstnWhRight = GetPos[1] * DIS_PER_PULSE;
    //std::cout <<. "[Infom]No1_Distance(m) is "<< std::dec << gMovDstnWhLeft <<endl;
    //std::cout <<. "[Infom]No2_Distance(m) is "<< std::dec << gMovDstnWhRight <<endl;
    //std::cout <<. "=========================================================="<<endl;

	//int32_t GetTor[2]={0,0};
	//m_pMainCore->m_ctrlDriveMotor.GetTorque(GetTor, sizeof(GetTor)/sizeof(int32_t));
	//std::cout << "[Infom]No1_Torque(m) is "<< std::dec << GetTor[0] <<endl;
   // std::cout << "[Infom]No2_Torque(m) is "<< std::dec << GetTor[1] <<endl;




	}
	// Tony 20180608  Update distance ===============================================================//
	void CMainLoop::UpdateMotorInfo(double _dstLeft, double _dstRight)
	{
		double current_dst = 0;
		static double last_dis[WHEEL_NUM] = {0, 0};

		current_dst = -_dstLeft;

		last_diff_dist[WHEEL_LEFT] = current_dst - last_dis[WHEEL_LEFT];
		last_dis[WHEEL_LEFT]      = current_dst;
		last_rad[WHEEL_LEFT]       += last_diff_dist[WHEEL_LEFT]/WHEEL_RADIUS;

		current_dst = _dstRight;

		last_diff_dist[WHEEL_RIGHT] = current_dst - last_dis[WHEEL_RIGHT];
		last_dis[WHEEL_RIGHT]      = current_dst;
		last_rad[WHEEL_RIGHT] += last_diff_dist[WHEEL_RIGHT]/WHEEL_RADIUS;
				
	}
	
	//=============================================================================================//
	// 20180829 Tony: Calc Odometry function
	int CMainLoop::CalcOdometry(double diff_time){
		
		double dist_wheel_l, dist_wheel_r;      // Distance of wheel [m]
		double delta_s, delta_theta;
		double theta_method1;
		static double last_theta = 0.0;
		double v, w;                  // v = translational velocity [m/s], w = rotational velocity [rad/s]
		double step_time;

		dist_wheel_l = dist_wheel_r = 0.0;
		delta_s = delta_theta = theta = 0.0;
		v = w = 0.0;
		step_time = 0.0;

		step_time = diff_time;

		if (step_time == 0)
			return false;

		dist_wheel_l = last_diff_dist[WHEEL_LEFT];  //   unit [m]:
		dist_wheel_r = last_diff_dist[WHEEL_RIGHT]; // [m]

		if (isnan(dist_wheel_l))
		dist_wheel_l = 0.0;

		if (isnan(dist_wheel_r))
		dist_wheel_r = 0.0;

		delta_s     = (dist_wheel_l + dist_wheel_r) / 2.0;
		// Calculate speed
		v = delta_s / step_time;
		//====================================================================================================
		// Orientation Calculation Methods
		// 1. Using Encoders
		// 2. Using IMU
		//====================================================================================================
		//====================================================================================================
		//====================================================================================================
		static double sum_theta_method1 = 0.0;

		// check imu operation 
		//for(int coun= 0; coun < IMU_COUNTER_CHECK_NUM-1; coun++)
		//{
		//	imu_counter_check[coun+1] = imu_counter_check[coun];
		//}
		//imu_counter_check[0] = m_pMainCore->m_pctrlImu.run_counter;

		//if(imu_counter_check[0] != imu_counter_check[IMU_COUNTER_CHECK_NUM-1])   // IMU ok, using IMU data
		//if(false)
		//{
			// Method 1. Using Encoders
			theta_method1 = (dist_wheel_r - dist_wheel_l) / WHEEL_SEPARATION; 
			// Method 2. Using IMU
			// Ref: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
			//m_pMainCore->m_pctrlImu.publish_imu_data(gImuMsg);	
			//imu_compensator_counter++;		
			/*
			double q0, q1, q2, q3;
			q0 = gImuMsg.orientation.w;		
			q1 = gImuMsg.orientation.x;
			q2 = gImuMsg.orientation.y;
			q3 = gImuMsg.orientation.z;
			theta =atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3) +  imu_compensator_counter*0.00002930618;// - 17.2433e-7; // + 6.02178e-7;

			//std::cout <<. "IMU Angle: " << theta << std::endl;
			*/
			//theta = -_mImuData.orientation_covariance[8];
			//====================================================================================================
			if(!init_)
			{
				last_raw_theta = theta;
				last_theta = theta; // at first time
				init_ = true;
			}
			delta_theta = theta -  last_theta;
			last_theta = theta;

			//odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));   // methodo 2
			//odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));  /// meethod 2
			//odom_pose[2] += delta_theta; 				// Method 2
			//last_theta = theta;
			
			// Encoder
			//delta_theta = theta_method1;
			// IMU
			delta_theta = delta_yaw_imu;


			// runge kutta method
			
			w = delta_theta / step_time;  // method 2
			odom_pose[0] += delta_s / 6 * ( cos(odom_pose[2]) + 
											4*cos(odom_pose[2] + delta_theta / 2.0) +
											cos(odom_pose[2] + delta_theta));   // methodo 2
			odom_pose[1] += delta_s / 6 * ( sin(odom_pose[2]) + 
											4*sin(odom_pose[2] + delta_theta / 2.0) +
											sin(odom_pose[2] + delta_theta));   // methodo 2
											
			if(abs(theta_method1) < 1e-15)			
			{
				odom_pose[2] += 0; 				// Method 2				
			}	
			else
			{
				odom_pose[2] += delta_theta; 				// Method 2
			}			
			//std::cout << "theta_method1: " << theta_method1 <<  std::endl;
			
			//odom_pose[2] += delta_theta; 
			/*
			odom_pose[0] += delta_s / 6 * ( cos(odom_pose[2]) + 
											4*cos(odom_pose[2] + theta_method1 / 2.0) +
											cos(odom_pose[2] + theta_method1));   // methodo 2
			odom_pose[1] += delta_s / 6 * ( sin(odom_pose[2]) + 
											4*sin(odom_pose[2] + theta_method1 / 2.0) +
											sin(odom_pose[2] + theta_method1));   // methodo 2
			odom_pose[2] += theta_method1; 				// Method 2
			w = theta_method1/step_time;
			*/

		//}
		//else // using encoder data
		//{

			//std::cout << "sum theta: " << theta <<  std::endl;
			sum_theta_method1 += theta_method1;
			//w = theta_method1 / step_time;
			// compute odometric pose
			//odom_pose[0] += delta_s * cos(odom_pose[2] + (theta_method1/2 ));  // methhod 1
			//odom_pose[1] += delta_s * sin(odom_pose[2] + (theta_method1/2 ));  /// method 1
			//odom_pose[2] += theta_method1; 				// Method 1
			//w = theta_method1/step_time;
			//std::cout <<. "Encoder based Pose: " << sum_theta_method1 << std::endl;
		//}
		//gImuMsg.linear_acceleration.x = odom_pose[2] ;
		//gImuMsg.linear_acceleration.y = sum_theta_method1 ;
		
		//====================================================================================================
		//======================================================================================================
		// 20180828 Tony: add filter for speed
		float velo_linear = 0;
		float velo_ang = 0;
		// speed[0] --> v; speed[1] -> w
		static float speed[2][SPEED_FILTER_NUM] = {0,};   
		speed[0][0] = v ;
		speed[1][0] = w ;
		velo_linear = 0;
		velo_ang = 0;
		for(int i = 0; i < SPEED_FILTER_NUM; i++)
		{
			velo_linear += speed[0][i];
			velo_ang  += speed[1][i];
		}
		v = velo_linear / SPEED_FILTER_NUM;
		w = velo_ang / SPEED_FILTER_NUM;
		for(int i = SPEED_FILTER_NUM - 1; i > 0; i--)
		{
			speed[0][i] = speed[0][i-1];
			speed[1][i] = speed[1][i-1];
		}
		// end of filter	
		//======================================================================================================
		//update speed
		last_velocity[WHEEL_LEFT]  = dist_wheel_l /WHEEL_RADIUS/ step_time;   // RAD/s
		last_velocity[WHEEL_RIGHT] = dist_wheel_r /WHEEL_RADIUS/ step_time;  // RAD/s

		// compute odometric instantaneouse velocity
		odom_vel[0] = v;
		odom_vel[1] = 0.0;
		odom_vel[2] = w;

		
		
		//std::cout <<. "Current Robot Pose: " << odom_pose[2] << std::endl;
		
		
		
		return true;

	}
	//=============================================================================================//
	void CMainLoop::DisplayStatus() {
    uint16_t GetStatus[2]={0,0};
		uint16_t Check_status[4]={0x0007,0x0020,0x0400,0x0800};
		bool result[4]={0,0,0,0};
		int status;
		m_pMainCore->m_ctrlDriveMotor.Check_Status(GetStatus, sizeof(GetStatus)/sizeof(uint16_t));

		for(status=0; status<ALL_STATUS_CHECK; status++ ){
			for(int nIndex=0; nIndex<2; nIndex++){
			result[status]=GetStatus[nIndex] & Check_status[status];
			}
		}
    if(result[0]==false && gSnsStsMsg.nEMSBtnSts == 0){
      m_pMainCore->m_ctrlDriveMotor.clear();
    }
    //std::cout <<. "[Info] : ready to switch on is " << (result[0]?"on":"off") << std::endl;
    //std::cout <<. "[Info] : switched on is " << (result[0]?"on":"off") << std::endl;
    //std::cout <<. "[Info] : operation enabled is " << (result[0]?"on":"off") << std::endl;
    //std::cout <<. "[Info] : fault is " << (result[0]?"off":"on") << std::endl;
    //std::cout <<. "[Info] : quick_stop is " << (result[1]?"off":"on") << std::endl;
    //std::cout <<. "[Info] : target_reached is " << (result[2]?"on":"off") << std::endl;
    //std::cout <<. "[Info] : speed limited is " << (result[3]?"on":"off") << std::endl;

	}
}
