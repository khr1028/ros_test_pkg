
#include "../include/common.h"
#include "../include/main_core.h"
#include "../include/main_loop.h"
#include "../include/entry_publisher_tway.h"
#include "../include/entry_subscriber_tway.h"


#include <string>
#include <thread>
#include <chrono>
#include <memory>

#include <cmath>
#include <thread>
#include <mutex>
#include <iostream>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

/******************************************************************
1Hz -> 1/1=1초
2Hz ->1/2=0.5초
10Hz ->1/10=0.1초
100Hz ->1/100=0.01초
1000Hz ->1/1000=0.001초
******************************************************************/
double gSetLinearVelocity = 0.0;
double gSetAngularVelocity = 0.0;
double gMovDstnWhLeft = 0.0;
double gMovDstnWhRight = 0.0;
/******************************************************************
* Declaration for SLAM and Navigation
******************************************************************/
unsigned long prev_update_time;
ros::Time prevUpdateTime;

float odom_pose[3];
double odom_vel[3];

sensor_msgs::Imu gImuMsg;
nav_msgs::Odometry gOdomMsg;
sensor_msgs::JointState gJntSts;
geometry_msgs::TransformStamped gTnsFomStmOdom;
geometry_msgs::TransformStamped imu_tf;

/******************************************************************
* Sensor Status User Message
******************************************************************/
tway_msgs::SensorState 	gSnsStsMsg; //Sensor Status
tway_msgs::MasterMsg 	gMasterMsg;

/******************************************************************
*IMU data
******************************************************************/
double imuAngle =  0.0;
extern sensor_msgs::Imu _mImuData;
/*****************************************************************/





//이놈때문에 ros_init() 에러 발생한다


namespace tway {
	CMainCore::CMainCore(int argc, char* argv[]) {
		ros::init(argc, argv, "tway_core");
		MainProcess();
	}
	CMainCore::CMainCore(int argc, char* argv[], int ndummy ) {
		ros::init(argc, argv, "tway_core");
		MainProcess();
	}
	CMainCore::~CMainCore() {
		int32_t nSetVel[2] = {0,0};
		m_ctrlDriveMotor.SetVelocity(nSetVel, sizeof(nSetVel)/sizeof(int32_t));
		m_ctrlDriveMotor.motor_stop();
		m_ctrlDriveMotor.shut_down();
	}
	void CMainCore::MainProcess() {
		if( !LoadEachDevice() ) {
			ERROR("CanLoadEachDevice() is failed.");
		}
		m_ctrlIMU.LoadEachDevice();
		StandByMotor();
		StandByIO();
		COMMON_SLEEP_MILI(500)
		RosRun();
	}
	
	int CMainCore::LoadEachDevice() {
		m_strPCAN[0] ="can0"; m_strPCAN[1] ="can1";
		std::string strCanCh="";

		for( int nCanCh=0; nCanCh<CAN_CHANNEL_NO; ++nCanCh) {
			strCanCh = (!nCanCh) ? m_strPCAN[0]:m_strPCAN[1];
			if (!m_CanMaster[nCanCh].start(strCanCh, m_strBaudrate)) {
				ERROR("Starting master failed.");
				return EXIT_FAILURE;
			}
			if( nCanCh==0){ //Maxon
				m_CanMaster[nCanCh].core.nmt.reset_all_nodes(); //fix baudrate
				COMMON_SLEEP_MILI(500)
				m_CanMaster[nCanCh].core.nmt.discover_nodes();
				COMMON_SLEEP_MILI(500)
			}

			size_t num = m_CanMaster[nCanCh].num_devices();
			std::cout << "[Infom-load_device()]device number is " << num <<endl;
			if( num <= 0 ) {
				std::cout << "[Error-load_device()]device number is 0" << endl;
				return RETURN_FAIL;
			}
			if( nCanCh == CHANNEL_CAN0 ) {//Maxon
				m_ctrlDriveMotor.LoadDevice( &(m_CanMaster[nCanCh]) );
				COMMON_SLEEP_MILI(500)
				m_ctrlDriveMotor.SetPDOMapping();
			}else if( nCanCh == CHANNEL_CAN1 ) {//Crevis
				m_ctrlIO.LoadDevice( &(m_CanMaster[nCanCh]) );
				COMMON_SLEEP_MILI(500)
				m_ctrlIO.SetPDOMapping();
			}
		}
		return RETURN_SUCCESS;//1
	}
	void CMainCore::StandByMotor() {
		m_ctrlDriveMotor.motor_ready();
		int32_t SetVel[2] = {0,0};
		m_ctrlDriveMotor.SetVelocity(SetVel, sizeof(SetVel)/sizeof(int32_t));
		m_ctrlDriveMotor.motor_start();
	}
	void CMainCore::StandByIO() {
	}
	void CMainCore::RosRun() {
		reg_publisher();
		reg_subscriber();
		reg_mainloop();
		m_Bridge.run();	// run ROS loop
	}
	void CMainCore::reg_publisher() {
		auto ImuPush = std::make_shared<tway::EntryPublisherTway>("imu", this);			 			 //100Hz -> 0.01 sec
		m_Bridge.add_publisher(ImuPush,IMU_PUBLISH_PERIOD);   // 100Hz
		auto OdomPush = std::make_shared<tway::EntryPublisherTway>("odom", this);
		m_Bridge.add_publisher(OdomPush,ODOM_PUBLISH); //100Hz -> 0.01 sec
		auto JointStsPush = std::make_shared<tway::EntryPublisherTway>( "joint_states", this);
		m_Bridge.add_publisher(JointStsPush,JOINT_STATE_PUBLISH); //300Hz -> 0.03 sec
		//auto TFPush = std::make_shared<tway::EntryPublisherTway>( "tf", this);
		//m_Bridge.add_publisher(TFPush,300); //300Hz -> 0.03 sec
		auto iopub_battery_state = std::make_shared<tway::EntryPublisherTway>( "BatteryState", this);
		m_Bridge.add_publisher(iopub_battery_state,1); //1Hz -> 1 sec
		auto SensorStsPush = std::make_shared<tway::EntryPublisherTway>( "SensorState", this);
		m_Bridge.add_publisher(SensorStsPush,30); //500Hz -> 0.05 sec
	}
	void CMainCore::reg_subscriber( ) {
		auto iosub_vel = std::make_shared<tway::EntrySubscriberTway>("cmd_vel", this);
		m_Bridge.add_subscriber(iosub_vel);
		auto MasterMsg = std::make_shared<tway::EntrySubscriberTway>("MasterMsg", this);
		m_Bridge.add_subscriber(MasterMsg);
	}
	void CMainCore::reg_mainloop() {
		auto iomain_loop = std::make_shared<tway::CMainLoop>(this);
		m_Bridge.add_control_loop(iomain_loop, DRIVE_INFORMATION_PUBLISH_PERIOD);//10Hz->0.1sec //100Hz -> 0.01 sec
	}
	//Publishing
	void CMainCore::MakePublishImuMsg(sensor_msgs::Imu& imu_msg, ros::NodeHandle& NodeHandle) {

		imu_msg  = gImuMsg;		
		//imuAngle = 	imu_data_temp.yaw;

		
		//std::cout << "update imu !!!!!!!!!!!!!!!!!" << _mImuData.orientation_covariance[8] << "\n" ;
	}
	void CMainCore::PublishOdometry(nav_msgs::Odometry& OdomStsMsg, ros::NodeHandle& NodeHandle) {//100Hz -> 0.01 sec
		OdomStsMsg = gOdomMsg;
	}
	void CMainCore::PublishJointState(sensor_msgs::JointState& JointStsMsg) {//300Hz -> 0.03 sec
		JointStsMsg = gJntSts;
	}
	//void CMainCore::PublishTF(geometry_msgs::TransformStamped& OdomTFMsg) {//300Hz -> 0.03 sec
	//
	//	}
	// tony 20180608
	void CMainCore::PublishTF(tf2_msgs::TFMessage& tfMsg) {//300Hz -> 0.03 sec
		//tfMsg.transforms.resize(1);
		//tfMsg.transforms[0] = gTnsFomStmOdom;
	}
	void CMainCore::PublishBatteryMsg(sensor_msgs::BatteryState& BatteryStsMsg) { //1Hz -> 1 sec
		//Battery 데이터 처리
	}
	void CMainCore::PublishSensorState(tway_msgs::SensorState& SndSnsStsMsg) {//500Hz -> 0.05 sec
		SndSnsStsMsg = gSnsStsMsg;
	}
	void CMainCore::SubscribeVelocity(double dLinearVel, double dAngularVel) {
		gSetLinearVelocity = dLinearVel;
		gSetAngularVelocity = dAngularVel;
	}
	void CMainCore::SubscribeMasterMsg(tway_msgs::MasterMsg MasterMsg) {
		m_ctrlIO.SetSignalTower(MasterMsg.nSignalTowerColor);
		m_ctrlIO.SetBuzzer(MasterMsg.nBuzzerOn);
		m_ctrlIO.SetMotorPowerOn(MasterMsg.nMotorPowerOn);
		m_ctrlIO.SetBatteryChargeModeOn(MasterMsg.nChargeModeOn);
		m_ctrlIO.SendData();
	}
}








