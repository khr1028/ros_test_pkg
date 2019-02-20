

#ifndef COMMON_H
#define COMMON_H
//




#define 	CAN_CHANNEL_NO 			2

#define		CHANNEL_CAN0			0  //maxon
#define		CHANNEL_CAN1			1  //crevis


#define 	COMMON_SLEEP(x) 		std::this_thread::sleep_for(std::chrono::seconds(x));
#define 	COMMON_SLEEP_MILI(x) 	std::this_thread::sleep_for(std::chrono::milliseconds(x));


#define 	RETURN_SUCCESS          1
#define 	RETURN_FAIL             0

#define 	WHEEL_NUM				2
#define 	WHEEL_LEFT				0
#define 	WHEEL_RIGHT				1


#define 	MOTOR_CONTROL_STOP		0
#define 	MOTOR_CONTROL_START		1
#define 	MOTOR_CONTROL_EQ_STOP	2
#define 	MOTOR_CONTROL_SHUTDOWN  3
#define		MOTOR_CONTROL_CLEAR		4

#define 	MOTOR_PROFILE_VELOCITY	0
#define 	MOTOR_PROFILE_POSITION	1


/////////////////////////////////////////////////////////////////////

#define 	NOMINAL_MOTOR_RPM						2680
#define 	REDUCER_RATE							50
#define 	PI										3.1415

#define 	PER_PULSE_70W							4096
#define 	PER_PULSE_400W							2000
//20180608 Tony
//#define     RAD_PER_PULSE                           0.0000062832
//double dDstnPls = ( 2 * PI * WHEEL_RADIUS ) / ( PER_PULSE_400W * REDUCER_RATE); //1pulse당 이동거리(m)
#define     DIS_PER_PULSE                            0.0000069115
		//모터 1회전당 펄스수 * 감속비 --> 휠1회전 모터 펄스수 --> 휠 1회전당 모터 펄스수는 2000 * 50(감속비) = 100000 inc
		//휠1회전당 이동거리 = 2 * pi * 0.11(m) = 0.69113(m)
		//1pulse 당 이동거리  0.000006911(m)
		//double dDstnPls = ( 2 * PI * WHEEL_RADIUS ) / ( PER_PULSE_400W * REDUCER_RATE); //1pulse당 이동거리(m)
/////////////////////////////////////////////////////////////////////
#define 	WHEEL_RADIUS							0.11    	//meter
#define 	WHEEL_SEPARATION						0.650   //0.663   	//meter
#define 	ENCODER_MIN								-2147483648   //
#define 	ENCODER_MAX								2147483648    //

#define     MAX_LINEAR_VELOCITY						1 // m/s
#define     MAX_ANGULAR_VELOCITY					1 // rad/s

#define 	VALOCITY_CONSTANT_VALUE  				1

/////////////////////////////////////////////////////////////////////
#define		CONTROL_MOTOR_SPEED_PERIOD				50 //hz
#define 	IMU_PUBLISH_PERIOD                  	30  //hz
#define 	CMD_VEL_PUBLISH_PERIOD             	 	30   //hz
#define 	DRIVE_INFORMATION_PUBLISH_PERIOD    	30   //hz   // for main loop
#define 	ODOM_PUBLISH							50
#define 	JOINT_STATE_PUBLISH						30
#define 	VERSION_INFORMATION_PUBLISH_PERIOD  	1    //hz

/////////////////////////////////////////////////////////////////////
#define		SIGNAL_TOWER_OFF						0
#define		SIGNAL_TOWER_RED						1
#define		SIGNAL_TOWER_RED_BLINKING					2
#define		SIGNAL_TOWER_GREEN						3
#define		SIGNAL_TOWER_GREEN_BLINKING				4
#define		SIGNAL_TOWER_ORANGE						5
#define		SIGNAL_TOWER_ORANGE_BLINKING				6

/////////////////////////////////////////////////////////////////////
#define 	DO_SHIFT_SIGNALTOWER_RED				0
#define 	DO_SHIFT_SIGNALTOWER_GREEN				1
#define 	DO_SHIFT_SIGNALTOWER_GREEN_L			2
#define 	DO_SHIFT_BUZZER							3
#define 	DO_SHIFT_POWER_ON						4
#define 	DO_SHIFT_CHARGE_ON						5
/////////////////////////////////////////////////////////////////////////
// FILTER 
#define 	SPEED_FILTER_NUM						10
#define 	IMU_COUNTER_CHECK_NUM					10

/////////////////////////////////////////////////////////////////////////
#define		ALL_STATUS_CHECK						4
#endif // COMMON_H
