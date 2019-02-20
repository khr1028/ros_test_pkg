
#ifndef LIFT_MOTOR_CONTROL_H_
#define LIFT_MOTOR_CONTROL_H_

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
	class CLiftMotorControl : public Control 	{
		public:
			CLiftMotorControl();
			~CLiftMotorControl();
			int SetPDOMapping() override;
		private:
	};
}
#endif /* LIFT_MOTOR_CONTROL_H_ */
