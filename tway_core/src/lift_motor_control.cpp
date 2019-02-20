
#include "../include/common.h"
#include "../include/lift_motor_control.h"
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

namespace tway {
	CLiftMotorControl::CLiftMotorControl() {
	}
	CLiftMotorControl::~CLiftMotorControl() {
	}
	int CLiftMotorControl::SetPDOMapping() {
		return RETURN_SUCCESS;
	}
}
