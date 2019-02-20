
#ifndef CORE_INCLUDE_CONTROL_H_
#define CORE_INCLUDE_CONTROL_H_

#include "logger.h"
#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include "../../tway_com/include/bridge.h"
#include "../../tway_com/include/entry_publisher.h"
#include "../../tway_com/include/entry_subscriber.h"


using namespace std;
namespace tway {
	class Control{
		public:
			virtual int SetPDOMapping() = 0;
		public:

	};
	
}
#endif /* CORE_INCLUDE_CONTROL_H_ */
