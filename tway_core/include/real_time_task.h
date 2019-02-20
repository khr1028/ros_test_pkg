#ifndef TWAY_CORE_INCLUDE_LOOP_H_
#define TWAY_CORE_INCLUDE_LOOP_H_

#include <rtt/TaskContext.hpp>
#include <rtt/Port.hpp>

#include <rtt/RTT.hpp>


namespace tway {
	class RealTimeTask: public RTT::TaskContext {
		public:
			RealTimeTask(std::string const& name);
            virtual void ControlLoop() = 0;

			bool configureHook(){};
            bool startHook(){};
            void updateHook(){};
            void stopHook(){};
            void cleanupHook(){};



	};
	
}

#endif /* CORE_INCLUDE_CONTROL_H_ */
