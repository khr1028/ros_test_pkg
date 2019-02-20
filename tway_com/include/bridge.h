 
#pragma once

#include "master.h"
#include <string>
#include <vector>
#include <memory>
#include <forward_list>
#include <mutex>
#include "../../tway_com/include/publisher.h"
#include "../../tway_com/include/subscriber.h"
//#include "../../tway_core/include/loop.h"
#include "../../test_component/src/test_component.hpp"



namespace kaco {
	/// This class is a bridge between a ROS network and a CanOpen network.
	class Bridge
	{
		public:

			/// Runs the ROS spinner and blocks until shutdown (e.g. via Ctrl+c).
			void run();

			/// Adds a Publisher, advertises it and publishes messages repeatedly.
			/// \param publisher The publisher. It's a smart pointer because references
			///   to a publisher may never change after advertising it to ROS.
			/// \param loop_rate Publishing rate in hertz. Default is 10 Hz.
			void add_publisher(std::shared_ptr<Publisher> publisher, double loop_rate = 10);

			/// Adds a Subscriber, which can advertise itself and receive
			/// messages on its own.
			/// \param subscriber The subscriber. It's a smart pointer because references
			///   to a subscriber may never change after advertising it to ROS.
			void add_subscriber(std::shared_ptr<Subscriber> subscriber);

			void add_control_loop(std::shared_ptr<tway::TestComponent> loop, double loop_rate  = 10);

		private:

			static const bool debug = false;

			std::vector<std::shared_ptr<Publisher>> m_publishers;	//publisher list
			std::vector<std::shared_ptr<Subscriber>> m_subscribers; //subscriber list
			//std::vector<std::shared_ptr<tway::CLoop>> m_mainloops; //mainloop list
			std::vector<std::shared_ptr<tway::TestComponent>> m_mainloops; //mainloop list

			std::forward_list<std::future<void>> m_futures;
	};

} // end namespace kaco
