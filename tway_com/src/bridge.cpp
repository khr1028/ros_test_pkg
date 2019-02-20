
#include "../../tway_com/include/bridge.h"
#include "logger.h"
#include "sdo_error.h"
#include "ros/ros.h"

#include <future>

namespace kaco {
	void Bridge::add_publisher(std::shared_ptr<Publisher> publisher, double loop_rate) {
		m_publishers.push_back(publisher);
		publisher->advertise();
		m_futures.push_front (
			std::async(std::launch::async, [publisher,loop_rate,this]() {
				ros::Rate rate(loop_rate);
				while(ros::ok()) {
					publisher->publish();
											//DUMP("Bridge::add_publisher is called..loop");
					rate.sleep();
				}
			})
		);
	}
	void Bridge::add_subscriber(std::shared_ptr<Subscriber> subscriber) {
		m_subscribers.push_back(subscriber);
		subscriber->advertise();
	}
	void Bridge::add_control_loop(std::shared_ptr<tway::TestComponent> loop, double loop_rate) {
		m_mainloops.push_back(loop);
		//publisher->advertise();
		m_futures.push_front (
			std::async(std::launch::async, [loop,loop_rate,this]() {
					ros::Rate rate(loop_rate);
					while(ros::ok()) {
						rate.sleep(); //10Hz->0.1sec
						loop->ControlLoop();
												//DUMP("Bridge::add_publisher is called..loop");
					}
			})
		);
	}
	void Bridge::run() {
		ros::AsyncSpinner spinner(0);
		spinner.start();
		ros::waitForShutdown();
	}
}












