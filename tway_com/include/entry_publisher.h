/*
 *
 */
 
#pragma once

#include "device.h"
#include "ros/ros.h"

#include "nav_msgs/Odometry.h"

#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
//#include "tway_msgs/Sensor_state.h"

#include <string>
#include "../../tway_com/include/publisher.h"

namespace kaco
{
	/// This class provides a Publisher implementation for
	/// use with kaco::Bridge. It publishes a value from
	/// a device's dictionary.
	class EntryPublisher : public Publisher
	{

	public:

		/// Constructor
		/// \param device The CanOpen device
		/// \param entry_name The name of the entry. See device profile.
		/// \param access_method You can choose default/sdo/pdo method. See kaco::Device docs.
		EntryPublisher(Device& device, const std::string& entry_name, const ReadAccessMethod access_method = ReadAccessMethod::use_default);

		/// \see interface Publisher
		void advertise() override;

		/// \see interface Publisher
		void publish() override;

	private:
		static const bool debug = false;

		// TODO: let the user change this?
		static const unsigned queue_size = 100;

		ros::Publisher m_publisher;

		std::string m_device_prefix;
		std::string m_name;

		Device& m_device;
		std::string m_entry_name;
		ReadAccessMethod m_access_method;
		Type m_type;

	public:


	};

} // end namespace kaco
