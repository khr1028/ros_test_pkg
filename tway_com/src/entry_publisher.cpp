/*
 * Tway
 *
 */
 
#include "../../tway_com/include/entry_publisher.h"

#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"



#include <string>

namespace kaco
{
	EntryPublisher::EntryPublisher(Device& device, const std::string& entry_name, const ReadAccessMethod access_method)
		: m_device(device), m_entry_name(entry_name), m_access_method(access_method)
	{
		uint8_t node_id = device.get_node_id();
		m_device_prefix = "device" + std::to_string(node_id) + "/";
		// no spaces and '-' allowed in ros names
		m_name = Utils::escape(entry_name);
		m_type = device.get_entry_type(entry_name);
	}

	void EntryPublisher::advertise()
	{
		std::string topic = m_device_prefix+"get_"+m_name;
		DEBUG_LOG("Advertising "<<topic);
		ros::NodeHandle nh;

		switch(m_type)
		{
			case Type::uint8:
				m_publisher = nh.advertise<std_msgs::UInt8>(topic, queue_size);
				break;
			case Type::uint16:
				m_publisher = nh.advertise<std_msgs::UInt16>(topic, queue_size);
				break;
			case Type::uint32:
				m_publisher = nh.advertise<std_msgs::UInt32>(topic, queue_size);
				break;
			case Type::int8:
				m_publisher = nh.advertise<std_msgs::Int8>(topic, queue_size);
				break;
			case Type::int16:
				m_publisher = nh.advertise<std_msgs::Int16>(topic, queue_size);
				break;
			case Type::int32:
				m_publisher = nh.advertise<std_msgs::Int32>(topic, queue_size);
				break;
			case Type::boolean:
				m_publisher = nh.advertise<std_msgs::Bool>(topic, queue_size);
				break;
			case Type::string:
				m_publisher = nh.advertise<std_msgs::String>(topic, queue_size);
				break;
			default:
				ERROR("[EntryPublisher::advertise] Invalid entry type.")
		}
	}

	void EntryPublisher::publish()
	{
		try
		{
			//wrhan[180502]
			m_device.set_synch(); //

			Value value = m_device.get_entry(m_entry_name, m_access_method);

			switch(m_type)
			{
				case Type::uint8:
				{
					std_msgs::UInt8 msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::uint16:
				{
					std_msgs::UInt16 msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::uint32:
				{
					std_msgs::UInt32 msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::int8:
				{
					std_msgs::Int8 msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::int16:
				{
					std_msgs::Int16 msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::int32:
				{
					std_msgs::Int32 msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::boolean:
				{
					std_msgs::Bool msg;
					msg.data = value; // auto cast!
					m_publisher.publish(msg);
					break;
				}

				case Type::string:
				{
					std_msgs::String msg;
					msg.data = (std::string) value;
					m_publisher.publish(msg);
					break;
				}
				default: {
					ERROR("[EntryPublisher::advertise] Invalid entry type.")
				}
			}
		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntryPublisher::publish(): "<<error.what());
		}
	}

} // end namespace kaco
