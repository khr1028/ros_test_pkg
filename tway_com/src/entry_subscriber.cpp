/*
 * Tway
 *
 *
 *
 *
*/
 
#include "../../tway_com/include/entry_subscriber.h"

#include "utils.h"
#include "logger.h"
#include "ros/ros.h"
#include "sdo_error.h"

#include <string>
#include <cstdio>

#include <iostream>
#include <cmath>
#include <thread>
using namespace std;


double goal_linear_velocity = 0.0;
double goal_angular_velocity = 0.0;


namespace kaco
{

	EntrySubscriber::EntrySubscriber(Device& device, const std::string& entry_name, const WriteAccessMethod access_method)
		: m_device(device), m_entry_name(entry_name), m_access_method(access_method)
	{
		uint8_t node_id = device.get_node_id();
		m_device_prefix = "device" + std::to_string(node_id) + "/";
		// no spaces and '-' allowed in ros names
		m_name = Utils::escape(entry_name);
		m_type = device.get_entry_type(entry_name);
	}

	void EntrySubscriber::advertise()
	{
		std::string topic = m_device_prefix+"set_"+m_name;
		DEBUG_LOG("Advertising "<<topic);
		ros::NodeHandle nh;

		switch(m_type)
		{
			case Type::uint8:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_uint8, this);
				break;
			case Type::uint16:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_uint16, this);
				break;
			case Type::uint32:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_uint32, this);
				break;
			case Type::int8:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_int8, this);
				break;
			case Type::int16:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_int16, this);
				break;
			case Type::int32:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_int32, this);
				break;
			case Type::boolean:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_boolean, this);
				break;
			case Type::string:
				m_subscriber = nh.subscribe(topic, queue_size, &EntrySubscriber::receive_string, this);
				break;
			default:
				ERROR("[EntryPublisher::advertise] Invalid entry type.")
		}
	}

	void EntrySubscriber::receive_uint8(const std_msgs::UInt8& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		} catch (const sdo_error& error) {
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_uint8(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_uint16(const std_msgs::UInt16& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		} catch (const sdo_error& error) {
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_uint16(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_uint32(const std_msgs::UInt32& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_uint32(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_int8(const std_msgs::Int8& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!

		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_int8(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_int16(const std_msgs::Int16& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!

		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_int16(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_int32(const std_msgs::Int32& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_int32(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_boolean(const std_msgs::Bool& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_boolean(): "<<error.what());
		}
	}

	void EntrySubscriber::receive_string(const std_msgs::String& msg)
	{
		try
		{
			DEBUG_LOG("Recieved msg: "<<msg.data);
			//m_device.set_synch();
			m_device.set_entry(m_entry_name, msg.data, m_access_method); // auto cast to Value!
		} catch (const sdo_error& error)
		{
			// TODO: only catch timeouts?
			ERROR("Exception in EntrySubscriber::receive_string(): "<<error.what());
		}
	}


} // end namespace kaco
