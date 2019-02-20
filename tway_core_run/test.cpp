
#include "logger.h"
#include <thread>
#include <chrono>
#include <memory>

#include <iostream>
#include <cmath>
#include <thread>

#include "../../tway_com/include/bridge.h"
#include "../../tway_com/include/entry_publisher.h"
#include "../../tway_com/include/entry_subscriber.h"

#include "../tway_core/include/main_core.h"

using namespace std;

bool g_bSync_End = true;
kaco::Device* g_pDevice = NULL;

void Sync_Thread_Func1()
{
	while( g_bSync_End )
	{
		g_pDevice->set_synch();
		std::cout<<"[Debug]Thread_Func1 Running.."<<endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	std::cout<<"Thread_Func1 End..."<<endl;
}

int main(int argc, char* argv[])
{
	const std::string busname = "can0";
	const std::string baudrate = "500K";

	kaco::Master master;
	if (!master.start(busname, baudrate)) {
		ERROR("Starting master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	if (master.num_devices()<1) {
		ERROR("No devices found.");
		return EXIT_FAILURE;
	}

	kaco::Device& device = master.get_device(0);
	g_pDevice = &device;

	device.start();
	device.load_dictionary_from_library();
	uint16_t profile = device.get_device_profile_number();

	device.print_dictionary();
	DUMP(device.get_entry("Manufacturer device name"));
	const auto node_id = device.get_node_id();

	kaco::Bridge bridge;

	if (profile == 401)
	{
		ros::init(argc, argv, "canopen_bridge");

		device.add_receive_pdo_mapping(0x180+node_id, "Read input 8-bit/Digital Inputs 1-8", 0); // offset 0,
		device.add_receive_pdo_mapping(0x180+node_id, "Read input 8-bit/Digital Inputs 9-16", 1); // offset 1
		device.add_transmit_pdo_mapping(0x200+node_id, { {"Write output 8-bit/Digital Outputs 1-8", 0} } ); // offset 0
		device.add_transmit_pdo_mapping(0x200+node_id, { {"Write output 8-bit/Digital Outputs 9-16",1} } ); // offset 1

		std::this_thread::sleep_for(std::chrono::seconds(1));

		device.set_entry(0x1601, 00, (uint8_t) 0x00);
		device.set_entry(0x1A01, 00, (uint8_t) 0x00);

		device.set_entry(0x1400, 02, (uint8_t) 0x01); //Slave->RxPDO Set sync
		device.set_entry(0x1800, 02, (uint8_t) 0x01); //Slave->TxPDO Set

		auto iosub = std::make_shared<kaco::EntrySubscriber>(device, "Write output 8-bit/Digital Outputs 1-8", kaco::WriteAccessMethod::pdo);
		bridge.add_subscriber(iosub);

		device.set_synch(); //std::thread t1(&Sync_Thread_Func1);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		auto iopub = std::make_shared<kaco::EntryPublisher>(device, "Read input 8-bit/Digital Inputs 1-8", kaco::ReadAccessMethod::cache);
		auto iopub2 = std::make_shared<kaco::EntryPublisher>(device, "Read input 8-bit/Digital Inputs 9-16", kaco::ReadAccessMethod::cache);
		bridge.add_publisher(iopub,100); //100Hz -> 0.01 sec
		bridge.add_publisher(iopub2,300); //300Hz -> 0.03 sec
	}
	else if (profile==402) //Maxon
	{
		ros::init(argc, argv, "canopen_bridge");

		device.add_receive_pdo_mapping(0x1A0+node_id, "Statusword", 0); //TxPDO1 offset 0,
		device.add_transmit_pdo_mapping(0x220+node_id,{ {"Modes of operation", 0} }); //RxPDO1[offset 0]  1byte
		device.add_transmit_pdo_mapping(0x320+node_id,{ {"Controlword",        0} }); //RxPDO2[offset 0]  2byte
		device.add_transmit_pdo_mapping(0x420+node_id,{ {"Target velocity",    0} }); //RxPDO3[offset 0]  4byte

		std::this_thread::sleep_for(std::chrono::seconds(1));

		auto iosub_op_mode = std::make_shared<kaco::EntrySubscriber>(device, "Modes of operation", kaco::WriteAccessMethod::pdo);
		auto iosub_control = std::make_shared<kaco::EntrySubscriber>(device, "Controlword", kaco::WriteAccessMethod::pdo);
		auto iosub_vel = std::make_shared<kaco::EntrySubscriber>(device, "Target velocity", kaco::WriteAccessMethod::pdo);
		bridge.add_subscriber(iosub_op_mode);
		bridge.add_subscriber(iosub_control);
		bridge.add_subscriber(iosub_vel);

		device.set_synch();
		std::this_thread::sleep_for(std::chrono::seconds(1));
		auto iopub = std::make_shared<kaco::EntryPublisher>(device, "Statusword", kaco::ReadAccessMethod::cache);
		bridge.add_publisher(iopub,300); //300Hz -> 0.03 sec
	}

	bridge.run();	// run ROS loop

	if(profile == 402 )
	{
		master.core.nmt.send_nmt_message(node_id,kaco::NMT::Command::reset_node);//
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	master.stop();
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

/*
1Hz -> 1/1=1초
2Hz ->1/2=0.5초
10Hz ->1/10=0.1초
100Hz ->1/100=0.01초
1000Hz ->1/1000=0.001초
*/

