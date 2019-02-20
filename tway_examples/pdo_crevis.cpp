

#include <thread>
#include <chrono>
#include <cstdint>

#include "master.h"
#include "logger.h"

#include <stdio.h>

using namespace std;

int main()
{
	const std::string busname = "can1";
	const std::string baudrate = "500K";

	kaco::Master master;
	if (!master.start(busname, baudrate))
	{
		PRINT("Starting Master failed.");
		return EXIT_FAILURE;
	}

	std::this_thread::sleep_for(std::chrono::seconds(1));

	if (master.num_devices()<1)
	{
		ERROR("No devices found.");
		return EXIT_FAILURE;
	}

	master.core.nmt.reset_all_nodes(); //fix baudrate
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	master.core.nmt.discover_nodes();
	std::this_thread::sleep_for(std::chrono::milliseconds(500));

	size_t index;
	bool found = false;
	for (size_t i=0; i<master.num_devices(); ++i)
	{
		kaco::Device& device = master.get_device(i);
		device.start();

		if (device.get_device_profile_number()==401)
		{
			index = i;
			found = true;
			PRINT("Found CiA 401 device with node ID "<<device.get_node_id());
		}
	}

	if (!found)
	{
		ERROR("This example is intended for use with a CiA 402 device but I can't find one.");
		return EXIT_FAILURE;
	}

	kaco::Device& device = master.get_device(index);
	const auto node_id = device.get_node_id();

	device.load_dictionary_from_library();
	DUMP(device.get_entry("Manufacturer device name"));


	// TODO: first configure PDO on device side?
    device.add_receive_pdo_mapping(0x180+node_id, "Read input 8-bit/Digital Inputs 1-8", 0); // offset 0,
    device.add_receive_pdo_mapping(0x180+node_id, "Read input 8-bit/Digital Inputs 9-16", 1); // offset 1

	// transmit PDO on change
	device.add_transmit_pdo_mapping(0x200+node_id, { {"Write output 8-bit/Digital Outputs 1-8", 0} } ); // offset 0
	device.add_transmit_pdo_mapping(0x200+node_id, { {"Write output 8-bit/Digital Outputs 9-16",1} } ); // offset 1
	std::this_thread::sleep_for(std::chrono::seconds(1));

	//master.core.nmt.send_nmt_message(node_id,kaco::NMT::Command::start_node);//operational mode

	device.set_entry(0x1601, 00, (uint8_t) 0x00);
	device.set_entry(0x1A01, 00, (uint8_t) 0x00);

	//device.set_entry(0x1010, 01, (uint32_t) 0x01);  //store parameter

/* Debugging
	std::cout << "Set Network Adapter PDO Mapping=====================================================" << endl;
	std::cout << "number of mapped objects[0x1600]==>" << device.get_entry(0x1600,00) << std::endl;

	std::cout << "number of mapped objects[0x1600,01]==>" << hex << device.get_entry(0x1600,01) << std::endl;
	std::cout << "number of mapped objects[0x1600,02]==>" << device.get_entry(0x1600,02) << std::endl;

	std::cout << "number of mapped objects[0x1601]=>"    << device.get_entry(0x1601,00) << std::endl;
	std::cout << "number of mapped objects[0x1601,01]=>" << device.get_entry(0x1601,01) << std::endl;
	std::cout << "number of mapped objects[0x1601,02]=>" << device.get_entry(0x1601,02) << std::endl;
	std::cout << "number of mapped objects[0x1601,03]=>" << device.get_entry(0x1601,03) << std::endl;
	std::cout << "number of mapped objects[0x1601,04]=>" << device.get_entry(0x1601,04) << std::endl;

	std::cout << "number of mapped objects[0x1602]=>" << device.get_entry(0x1602,00) << std::endl;
	std::cout << "number of mapped objects[0x1603]=>" << device.get_entry(0x1603,00) << std::endl;


	std::cout << "number of mapped objects[0x1A00,00]==>" << device.get_entry(0x1A00,00) << std::endl;
	std::cout << "number of mapped objects[0x1A00,01]==>" << device.get_entry(0x1A00,01) << std::endl;
	std::cout << "number of mapped objects[0x1A00,02]==>" << device.get_entry(0x1A00,02) << std::endl;
	std::cout << "number of mapped objects[0x1A00,03]==>" << device.get_entry(0x1A00,03) << std::endl;


	std::cout << "number of mapped objects[0x1A01,00]==>" << device.get_entry(0x1A01,00) << std::endl;
	std::cout << "number of mapped objects[0x1A01,01]==>" << device.get_entry(0x1A01,01) << std::endl;
	std::cout << "number of mapped objects[0x1A01,02]==>" << device.get_entry(0x1A01,02) << std::endl;
	std::cout << "number of mapped objects[0x1A01,03]==>" << device.get_entry(0x1A01,03) << std::endl;

	std::cout << "number of mapped objects[0x1A02]==>" << device.get_entry(0x1A02,00) << std::endl;
	std::cout << "number of mapped objects[0x1A03]==>" << device.get_entry(0x1A03,00) << std::endl;

	std::cout << "================================================================++++++++======" << endl;
	std::cout << "number of mapped objects[0x1800,00]==>" << device.get_entry(0x1800,00) << std::endl;
	std::cout << "number of mapped objects[0x1800,01]==>" << device.get_entry(0x1800,01) << std::endl;
	std::cout << "number of mapped objects[0x1800,02]==>" << device.get_entry(0x1800,02) << std::endl;
	std::cout << "number of mapped objects[0x1800,03]==>" << device.get_entry(0x1800,03) << std::endl;
	//std::cout << "number of mapped objects[0x1800,04]==>" << device.get_entry(0x1800,04) << std::endl;
	std::cout << "number of mapped objects[0x1800,05]==>" << device.get_entry(0x1800,05) << std::endl;


	std::cout << "number of mapped objects[0x1400,00]==>" << device.get_entry(0x1400,00) << std::endl;
	std::cout << "number of mapped objects[0x1400,01]==>" << device.get_entry(0x1400,01) << std::endl;
	std::cout << "number of mapped objects[0x1400,02]==>" << device.get_entry(0x1400,02) << std::endl;
	//std::cout << "number of mapped objects[0x1400,03]==>" << device.get_entry(0x1400,03) << std::endl;
	//std::cout << "number of mapped objects[0x1400,04]==>" << device.get_entry(0x1400,04) << std::endl;
	//std::cout << "number of mapped objects[0x1400,05]==>" << device.get_entry(0x1400,05) << std::endl;
*/

	device.set_entry(0x1400, 02, (uint8_t) 0x01); //Slave->RxPDO Set sync
	device.set_entry(0x1800, 02, (uint8_t) 0x01); //Slave->TxPDO Set
	//original
	//device.set_entry(0x1400, 02, (uint8_t) 0xFE);
	//device.set_entry(0x1800, 02, (uint8_t) 0xFE);


	device.set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t)0x0f, kaco::WriteAccessMethod::cache);
 // 	DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 1-8",kaco::ReadAccessMethod::cache));
	device.set_entry("Write output 8-bit/Digital Outputs 9-16", (uint8_t)0x01, kaco::WriteAccessMethod::pdo);
 // 	DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 9-16",kaco::ReadAccessMethod::cache));

	device.set_synch();
	//std::this_thread::sleep_for(std::chrono::seconds(1));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	std::cout <<"Status[0]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache) << std::endl;//Read Status
	std::cout <<"Status[1]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache) << std::endl;//Read Status


//	std::cout <<"Status[0]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::pdo_request_and_wait) << std::endl;//Read Status
//	std::cout <<"Status[1]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::pdo_request_and_wait) << std::endl;//Read Status


/*
    device.set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t)0x0f, kaco::WriteAccessMethod::pdo);
    DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 1-8",kaco::ReadAccessMethod::cache));
    std::this_thread::sleep_for(std::chrono::seconds(1));
//  DUMP_HEX(device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache));

	std::cout <<"Status[0]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache) << std::endl;//Read Status
	std::cout <<"Status[1]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache) << std::endl;//Read Status


    device.set_entry("Write output 8-bit/Digital Outputs 9-16", (uint8_t)0x0f, kaco::WriteAccessMethod::pdo);
    DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 9-16",kaco::ReadAccessMethod::cache));
    std::this_thread::sleep_for(std::chrono::seconds(1));
//  DUMP_HEX(device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache));

	std::cout <<"Status[0]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache) << std::endl;//Read Status
	std::cout <<"Status[1]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache) << std::endl;//Read Status

    device.set_entry("Write output 8-bit/Digital Outputs 9-16", (uint8_t)0xf0, kaco::WriteAccessMethod::pdo);
    DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 9-16",kaco::ReadAccessMethod::cache));
    std::this_thread::sleep_for(std::chrono::seconds(1));

	std::cout <<"Status[0]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache) << std::endl;//Read Status
	std::cout <<"Status[1]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache) << std::endl;//Read Status

    device.set_entry("Write output 8-bit/Digital Outputs 1-8", (uint8_t)0xf0, kaco::WriteAccessMethod::pdo);
    DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 1-8",kaco::ReadAccessMethod::cache));
    std::this_thread::sleep_for(std::chrono::seconds(1));

	std::cout <<"Status[0]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 1-8",kaco::ReadAccessMethod::cache) << std::endl;//Read Status
	std::cout <<"Status[1]==>"<< device.get_entry("Read input 8-bit/Digital Inputs 9-16",kaco::ReadAccessMethod::cache) << std::endl;//Read Status

    device.set_entry("Write output 8-bit/Digital Outputs 9-16", (uint8_t)0xf1, kaco::WriteAccessMethod::pdo);
    DUMP_HEX(device.get_entry("Write output 8-bit/Digital Outputs 9-16",kaco::ReadAccessMethod::cache));
    std::this_thread::sleep_for(std::chrono::seconds(1));
*/

	master.stop();
	std::this_thread::sleep_for(std::chrono::seconds(1));
}





