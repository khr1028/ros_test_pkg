
#ifndef AHRS_H_
#define AHRS_H_

#include "logger.h"
#include "utils.h"
#include "ros/ros.h"

#include "../include/common.h"
#include "../include/control.h"
#include "../include/imu/imu.h"

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

using namespace std;

///////////////////////////////////////
#include "../include/SerialPort.h"
//#include <SerialPort.h>
#include <bitset>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <unistd.h>

using namespace LibSerial;
///////////////////////////////////////

namespace tway
 {

    typedef struct
    {
        // Euler
        float roll;
        float pitch;
        float yaw;

    }Euler;

	class CAhrs
    {
                public:
                        CAhrs();
                        ~CAhrs();

                public:
                        int LoadEachDevice();
                        void SerialComInit();

                public:
                        Euler GetEulerData();

                private:
                        Euler m_euler;
                        char m_RxBufr[100];
                        unsigned char m_TxBufr[5];

                        // ASCII CODE
                        const unsigned char A = 0x61;
                        const unsigned char N = 0x6E;
                        const unsigned char G = 0x67;
                        const unsigned char CR = 0x0D;
                        const unsigned char LF = 0x0A;

                        // Serial Port Device Name
                        int m_nSerialPortID;

                        // Serperate Euler Angle Variable
                        int ang_count;

                private:
                        LibSerial::SerialPort serial_port;
                        bool m_bSerialOpened;
    };
}
#endif /* AHRS_H_ */
