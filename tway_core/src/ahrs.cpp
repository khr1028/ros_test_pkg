
#include "../include/common.h"
#include "../include/ahrs.h"

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

#include <pthread.h>

//#include <fstream>
#include <termios.h>
#include <fcntl.h>

namespace tway
{
        using namespace std;

        bool bAhrsComThExitFlg = true;
        void* AhrsCommThread( void* pThis);
        int open_serial(char *dev_name, int baud, int vtime, int vmin);
        void close_serial(int fd);

        CAhrs::CAhrs()
        {
            m_bSerialOpened = false;
            m_nSerialPortID = 0;
            ang_count = 0;


            //ros::NodeHandle nh;
            //ros::Timer      timer = nh.createTimer( ros::Duration(0.1), CBatChecker::TimerCallBack);
        }

        CAhrs::~CAhrs()
        {
            if(m_bSerialOpened)
                close_serial(m_nSerialPortID);
        }

        int CAhrs::LoadEachDevice()
        {
                // Setup RS485 communication
                //SerialComInit();
                m_nSerialPortID = open_serial((char*)"/dev/ttyS0", 115200, 0, 0);
                m_bSerialOpened = true;
                std::cout << "Initializing IMU... " << std::endl;
                // get some init value
                for(int i = 0; i < 2; i++)
                {
                    GetEulerData();  
                    sleep(1);
                   std::cout << "Initializing IMU... " <<m_euler.yaw << std::endl;
                }

                std::cout << "Finished Initializing IMU... " <<m_euler.yaw << std::endl;




                return RETURN_SUCCESS;//1
      	 }

        void* AhrsCommThread( void* pThis)
        {
                CAhrs* pObj = ((CAhrs*)pThis);
                while (bAhrsComThExitFlg)
                {
                    
                }
        }

        void CAhrs::SerialComInit()
        {
                // Open the Serial Port at the desired hardware port.
                serial_port.Open("/dev/ttyUSB0");

                // Set the baud rate of the serial port.
                serial_port.SetBaudRate(BaudRate::BAUD_115200);

                // Set the number of data bits.
                serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

                // Turn off hardware flow control.
                serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

                // Disable parity.
                serial_port.SetParity(Parity::PARITY_NONE);

                // Set the number of stop bits.
                serial_port.SetStopBits(StopBits::STOP_BITS_1);

        }
#define MAX_ANGLE_BYTES 100
        Euler CAhrs::GetEulerData(void)
        {
                size_t ms_timeout = 20;

                char tmp_read[100];
                int  nTotRead = 0;
                
            
                memset( m_TxBufr, 0x00,5);
                memset( m_RxBufr, 0x00,100);

                m_TxBufr[0] = A;       // a
                m_TxBufr[1] = N;       // n
                m_TxBufr[2] = G;       // g
                m_TxBufr[3] = CR;     // CR
                m_TxBufr[4] = LF;      // LF

                write(m_nSerialPortID,m_TxBufr,5);
                //printf("Send->[%s] \n",m_TxBufr);
                read(m_nSerialPortID, &m_RxBufr, 100);

                /*===========================================================================
                usleep(10000);
                

                while(true)
                {
                                   
                   
                    memset(tmp_read,NULL, 100);
                    int n_read = read(m_nSerialPortID, tmp_read, 100);
                    nTotRead += n_read;
                    strncat(m_RxBufr,tmp_read,n_read);
                    printf("in while Receive->[%s] \n",m_RxBufr);
                    
                    if(m_RxBufr[nTotRead-1] == '\n'  && m_RxBufr[0] =='a' ) break;
                                     
                    usleep(500);
                }
                
                //serial_port.FlushInputBuffer();




                printf("Receive->[%s] \n",m_RxBufr);

                //..
                //usleep(10000);
                //read(m_nSerialPortID, m_RxBufr, 100);
                //printf("Receive->[%s] \n",m_RxBufr);
                */
               //==============================================================================
               /*
                char oneChar;
                bool bStartRead = false;
                bool bValidData = false;
                for(int i=0;i<100;i++)
                {
                    oneChar = m_RxBufr[i];
                    if(oneChar == 'a')
                    {
                        bStartRead = true;                      
                    }
                    if(bStartRead)
                    {
                        strncat(m_RxBufr,&oneChar,1);  
                    }
                    if(oneChar == '\n')
                    {
                        bValidData = true;
                        break;
                    }
                    usleep(10);
                        
                }
                printf("Receive->[%s] \n",m_RxBufr);
                if(!bValidData) 
                {
                    m_euler.roll = 0;
                    m_euler.pitch = 0;
                    m_euler.yaw = 0;
                    return m_euler;

                }
                */
               //printf("Receive:  %s \n",m_RxBufr);
               static int fail_counter = 0;
               static int total_counter = 0;
                if(m_RxBufr[0] == 'a' && m_RxBufr[1] == 'n' && m_RxBufr[2] == 'g')
                {
                    //printf("decode data\n");
                    char *ptr = strtok(m_RxBufr, " ");
                    //printf("get data add\n");

                    ang_count=0;

                    while(ptr != NULL)
                    {
                        
                       // if(ptr == NULL) {fail_counter++;printf("Receive->[%s] \n",ptr); break;}
                        if(ang_count == 1)
                        {
                            //printf("get Roll\n");
                            m_euler.roll = atof(ptr);
                        }
                        else if(ang_count == 2)
                        {
                            //printf("get Pitch %s\n", ptr);
                            m_euler.pitch = atof(ptr);
                        }
                        else if(ang_count == 3)
                        {
                            //printf("get Yaw\n");
                            m_euler.yaw = atof(ptr);
                        }
                        ang_count++;
                        ptr = strtok(NULL, " ");
                    }
                }
                if(ang_count < 4){fail_counter++;}
                total_counter++;
/*
                std::cout << "roll = " << m_euler.roll << std::endl;
                std::cout << "pitch = " << m_euler.pitch << std::endl;
                std::cout << "yaw = " << m_euler.yaw << std::endl;
                std::cout << std::endl;
*/
                printf("Fail counter/total counter: %d/%d\n", fail_counter, total_counter);
                return m_euler;
        }


        int open_serial(char *dev_name, int baud, int vtime, int vmin)
        {
            int fd;
            struct termios newtio;

            //open serial port
            fd = open(dev_name, O_RDWR | O_NOCTTY);
            //cout << "hahah" << endl;
            if(fd < 0)
            {
                //fail open
                cout << "fail" << endl;
                printf("fail open");
                return -1;
            }

            // port configure
            memset(&newtio, 0, sizeof(newtio));
            newtio.c_iflag = IGNPAR;    // no-parity
            newtio.c_oflag = 0;

            newtio.c_cflag = CS8 | CLOCAL | CREAD;  // no-rts & no-cts

            switch(baud)
            {
                case 500000 : newtio.c_cflag |= B500000; break;
                case 250000 : newtio.c_cflag |= B230400; break;
                case 115200 : newtio.c_cflag |= B115200; break;
                case 57600  : newtio.c_cflag |= B57600; break;
                case 9600   : newtio.c_cflag |= B9600; break;
                default     : newtio.c_cflag |= B115200; break;
            }

            newtio.c_lflag = 0;
            newtio.c_cc[VTIME] = vtime;     // timeout 0.1s
            newtio.c_cc[VMIN] = vmin;       // wait

            tcflush(fd, TCIFLUSH);
            tcsetattr(fd, TCSANOW, &newtio);

            printf("connected");
            return fd;
        }

        void close_serial(int fd)
        {
            close(fd);
        }

}
