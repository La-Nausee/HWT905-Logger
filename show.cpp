#include <fstream>
#include <iostream>
#include <cassert>
#include <cstring>
#include <queue>
#include <mutex>
#include <pthread.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <math.h>

#define DEV_NAME    	 "/dev/ttyAMA0"
#define BAUD_RATE   	 B460800
#define BUFFER_LENGTH    10240

#define SHOW_ACCEL	  			(1<<0)
#define SHOW_ANGVEL	  			(1<<1)
#define SHOW_MAG  	  			(1<<2)
#define SHOW_EULER     			(1<<3)
#define SHOW_Q		  			(1<<4)
#define SHOW_TIMESTAMP		    (1<<5)
#define SHOW_NONE    		    (1<<6)
#define APP_EXIT_EVENT    			(1<<7)

using namespace std;

using namespace std;

std::queue<int> hwt_rcv_queue,hwt_log_queue;
ofstream hwt_logfile;
string hwt_filename;
std::mutex hwt_mutex;

unsigned char *buffer;
ssize_t pos_read,pos_write;

static int fd;

void serial_init(int fd) {
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CS8 | CLOCAL | CREAD;
    tio.c_cc[VTIME] = 100;
    // ボーレートの設定
    cfsetispeed(&tio, BAUD_RATE);
    cfsetospeed(&tio, BAUD_RATE);
    // デバイスに設定を行う
    tcsetattr(fd, TCSANOW, &tio);
}

void *hwt_rcv_thread(void *threadid)
{
	long tid;
	ssize_t size;
	bool start = false;

	tid = (long)threadid;

	hwt_mutex.lock();

	buffer = (unsigned char*)malloc(BUFFER_LENGTH);
	pos_read = 0;
	pos_write = 0;
    if((fd = open(DEV_NAME, O_RDWR|O_NONBLOCK )) < 0)
    {
        printf("Failed to open serial port\r\n");
        pthread_exit(NULL);
    }

	serial_init(fd);

	hwt_mutex.unlock();

	while(1)
	{
		if(!hwt_rcv_queue.empty())
		{
			switch(hwt_rcv_queue.front())
			{
			case SHOW_ACCEL:
			case SHOW_ANGVEL:
			case SHOW_MAG:
			case SHOW_EULER:
				start = true;

				hwt_mutex.lock();
				pos_read = 0;
				pos_write = 0;
				hwt_mutex.unlock();
				
				break;
			case SHOW_NONE:
				start = false;
				break;
			case APP_EXIT_EVENT:
				start = false;
				free(buffer);
				pthread_exit(NULL);
				break;
			default:
				break;
			}
			
			hwt_rcv_queue.pop();
		}


		if(hwt_mutex.try_lock() && start)
		{
			size = read(fd,buffer+(pos_write%BUFFER_LENGTH),BUFFER_LENGTH - (pos_write%BUFFER_LENGTH));//SSIZE_MAX
			pos_write += size;
			if((pos_write - pos_read) > BUFFER_LENGTH)
			{
				printf("Buffer overflow\r\n");
			}
			if( (pos_write/BUFFER_LENGTH) && (pos_read/BUFFER_LENGTH))
			{
				pos_write = pos_write%BUFFER_LENGTH;
				pos_read = pos_read%BUFFER_LENGTH;
			}

			hwt_mutex.unlock();
		}

		usleep(50);
	}
}

void *hwt_log_thread(void *threadid)
{
	long tid;
	bool start = false;

	tid = (long)threadid;
	
	int sensor;
	unsigned char data;
	size_t index;
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz,temperature;
	int16_t roll,pitch,yaw;

	while(1)
	{
		if(!hwt_log_queue.empty())
		{
			sensor = hwt_log_queue.front();
			switch(sensor)
			{
			case SHOW_ACCEL:
			case SHOW_ANGVEL:
			case SHOW_MAG:
			case SHOW_EULER:
				start = true;
				
				break;
			case SHOW_NONE:
				printf("\r\n[INFO] stop showing.\r\n");

				start = false;
				break;
			case APP_EXIT_EVENT:
				printf("\r\n[INFO] exit HWT thread.\r\n");
				
				start = false;

				pthread_exit(NULL);
				break;
			default:
				break;
			}

			hwt_rcv_queue.push(hwt_log_queue.front());
			hwt_log_queue.pop();
		}
		
		if(hwt_mutex.try_lock() && start)
		{
			index = (pos_read++)%BUFFER_LENGTH;
			data = buffer[index];
			if(data == 0x55)
			{
				index = (pos_read++)%BUFFER_LENGTH;
				data = buffer[index];
				if(data == 0x51) //acc
				{
					index = (pos_read++)%BUFFER_LENGTH;
					ax = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					ax |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					ay = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					ay |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					az = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					az |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					temperature = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					temperature |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;//skip sum
				}
				else if(data == 0x52) //gyro
				{
					index = (pos_read++)%BUFFER_LENGTH;
					gx = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					gx |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					gy = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					gy |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					gz = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					gz |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					temperature = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					temperature |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;//skip sum
				}
				else if(data == 0x53) //euler
				{
					index = (pos_read++)%BUFFER_LENGTH;
					roll = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					roll |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					pitch = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					pitch |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					yaw = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					yaw |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					temperature = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					temperature |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;//skip sum
				}
				else if(data == 0x54) //mag
				{
					index = (pos_read++)%BUFFER_LENGTH;
					mx = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					mx |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					my = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					my |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					mz = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					mz |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;
					temperature = buffer[index];
					index = (pos_read++)%BUFFER_LENGTH;
					temperature |= (buffer[index]<<8);

					index = (pos_read++)%BUFFER_LENGTH;//skip sum
				}
			}
			hwt_mutex.unlock();
			switch(sensor)
			{
			case SHOW_ACCEL:
				printf("ax: %0.2f, ay: %0.2f, az: %0.2f\r\n", float(ax/32768.0*16.0), float(ay/32768.0*16.0), float(az/32768.0*16.0));
				break;
			case SHOW_ANGVEL:
				printf("wx: %0.2f, wy: %0.2f, wz: %0.2f\r\n", float(gx/32768.0*2000.0), float(gy/32768.0*2000.0), float(gz/32768.0*2000.0));
				break;
			case SHOW_MAG:
				printf("mx: %0.2f, my: %0.2f, mz: %0.2f\r\n", mx, my, mz);
				break;
			case SHOW_EULER:
				printf("roll: %0.2f, pitch: %0.2f, yaw: %0.2f\r\n", float(roll/32768.0*180.0), float(pitch/32768.0*180.0), float(yaw/32768.0*180.0));
				break;
			default:
				break;
			}	
		}
	}
}

int main()
{
	char key = 0;
	pthread_t thwt_rcv,thwt_log;
	
	pthread_create(&thwt_rcv,NULL,hwt_rcv_thread,(void *)1234);
	pthread_create(&thwt_log,NULL,hwt_log_thread,(void *)1234);

	while(1)
	{
		cout<<endl;
		cout<<"a: show accelerator's data (g)"<<endl;
		cout<<"g: show gyroscope's data (rad/s)"<<endl;
		cout<<"m: show magnetometer's data (Gauss) "<<endl;
		cout<<"e: show euler angles (degree) "<<endl;
		cout<<"n: stop showing"<<endl;
		cout<<"x: exit"<<endl;
		cout<<"input your option:";
		key = getchar();
		if(key == 'a')
		{	
			hwt_log_queue.push(SHOW_ACCEL);
		}
		else if(key == 'g')
		{
			hwt_log_queue.push(SHOW_ANGVEL);
		}
		else if(key == 'm')
		{
			hwt_log_queue.push(SHOW_MAG);
		}
		else if(key == 'e')
		{
			hwt_log_queue.push(SHOW_EULER);
		}
		else if(key == 'n')
		{
			hwt_log_queue.push(SHOW_NONE);
		}
		else if(key == 'x')
		{
			hwt_log_queue.push(APP_EXIT_EVENT);
			pthread_join(thwt_log, NULL);
			pthread_join(thwt_rcv, NULL);
			exit(0);
		}
	}
 
	return 0;
}