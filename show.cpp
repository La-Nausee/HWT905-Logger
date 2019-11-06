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

#define DEV_NAME    	 "/dev/ttyUSB0"
#define BAUD_RATE   	 B460800
#define SWAP_SIZE        1024
#define BUFFER_LENGTH    (SWAP_SIZE*10)

#define SHOW_ACCEL	  			(1<<0)
#define SHOW_ANGVEL	  			(1<<1)
#define SHOW_MAG  	  			(1<<2)
#define SHOW_EULER     			(1<<3)
#define SHOW_Q		  			(1<<4)
#define SHOW_TIMESTAMP		    (1<<5)
#define SHOW_NONE    		    (1<<6)
#define APP_EXIT_EVENT    			(1<<7)

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
    if((fd = open(DEV_NAME, O_RDWR )) < 0)
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

		if(start)
		{
			hwt_mutex.lock();
			size = read(fd,buffer+pos_write,BUFFER_LENGTH - 1 - pos_write);//SSIZE_MAX
			if(size >= 0)
			{
				pos_write += size;
				if (pos_write >= BUFFER_LENGTH - 1)
					pos_write = 0;
				if( pos_read - pos_write == 1)
				{
					printf("Buffer overflow\r\n");
				}
			}
			hwt_mutex.unlock();
		}
		else
		{
			read(fd,buffer,BUFFER_LENGTH - 1);
		}
		
		usleep(10);
	}
}

ssize_t increase_pos_read()
{
	pos_read++;
	if (pos_read >= BUFFER_LENGTH - 1)
		pos_read = 0;
	
	return pos_read;
}

void *hwt_log_thread(void *threadid)
{
	long tid;
	bool start = false;

	tid = (long)threadid;
	
	int sensor;
	unsigned char data;
	ssize_t index;
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz,temperature;
	int16_t roll,pitch,yaw;

	while(1)
	{
		if(!hwt_log_queue.empty())
		{
			sensor = hwt_log_queue.front();
			hwt_rcv_queue.push(hwt_log_queue.front());
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
			
			hwt_log_queue.pop();
		}
		
		hwt_mutex.lock();
		index = pos_write-pos_read;
		if (index < 0)
			index += BUFFER_LENGTH; 
		if(start && index >= 11)
		{
			index = increase_pos_read();
			data = buffer[index];
			if(data == 0x55)
			{
				index = increase_pos_read();
				data = buffer[index];
				if(data == 0x51) //acc
				{
					index = increase_pos_read();
					ax = buffer[index];
					index = increase_pos_read();
					ax |= (buffer[index]<<8);

					index = increase_pos_read();
					ay = buffer[index];
					index = increase_pos_read();
					ay |= (buffer[index]<<8);

					index = increase_pos_read();
					az = buffer[index];
					index = increase_pos_read();
					az |= (buffer[index]<<8);

					index = increase_pos_read();
					temperature = buffer[index];
					index = increase_pos_read();
					temperature |= (buffer[index]<<8);

					index = increase_pos_read();//skip sum
				}
				else if(data == 0x52) //gyro
				{
					index = increase_pos_read();
					gx = buffer[index];
					index = increase_pos_read();
					gx |= (buffer[index]<<8);

					index = increase_pos_read();
					gy = buffer[index];
					index = increase_pos_read();
					gy |= (buffer[index]<<8);

					index = increase_pos_read();
					gz = buffer[index];
					index = increase_pos_read();
					gz |= (buffer[index]<<8);

					index = increase_pos_read();
					temperature = buffer[index];
					index = increase_pos_read();
					temperature |= (buffer[index]<<8);

					index = increase_pos_read();//skip sum
				}
				else if(data == 0x53) //euler
				{
					index = increase_pos_read();
					roll = buffer[index];
					index = increase_pos_read();
					roll |= (buffer[index]<<8);

					index = increase_pos_read();
					pitch = buffer[index];
					index = increase_pos_read();
					pitch |= (buffer[index]<<8);

					index = increase_pos_read();
					yaw = buffer[index];
					index = increase_pos_read();
					yaw |= (buffer[index]<<8);

					index = increase_pos_read();
					temperature = buffer[index];
					index = increase_pos_read();
					temperature |= (buffer[index]<<8);

					index = increase_pos_read();//skip sum
				}
				else if(data == 0x54) //mag
				{
					index = increase_pos_read();
					mx = buffer[index];
					index = increase_pos_read();
					mx |= (buffer[index]<<8);

					index = increase_pos_read();
					my = buffer[index];
					index = increase_pos_read();
					my |= (buffer[index]<<8);

					index = increase_pos_read();
					mz = buffer[index];
					index = increase_pos_read();
					mz |= (buffer[index]<<8);

					index = increase_pos_read();
					temperature = buffer[index];
					index = increase_pos_read();
					temperature |= (buffer[index]<<8);

					index = increase_pos_read();//skip sum
				}
			}
			switch(sensor)
			{
			case SHOW_ACCEL:
				printf("ax: %0.2f, ay: %0.2f, az: %0.2f\r\n", float(ax/32768.0*16.0), float(ay/32768.0*16.0), float(az/32768.0*16.0));
				break;
			case SHOW_ANGVEL:
				printf("wx: %0.2f, wy: %0.2f, wz: %0.2f\r\n", float(gx/32768.0*2000.0), float(gy/32768.0*2000.0), float(gz/32768.0*2000.0));
				break;
			case SHOW_MAG:
				printf("mx: %d, my: %d, mz: %d\r\n", mx, my, mz);
				break;
			case SHOW_EULER:
				printf("roll: %0.2f, pitch: %0.2f, yaw: %0.2f\r\n", float(roll/32768.0*180.0), float(pitch/32768.0*180.0), float(yaw/32768.0*180.0));
				break;
			default:
				break;
			}	
		}
		hwt_mutex.unlock();
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
