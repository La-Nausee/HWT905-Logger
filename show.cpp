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

	serial_init();

	hwt_mutex.unlock();

	while(1)
	{
		if(!hwt_rcv_queue.empty())
		{
			switch(hwt_rcv_queue.front())
			{
			case NEW_FILE_EVENT:

				start = true;

				hwt_mutex.lock();
				pos_read = 0;
				pos_write = 0;
				hwt_mutex.unlock();

				break;
			case FILE_CLOSE_EVENT:
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
	struct timeval tp;

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
				gettimeofday(&tp,NULL);
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


void *gx3_log_thread(void *threadid)
{
	long tid;
	tid = (long)threadid;
	
	bool start = false;
	int sensor;
	int k;
	float acc[3];
	float ang_vel[3];
	float mag[3];
	float M[9]; //rotation matrix
	float qw,qx,qy,qz;	
	float pitch, roll, yaw;
	double deltaT;
	
	unsigned char *data = (unsigned char*)malloc(DATA_LENGTH);

	while(1)
	{
		if(!gx3_queue.empty())
		{
			sensor = gx3_queue.front();
			switch(sensor)
			{
			case SHOW_ACCEL:
			case SHOW_ANGVEL:
			case SHOW_MAG:
			case SHOW_EULER:
			case SHOW_Q:
			case SHOW_TIMESTAMP:
				gx3_25_start();

				start = true;
				
				break;
			case SHOW_NONE:
				printf("\r\n[INFO] stop showing.\r\n");
				if(start)
					gx3_25_stop();

				start = false;
				break;
			case APP_EXIT_EVENT:
				printf("\r\n[INFO] exit GX3 thread.\r\n");
				
				if(start)
					gx3_25_stop();
				
				start = false;
				
				free(data);
				pthread_exit(NULL);
				break;
			default:
				break;
			}
			gx3_queue.pop();
		}
		
		if(start)
		{
			i = read(fd,data,DATA_LENGTH);

			if(!validate_checksum(data,DATA_LENGTH))
			{
				printf("Failed to checksum on message\r\n");
				continue;
			}
			k = 1;
			for (unsigned int j = 0; j < 3; j++, k += 4)
				acc[j] = -extract_float(&(data[k]));
			for (unsigned int j = 0; j < 3; j++, k += 4)
				ang_vel[j] = -extract_float(&(data[k]));
			for (unsigned int j = 0; j < 3; j++, k += 4)
				mag[j] = -extract_float(&(data[k]));
			for (unsigned int j = 0; j < 9; j++, k += 4)
				M[j] = extract_float(&(data[k]));
			deltaT = extract_int(&(data[k])) / 62500.0;

			qw = sqrt((1.0+M[0]+M[4]+M[8])/2.0);

			qx = (M[7]-M[5])/(4.0*qw);

			qy = (M[2]-M[6])/(4.0*qw);

			qz = (M[3]-M[1])/(4.0*qw);
			
			pitch = asin(-M[2])*180.0/M_PI;
			roll = atan2(M[5],M[8])*180.0/M_PI;
			yaw = atan2(M[1],M[0])*180.0/M_PI;

			switch(sensor)
			{
			case SHOW_ACCEL:
				printf("ax: %0.2f, ay: %0.2f, az: %0.2f\r\n", acc[0], acc[1], acc[2]);
				break;
			case SHOW_ANGVEL:
				printf("wx: %0.2f, wy: %0.2f, wz: %0.2f\r\n", ang_vel[0], ang_vel[1], ang_vel[2]);
				break;
			case SHOW_MAG:
				printf("mx: %0.2f, my: %0.2f, mz: %0.2f\r\n", mag[0], mag[1], mag[2]);
				break;
			case SHOW_EULER:
				printf("roll: %0.2f, pitch: %0.2f, yaw: %0.2f\r\n", roll, pitch, yaw);
				break;
			case SHOW_Q:
				printf("q0: %0.2f, q1: %0.2f, q2: %0.2f, q3: %0.2f\r\n", qw, qx, qy, qz);
				break;
			case SHOW_TIMESTAMP:
				printf("timestamp: %0.2f\r\n", deltaT);
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