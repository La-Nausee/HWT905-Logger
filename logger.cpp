#include <fstream>
#include <iostream>
#include <cassert>
#include <cstring>
#include <queue>
#include <mutex>
#include <pthread.h>
#include <sys/time.h>

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
#define BUFFER_LENGTH    10240

#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define APP_EXIT_EVENT    0x07

using namespace std;

std::queue<char> hwt_rcv_queue,hwt_log_queue;
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


		if(start)
		{
			hwt_mutex.lock();
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
	
	unsigned char data;
	size_t index;
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz,temperature;
	int16_t roll,pitch,yaw;
	struct timeval tp;

	while(1)
	{
		if(!hwt_log_queue.empty())
		{
			hwt_rcv_queue.push(hwt_log_queue.front());
			switch(hwt_log_queue.front())
			{
			case NEW_FILE_EVENT:
				printf("\r\n[INFO] create new HWT log file.\r\n");
				if(hwt_logfile.is_open())
					hwt_logfile.close();
				hwt_logfile.open(hwt_filename.c_str());
				assert(!hwt_logfile.fail());
				//write headers
				if(hwt_logfile.is_open())
				{
					hwt_logfile<<"Sec.Usec,";
					hwt_logfile<<"ax(g),ay,az,";
					hwt_logfile<<"wx(dps),wy,wz,";
					hwt_logfile<<"roll(degree),pitch,yaw,";
					hwt_logfile<<"mx(Gauss),my,mz,";
					hwt_logfile<<"temperature";
					hwt_logfile<<endl;
				}
				
				start = true;
				
				break;
			case FILE_CLOSE_EVENT:
				printf("\r\n[INFO] close HWT log file.\r\n");
				
				if(hwt_logfile.is_open())
					hwt_logfile.close();
				
				start = false;
				break;
			case APP_EXIT_EVENT:
				printf("\r\n[INFO] exit HWT log thread.\r\n");
				
				if(hwt_logfile.is_open())
					hwt_logfile.close();
				
				start = false;

				pthread_exit(NULL);
				break;
			default:
				break;
			}
			hwt_log_queue.pop();
		}
		
		if(start && (pos_write-pos_read)>= 11)
		{
			hwt_mutex.lock();
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
					if(hwt_logfile.is_open())
					{
						hwt_logfile<<tp.tv_sec<<"."<<tp.tv_usec<<",";
						hwt_logfile<<float(ax/32768.0*16.0)<<","<<float(ay/32768.0*16.0)<<","<<float(az/32768.0*16.0)<<",";
					}
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
					if(hwt_logfile.is_open())
						hwt_logfile<<float(gx/32768.0*2000.0)<<","<<float(gy/32768.0*2000.0)<<","<<float(gz/32768.0*2000.0)<<",";
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
					if(hwt_logfile.is_open())
						hwt_logfile<<float(roll/32768.0*180.0)<<","<<float(pitch/32768.0*180.0)<<","<<float(yaw/32768.0*180.0)<<",";
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
					if(hwt_logfile.is_open())
						hwt_logfile<<mx<<","<<my<<","<<mz<<","<<float(temperature/100.0)<<endl;
				}
			}
			hwt_mutex.unlock();
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
		cout<<"c: create new file for logging data"<<endl;
		cout<<"s: close and save data file"<<endl;
		cout<<"q: save data file and exit the application"<<endl;
		cout<<"Input your selection:";
		key = getchar();
		if(key == 'c')
		{	
			string str;
			cout<<"Input the new filename:";
			//getline(cin,filename);
			cin>>str;
			hwt_filename.clear();
			//hwt_filename = "gx3_";
			hwt_filename.append(str);
			hwt_filename.append(".txt");
			hwt_log_queue.push(NEW_FILE_EVENT);
		}
		else if(key == 's')
		{
			hwt_log_queue.push(FILE_CLOSE_EVENT);
		}
		else if(key == 'q')
		{
			hwt_log_queue.push(APP_EXIT_EVENT);
			pthread_join(thwt_log, NULL);
			pthread_join(thwt_rcv, NULL);
			exit(0);
		}
	}
 
	return 0;
}
