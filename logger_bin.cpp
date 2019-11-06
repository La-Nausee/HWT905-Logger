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
#define SWAP_SIZE        1024
#define BUFFER_LENGTH    (SWAP_SIZE*10)

#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define APP_EXIT_EVENT    0x07

using namespace std;

std::queue<char> hwt_rcv_queue;
ofstream hwt_logfile;
string hwt_filename;

char *buffer;
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

	buffer = (char*)malloc(BUFFER_LENGTH);
	if(buffer == NULL)
	{
		printf("Failed to allocate memory\r\n");
		pthread_exit(NULL);
	}

	if((fd = open(DEV_NAME, O_RDWR|O_NONBLOCK )) < 0)
	{
		printf("Failed to open serial port\r\n");
		pthread_exit(NULL);
	}

	serial_init(fd);

	while(1)
	{
		if(!hwt_rcv_queue.empty())
		{
			switch(hwt_rcv_queue.front())
			{
			case NEW_FILE_EVENT:

				start = true;

				if(hwt_logfile.is_open())
					hwt_logfile.close();
				hwt_logfile.open(hwt_filename.c_str(),ofstream::out | ofstream::trunc | ofstream::binary);
				assert(!hwt_logfile.fail());

				break;
			case FILE_CLOSE_EVENT:
				start = false;
				if(hwt_logfile.is_open())
					hwt_logfile.close();
				break;
			case APP_EXIT_EVENT:
				printf("\r\n[INFO] exit HWT rcv thread.\r\n");
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
			size = read(fd,buffer,BUFFER_LENGTH - 1);//SSIZE_MAX
			if(size > 0)
			{
				if(hwt_logfile.is_open())
					hwt_logfile.write(buffer,size);
			}
		}
	}
}

int main()
{
	char key = 0;
	pthread_t thwt_rcv;
	
	pthread_create(&thwt_rcv,NULL,hwt_rcv_thread,(void *)1234);

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
			hwt_filename.append(".bin");
			hwt_rcv_queue.push(NEW_FILE_EVENT);
		}
		else if(key == 's')
		{
			hwt_rcv_queue.push(FILE_CLOSE_EVENT);
		}
		else if(key == 'q')
		{
			hwt_rcv_queue.push(APP_EXIT_EVENT);
			pthread_join(thwt_rcv, NULL);
			exit(0);
		}
	}
 
	return 0;
}
