#include "ros/ros.h"
#include "viconros/viconmocap.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


#define BAUDRATE 	B57600
#define DEVICE 	"/dev/ttyUSB0"  

int serial_fd;
char send_buf[20];
unsigned char data_send[14]
short data[6];
unsigned short crc_data = 0;

void viconCallback(const viconros::viconmocap msg);
int set_serial(int fd, int nSpeed, int nBits, char nEvent, int nStop) ;
void serial_init();
void send(char*data, int data_byte);
unsigned short crc_update(unsigned short  crc ,  unsigned char data);
unsigned short crc(void* data, unsigned short count);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/vicon", 10, viconCallback);
	ros::Rate loop_rate(10);

	serial_init();
	while(ros::ok())
	{
		sprintf(send_buf, ">*>%c%c%c%c%c%c%c%c%c%c%c%c%c%c<*<", data_send[0], data_send[1],
						         data_send[2], data_send[3],
						         data_send[4], data_send[5],
						         data_send[6], data_send[7],
						         data_send[8], data_send[9],
						         data_send[10], data_send[11],
						         data_send[12], data_send[13]);
		send(data_send, 20);
		ros::spinOnce();
		loop_rate.sleep();
	}
	close(serial_fd);
	return 0;
}

void viconCallback(const viconros::viconmocap msg)
{
 	data[0] = (int) msg.position.x * 1000;
 	data[1] = (int) msg.position.y * 1000;
 	data[2] = (int) msg.position.z * 1000;
 	data[3] = (int) msg.velocity.x * 1000;
 	data[4] = (int) msg.velocity.y * 1000;
 	data[5] = (int) msg.velocity.z * 1000;
 	crc_data = crc(data, 12);
 	data_send[0] = data[0];
 	data_send[1] = data[0] >> 8;
 	data_send[2] = data[1];
 	data_send[3] = data[1] >> 8;
 	data_send[4] = data[2];
 	data_send[5] = data[2] >> 8;
 	data_send[6] = data[3];
 	data_send[7] = data[3] >> 8;
 	data_send[8] = data[4];
 	data_send[9] = data[4] >> 8;
 	data_send[10] = data[5];
 	data_send[11] = data[5] >> 8;
 	data_send[12] = crc_data;
 	data_send[13] = crc_data >> 8;
}

int set_serial(int fd,int nSpeed, int nBits, char nEvent, int nStop)  
{  
	struct termios newtio,oldtio;  
	if( tcgetattr( fd,&oldtio)  !=  0) 
	{   
		ROS_INFO("SetupSerial 1");  
		return -1;  
	}  
	bzero( &newtio, sizeof( newtio ) );  
	newtio.c_cflag  |=  CLOCAL | CREAD;  
	newtio.c_cflag &= ~CSIZE;  

	switch( nBits )  
	{  
		case 7:  
			newtio.c_cflag |= CS7;  
			break;  
		case 8:  
			newtio.c_cflag |= CS8;  
			break;  
	}

	switch( nEvent )  
	{  
		case 'O':  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag |= PARODD;  
			newtio.c_iflag |= (INPCK | ISTRIP);  
			break;  
		case 'E':   
			newtio.c_iflag |= (INPCK | ISTRIP);  
			newtio.c_cflag |= PARENB;  
			newtio.c_cflag &= ~PARODD;  
	    	break;
		case 'N':    																									
			newtio.c_cflag &= ~PARENB;  
			break;  
	}  

	switch( nSpeed )  
	{  
		case 2400:  
			cfsetispeed(&newtio, B2400);  
			cfsetospeed(&newtio, B2400);  
			break;  
		case 4800:  
			cfsetispeed(&newtio, B4800);  
			cfsetospeed(&newtio, B4800);  
			break;
		case 9600:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
		case 19200:  
			cfsetispeed(&newtio, B19200);  
			cfsetospeed(&newtio, B19200);  
			break; 
		case 57600:  
			cfsetispeed(&newtio, B57600);  
			cfsetospeed(&newtio, B57600);  
			break; 
		case 115200:  
			cfsetispeed(&newtio, B115200);  
			cfsetospeed(&newtio, B115200);  
			break;  
		case 460800:  
			cfsetispeed(&newtio, B460800);  
			cfsetospeed(&newtio, B460800);  
			break;  
		default:  
			cfsetispeed(&newtio, B9600);  
			cfsetospeed(&newtio, B9600);  
			break;  
	}  

	if( nStop == 1 )  
	{
		newtio.c_cflag &=  ~CSTOPB;  
	}
	else if ( nStop == 2 )
	{  
		newtio.c_cflag |=  CSTOPB;  
	} 

	newtio.c_cc[VTIME]  = 100;
	newtio.c_cc[VMIN] = 0;
	tcflush(fd,TCIFLUSH);  
	if((tcsetattr(fd,TCSANOW,&newtio))!=0)  
	{  
		ROS_INFO("com set error!");  
		return -1;  
	}  
	return 0;  
} 

void serial_init()
{
	int ret;
	serial_fd = open(DEVICE, O_RDWR);
	if (serial_fd == -1)
	{
		ROS_INFO("Open Error!");  
		exit(1);    
	}  
	ret = set_serial(serial_fd, BAUDRATE, 8, 'N', 1);
	if (ret == -1)  
	{
		ROS_INFO("Set Serial Error!");  
		exit(1);  
	}
}

void send(char*data, int data_byte)
{
		int ret;
		ret = write(serial_fd,data,data_byte);
		if (ret == -1)  
		{
			ROS_INFO("write Error!");  
			exit(1);  
		}
}

/*crc*/
unsigned short crc_update(unsigned short  crc ,  unsigned char data)
{
	data ^= (crc & 0xff) ;
	data ^= data << 4;
	return ((((unsigned short)data << 8) | ((crc >> 8) & 0xff) )^ (unsigned char)(data >> 4) ^ ((unsigned short)data << 3)) ;
}

unsigned short crc(void* data, unsigned short count)
{
	unsigned short crc = 0xff;
	unsigned char *ptr = (unsigned char*)data;
	for(int i = 0 ; i < count ; i++)
	{
		crc = crc_update(crc , *ptr);
		ptr++;
	}
	return crc;
}