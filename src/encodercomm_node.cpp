#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <string>

#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

#define NODE_NAME "encodercomm_node"
#define ADVERTISE_WHEEL_SPEED "wheel_velocity"
#define BUFER_SIZE 5
#define MESSAGE_LENGTH 12
#define MESSAGE_FREQUENCY 20

#define L_DIR_INDEX 1
#define R_DIR_INDEX 6
#define L_SPEED_INDEX 2
#define R_SPEED_INDEX 7
#define SOM 250
#define EOM 251


#define BAUDRATE B115200

int fileDescriptor;


std_msgs::Float32MultiArray wheel_velocities;
//std_msgs::MultiArrayLayout wheel_velocities_layout;

unsigned char serial_data[MESSAGE_LENGTH];

long char_arr_to_long(unsigned char data[]);

int main(int argc, char **argv)
{

    ros::init(argc, argv, "encodercomm");
    ros::NodeHandle n;
    wheel_velocities.data.clear();
    ros::Publisher L_vel_pub = n.advertise<std_msgs::Float32MultiArray>(ADVERTISE_WHEEL_SPEED, BUFER_SIZE);
    ros::Rate loop_rate(MESSAGE_FREQUENCY);

    // The serial setup is mostly inspired by this:
    // https://stackoverflow.com/questions/38533480/c-libserial-serial-connection-to-arduino
    // Which sets things like parity, stop byte, length, etc.

    // This sets up the serial communication to the arduino driver.
    fileDescriptor = open("/dev/ttyACM0", O_RDWR | O_NOCTTY); //open link to arudino


    struct termios newtio;
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;

    // set to 8N1
    newtio.c_cflag &= ~PARENB;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~CSIZE;
    newtio.c_cflag |= CS8;

    newtio.c_iflag = IGNPAR;

    // output mode to
    //newtio.c_oflag = 0;
    newtio.c_oflag |= OPOST;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME] = 10; /* inter-character timer 1 sec */
    newtio.c_cc[VMIN] = 0; /* blocking read disabled  */

    tcflush(fileDescriptor, TCIFLUSH);
    if (tcsetattr(fileDescriptor, TCSANOW, &newtio)) {
        perror("could not set the serial settings!");
        return -99;
    }

    long L_speed = 0;
    long L_dir = 0;
    long R_speed = 0;
    long R_dir = 0;
    int badCount = 0;
    int readcount = 0;
    int rc = 0;
    // long L_vel exists
    while(ros::ok()){

    
	//ROS_INFO("encodercomm top of loop.");

	read(fileDescriptor, serial_data, sizeof(char));
	while(serial_data[0] != SOM){
		read(fileDescriptor, serial_data, sizeof(char));
		//ROS_INFO("looking for SOM :: %d",serial_data[0]);
		badCount++;
	}
	//ROS_INFO("Found SOM after %d bad cound.", badCount);
	badCount = 0;
	readcount = 1;
    
	while(readcount < MESSAGE_LENGTH){
		rc = read(fileDescriptor, &serial_data[readcount], (MESSAGE_LENGTH - readcount)*sizeof(char));
		readcount += rc;
		//ROS_INFO("Managed to read %d bytes",rc);
	}
	
	if(serial_data[MESSAGE_LENGTH-1] == EOM && (serial_data[L_DIR_INDEX] == 0 || serial_data[L_DIR_INDEX] == 1) ){ // data is good!, both SOM and EOM are there, also we do have a dir message in beginning of data.
		
		//ROS_INFO("Data is good!");
		L_speed = char_arr_to_long(&serial_data[L_SPEED_INDEX]);
		R_speed = char_arr_to_long(&serial_data[R_SPEED_INDEX]);
		L_dir = serial_data[L_DIR_INDEX];
		R_dir = serial_data[R_DIR_INDEX];

		wheel_velocities.data.push_back(1.0 * (L_dir*2 - 1) * MESSAGE_FREQUENCY * L_speed / 6337);
		wheel_velocities.data.push_back(1.0 * (R_dir*2 - 1) * MESSAGE_FREQUENCY * R_speed / 6337);

		//L_vel.data =  // experimental data from encoders
		//R_vel.data = 1.0 * (R_dir*2 - 1) * MESSAGE_FREQUENCY * R_speed / 6337; // experimental data from encoders

		//ROS_INFO("%f %f", L_vel, R_vel);
		L_vel_pub.publish(wheel_velocities);
		wheel_velocities.data.clear();
		

	} else { // data is bad...
		ROS_INFO("Data is bad...");
		ROS_INFO("%d %d %d %d %d %d %d %d %d %d %d %d", serial_data[0],serial_data[1],serial_data[2],serial_data[3],serial_data[4],serial_data[5],serial_data[6],serial_data[7],serial_data[8],serial_data[9],serial_data[10],serial_data[11] );
 		
 		
	}

 	ros::spinOnce();
 	loop_rate.sleep();

    }

  return 0;
}


// data over serial comes in bytes, but info may be a split up long, here we join together that.
long char_arr_to_long(unsigned char data[]){ // PLEASE! Make sure incoming data has at least 4 bytes... *COUGH SegFault...*

	return ((long)data[0]  << 24) +
		((long)data[1] << 16) +
		((long)data[2] << 8) +
		(long)data[3];

}

