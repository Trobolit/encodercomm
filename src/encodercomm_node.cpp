#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <string>
#include <fcntl.h> // File control definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

#define NODE_NAME "encodercomm_node"
#define ADVERTISE_WHEEL_SPEED "wheel_velocity"
#define BUFER_SIZE 5
#define MESSAGE_LENGTH 12
#define MESSAGE_FREQUENCY 20
#define SERIAL_PATH "/dev/ttyACM2"

#define L_DIR_INDEX 1
#define R_DIR_INDEX 6
#define L_SPEED_INDEX 2
#define R_SPEED_INDEX 7
#define SOM 250
#define EOM 251
#define BAUDRATE B115200

// structs
struct encoder_data {
    long L_speed;
    long L_dir;
    long R_speed;
    long R_dir;
};

struct wheel_velocities {  // SI unit!
    float L_vel;
    float R_vel; 
};

// globals
int fileDescriptor;
std_msgs::Float32MultiArray wheel_velocities_msg;
unsigned char serial_data[MESSAGE_LENGTH];


// function declarations
long char_arr_to_long(unsigned char data[]);
void init_encoder_data(encoder_data *encdata);
void init_wheel_velocities(wheel_velocities *wv);
void parse_incoming_message(encoder_data *md, unsigned char sd[]);
void change_to_SI_units(encoder_data *md, wheel_velocities *wv);


int main(int argc, char **argv)
{
    ///// Start node //////
    ros::init(argc, argv, "encodercomm");
    ros::NodeHandle n;
    wheel_velocities_msg.data.clear();
    ros::Publisher vel_pub = n.advertise<std_msgs::Float32MultiArray>(ADVERTISE_WHEEL_SPEED, BUFER_SIZE);
    ros::Rate loop_rate(MESSAGE_FREQUENCY);


    ////// Setup Serial Link ///////

    // The serial setup is mostly inspired by this:
    // https://stackoverflow.com/questions/38533480/c-libserial-serial-connection-to-arduino
    // Which sets things like parity, stop byte, length, etc.

    fileDescriptor = open(SERIAL_PATH, O_RDWR | O_NOCTTY); //open link to arudino
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


    /////// Setup variables for loop /////////

    encoder_data encdata; 
    wheel_velocities wheelvels;
    init_encoder_data(&encdata);
    init_wheel_velocities(&wheelvels);

    int badCount = 0;	// used when debugging serial connection
    int readcount = 0;  // sucessfully read bytes from serial
    int rc = 0;		// local storage of readcount



    //////  Main loop of this node //////////

    while(ros::ok()) {

	// read serial data from serial port until a SOM is found
	read(fileDescriptor, serial_data, sizeof(char)); 	 // read one byte.
	while(serial_data[0] != SOM){			 	 // loop until a SOM is found.
		read(fileDescriptor, serial_data, sizeof(char)); // read one byte.
		badCount++;
	}
	badCount = 0;
	readcount = 1; // we now have one valid SOM in buffer.
    
	
	// Read serial data until a full message is in buffer
	while(readcount < MESSAGE_LENGTH){
		rc = read(fileDescriptor, &serial_data[readcount], (MESSAGE_LENGTH - readcount)*sizeof(char));
		readcount += rc;
	}
	
	// check that incoming data looks like a valid message. If so parse it.
	if(serial_data[MESSAGE_LENGTH-1] == EOM && (serial_data[L_DIR_INDEX] == 0 || serial_data[L_DIR_INDEX] == 1) ){ // data is good!, both SOM and EOM are there, also we do have a dir message in beginning of data.
		
		parse_incoming_message(&encdata, serial_data);		// parse buffer into useful datatypes
		change_to_SI_units(&encdata, &wheelvels);		// convert into useful units

		wheel_velocities_msg.data.push_back(wheelvels.L_vel);	// add info to message (array) to be publishes
		wheel_velocities_msg.data.push_back(wheelvels.R_vel);

		vel_pub.publish(wheel_velocities_msg);			// publish data
		wheel_velocities_msg.data.clear();			// clear message array for next iteration
		

	} else { // data is bad...
		ROS_INFO("Data is bad...");
		ROS_INFO("%d %d %d %d %d %d %d %d %d %d %d %d", serial_data[0],serial_data[1],serial_data[2],serial_data[3],serial_data[4],serial_data[5],serial_data[6],serial_data[7],serial_data[8],serial_data[9],serial_data[10],serial_data[11] );
	}

 	ros::spinOnce();
 	loop_rate.sleep();

    }

  return 0;
}



//////  Other functions  ////////


// data over serial comes in bytes, but info may be a split up long, here we join together that.
long char_arr_to_long(unsigned char data[]){ // PLEASE! Make sure incoming data has at least 4 bytes... *COUGH SegFault...*

	return ((long)data[0]  << 24) +
		((long)data[1] << 16) +
		((long)data[2] << 8) +
		(long)data[3];

}

void init_encoder_data(encoder_data *encdata) {
    encdata->L_speed = 0;
    encdata->L_dir = 0;
    encdata->R_speed = 0;
    encdata->R_dir = 0;
}
void init_wheel_velocities(wheel_velocities *wv) {
    wv->L_vel = 0;
    wv->R_vel = 0;
}

void parse_incoming_message(encoder_data *md, unsigned char sd[]) {
	md->L_speed = char_arr_to_long(&sd[L_SPEED_INDEX]);
	md->R_speed = char_arr_to_long(&sd[R_SPEED_INDEX]);
	md->L_dir = sd[L_DIR_INDEX];
	md->R_dir = sd[R_DIR_INDEX];
}

// These numbers come from experimental data. I.e. this approximates the actual velocities of the wheels over ground.
void change_to_SI_units(encoder_data *md, wheel_velocities *wv) {
	wv->L_vel = 1.0 * (md->L_dir*2 - 1) * MESSAGE_FREQUENCY * md->L_speed / 6337;
	wv->R_vel = 1.0 * (md->R_dir*2 - 1) * MESSAGE_FREQUENCY * md->R_speed / 6337;
}

