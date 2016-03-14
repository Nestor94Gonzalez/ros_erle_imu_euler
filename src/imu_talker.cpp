#include <stdio.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/time.h>
#include "MPU9250.h"
#include "AHRS.hpp"
#include "ros_erle_imu_euler/euler.h"
#include <ros/ros.h>

// Objects

MPU9250 imu;    // MPU9250
AHRS    ahrs;   // Mahony AHRS

// Sensor data

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

// Orientation data

float roll, pitch, yaw;

// Timing data

float offset[3];
struct timeval tv;
float dt;
unsigned long previoustime, currenttime;

//============================= Initial setup =================================

void imuSetup()
{
    //----------------------- MPU initialization ------------------------------

    imu.initialize();

    //-------------------------------------------------------------------------

	printf("Beginning Gyro calibration...\n");
	for(int i = 0; i<100; i++)
	{
		imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		offset[0] += (-gx*0.0175);
		offset[1] += (-gy*0.0175);
		offset[2] += (-gz*0.0175);
		usleep(10000);
	}
	offset[0]/=100.0;
	offset[1]/=100.0;
	offset[2]/=100.0;

	printf("Offsets are: %f %f %f\n", offset[0], offset[1], offset[2]);
	ahrs.setGyroOffset(offset[0], offset[1], offset[2]);
}

//=============================================================================

int main(int argc, char *argv[])
{
    //-------------------- IMU setup and main loop ----------------------------
    imuSetup();

    ros::init(argc, argv, "ros_erle_imu_euler");
    
    ros::NodeHandle n;

    ros::Publisher imu_euler_pub = n.advertise<ros_erle_imu_euler::euler>("euler", 1000);

    ros::Rate loop_rate(50);

    while (ros::ok()){

        //----------------------- Calculate delta time ----------------------------

        gettimeofday(&tv,NULL);
        previoustime = currenttime;
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;
        if(dt < 1/1300.0) 
            usleep((1/1300.0-dt)*1000000);
        gettimeofday(&tv,NULL);
        currenttime = 1000000 * tv.tv_sec + tv.tv_usec;
        dt = (currenttime - previoustime) / 1000000.0;

        //-------- Read raw measurements from the MPU and update AHRS --------------
        // Accel + gyro.
        imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        ahrs.updateIMU(ax, ay, az, gx*0.0175, gy*0.0175, gz*0.0175, dt);

        //------------------------ Read Euler angles ------------------------------

        ahrs.getEuler(&roll, &pitch, &yaw);

        ros_erle_imu_euler::euler msg;
        msg.roll = roll;
        msg.pitch = pitch;
        msg.yaw = yaw;
        imu_euler_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
