#include <cstdio>
#include <cstdint>

// ROS
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

// Dynamixel SDK
#include <dynamixel/dynamixel.h>


// Control table address
#define P_GOAL_POSITION_L		30
#define P_GOAL_POSITION_H		31

#define P_MOVING_SPEED_L 		32
#define P_MOVING_SPEED_H 		33

#define P_PRESENT_POSITION_L	36
#define P_PRESENT_POSITION_H	37

#define P_MOVING		46

// Defulat setting
#define BAUDNUM_OPTION		1 // 1Mbps
#define USB_ID				0

const uint8_t DXL_IDs[8] = {1, 2, 14, 3, 5, 6, 7, 8};
const float JOINT_MIN_LIMIT[8] = {  0,  25,   0,   0,  50,   0, 123, 50}; // Degrees
const float JOINT_MAX_LIMIT[8] = {300, 275, 300, 300, 250, 300, 250, 177}; // Degrees
const int DOFs = 6;

void new_data(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	// ROS_INFO("Message: ");
	// for(unsigned int i = 0; i < msg->data.size(); i++)
	// 		ROS_INFO("%lf", msg->data[i]);
	uint16_t pos;
	for(size_t i = 0; i < DOFs; i++)
	{
		if(msg->data[i] < JOINT_MIN_LIMIT[i] || msg->data[i] > JOINT_MAX_LIMIT[i])
		{
			ROS_WARN("Joint %f position is out of limits.", i);
			continue;
		}

		pos = uint16_t(1023 * (msg->data[i] / 300.0f));
		dxl_write_word(DXL_IDs[i], P_GOAL_POSITION_L, pos );
	}

	// Grasp
	if(msg->data.back() < JOINT_MIN_LIMIT[6] || msg->data.back() > JOINT_MAX_LIMIT[6])
	{
		ROS_WARN("End effector grasp position is out of limits.");
		return;
	}

	pos = uint16_t(1023 * (msg->data.back() / 300.0f));
	dxl_write_word(DXL_IDs[6], P_GOAL_POSITION_L, pos);

	pos = uint16_t(1023 * ((300 - msg->data.back()) / 300.0f));
	dxl_write_word(DXL_IDs[7], P_GOAL_POSITION_L, pos);
}

int main(int argc, char **argv)
{
	if( dxl_initialize(USB_ID, BAUDNUM_OPTION) == 0 )
	{
		printf( "Failed to open USB2Dynamixel!\n" );
		printf( "Press Enter key to terminate...\n" );
		getchar();
		return 0;
	}
	else
	{
		printf( "Succeed to open USB2Dynamixel!\n" );
	}

	//uint8_t data = dxl_read_byte(1, 0x03 );
	//printf("ID: %u\n", data);

	// write id: dxl_write_byte( 10, 0x03, 1 );

	// int pos;
	// while(1)
	// {
	// 	scanf("%d", &pos);

	// 	if(pos < 0 || pos > 1023) break;

	// 	dxl_write_word( 1, P_GOAL_POSITION_L, pos );
	// }

	ros::init(argc, argv, "dxl_arm_control");
	ros::NodeHandle n("~");
	ros::Subscriber sub = n.subscribe("joints", 1000, new_data);

	ros::Rate rate(100);

	while(ros::ok())
	{

		ros::spinOnce();
		rate.sleep();
	}

	dxl_terminate();
}
