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
const int DOFs = 6;

void new_data(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	// ROS_INFO("Message: ");
	// for(unsigned int i = 0; i < msg->data.size(); i++)
	// 		ROS_INFO("%lf", msg->data[i]);

	for(size_t i = 0; i < DOFs; i++)
	{
		if(msg->data[i] < 0 || msg->data[i] > 300.0f)
		{
			ROS_WARN("Joint position is out of limits.");
			continue;
		}

		uint16_t pos = uint16_t(1023 * (msg->data[i] / 300.0f));
		dxl_write_word(DXL_IDs[i], P_GOAL_POSITION_L, pos );
	}

	// Grasp
	msg->data.size()
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
