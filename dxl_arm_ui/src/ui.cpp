#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

#include <opencv2/opencv.hpp>

const int DOFs = 7;

bool send_pos = true;
int joint_pos[DOFs];


void on_trackbar(int value, void* data)
{
	send_pos = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dxl_arm_ui");
	ros::NodeHandle n("~");
	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("/dxl_arm_control/joints", 1000);
	std_msgs::Float32MultiArray msg;
	msg.data.resize(DOFs);

	for(int i = 0; i < DOFs; ++i)
	{
		joint_pos[i] = 150;
	}

	cv::namedWindow("Control Panel", CV_GUI_EXPANDED);
	cv::createTrackbar("Joint 1", "Control Panel", joint_pos + 0, 300, on_trackbar, NULL);
	cv::createTrackbar("Joint 2", "Control Panel", joint_pos + 1, 300, on_trackbar, NULL);
	cv::createTrackbar("Joint 3", "Control Panel", joint_pos + 2, 300, on_trackbar, NULL);
	cv::createTrackbar("Joint 4", "Control Panel", joint_pos + 3, 300, on_trackbar, NULL);
	cv::createTrackbar("Joint 5", "Control Panel", joint_pos + 4, 300, on_trackbar, NULL);
	cv::createTrackbar("Joint 6", "Control Panel", joint_pos + 5, 300, on_trackbar, NULL);
	cv::createTrackbar(" Grasp ", "Control Panel", joint_pos + 6, 300, on_trackbar, NULL);

	while(ros::ok())
	{
		if (send_pos)
		{
			send_pos = false;

			for(int i = 0; i < DOFs; ++i)
			{
				msg.data[i] = joint_pos[i];
			}

			pub.publish(msg);
		}

		ros::spinOnce();
		cv::waitKey(5);
	}
}

