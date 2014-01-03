#include "dxl_arm_plugin/dxl_arm_plugin.h"
#include "v_repLib.h"

#include <string>
#include <iostream>
#include <cmath>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

LIBRARY vrepLib; // the V-REP library that will be dynamically loaded and binded

const double RAD_TO_DEG = (180.0 / M_PI);

const int DOFs =		7;
const int JOINT_COUNT =	6;

int joint_handles[JOINT_COUNT] = {0};
std_msgs::Float32MultiArray msg;

ros::NodeHandle* node;
ros::Publisher pub;

void getJointHandles()
{
	std::stringstream ss;
	bool error = false;

	for (int i = 1; i <= 6; ++i)
	{
		ss.str("");
		ss << "Joint" << i;
		joint_handles[i-1] = simGetObjectHandle(ss.str().c_str());
		if (joint_handles[i-1] == -1)
		{
			error = true;
			ROS_WARN("Unable to retrieve Joint%d handle.", i);
		}
	}

	if (!error) ROS_INFO("Finished reading joint handles.");
}

// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];

	char * filenptr = getcwd(curDirAndFile, sizeof(curDirAndFile));
	if (filenptr == NULL)
	{
	        std::cout << "[V-REP_KEPLER] Cannot get current directory." << std::endl;
	        return(0); // Means error, V-REP will unload this plugin
	}
	std::string currentDirAndPath(curDirAndFile);

	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
	temp+="/libv_rep.so";

	// 3. Load the V-REP library:
	vrepLib=loadVrepLibrary(temp.c_str());
	if (vrepLib == NULL)
	{
	        std::cout << "[V-REP_KEPLER] Cannot find or correctly load the V-REP library." << std::endl;
	        return (0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib)==0)
	{
	        std::cout << "[V-REP_KEPLER] Could not find all required functions in the V-REP library." << std::endl;
	        unloadVrepLibrary(vrepLib);
	        return (0); // Means error, V-REP will unload this plugin
	}
	// ******************************************


	// Here you could handle various initializations
	std::cout << "[V-REP_DXL_ARM] Initializing v-rep plugin..." << std::endl;

	int argc = 0;
	char** argv = NULL;
	ros::init(argc, argv, "v_rep_dxl_arm_plugin");

	if (!ros::master::check())
	{
		ROS_ERROR("[V-REP_DXL_ARM] Cannot detect ROS master!");
		return 0;
	}

	node = new ros::NodeHandle();
	pub = node->advertise<std_msgs::Float32MultiArray>("/dxl_arm_control/joints", 1000);
	msg.data.resize(DOFs);


	ROS_INFO("[V-REP_DXL_ARM] Created node...");
	return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd()
{
	// Here you could handle various clean-up tasks
	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    // This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 5 lines at the beginning and unchanged:
	simLockInterface(1);
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode,&errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,sim_api_errormessage_ignore);
	void* retVal = NULL;


	// Here we can intercept many messages from V-REP (actually callbacks).
	// Only the most important messages are listed here:
	if (message==sim_message_eventcallback_simulationabouttostart)
	{
	    // Simulation is about to start
		ROS_INFO("[V-REP_DXL_ARM] Simulation is about to start.");
		getJointHandles();
	}

	if (message==sim_message_eventcallback_mainscriptabouttobecalled)
	{
		std::cout << " ---- ---- ---- " << std::endl;
	    // A simulation iteration is about to take place
		for (int i = 0; i < JOINT_COUNT; ++i)
		{
			simGetJointPosition(joint_handles[i], &msg.data[i]);
			msg.data[i] *= RAD_TO_DEG;
			std::cout << msg.data[i] << std::endl;
		}
		msg.data[JOINT_COUNT] = 0; // Grasp -> center position

		pub.publish(msg);
	}

	if (message==sim_message_eventcallback_simulationended)
	{
	    // Simulation just ended
	    ROS_INFO("[V-REP_DXL_ARM] Simulation just ended.");
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode, errorModeSaved); // restore previous settings
	simLockInterface(0);
	return (retVal);
}

