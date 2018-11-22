#pragma once

#include <ros/ros.h>

#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "string"
#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"
#include <iostream>

using namespace std;

class RobotManager
{
public:

	RobotManager();

	~RobotManager();

   // void Joint_StateCallback(const sensor_msgs::JointStatePtr&);
    void Initialize();
    void StartRobot(); // Start a Thread for
                       // getting and setting joint values

	boost::thread mThread;

	boost::mutex mMutex;

    ros::NodeHandle nh;
    float target[6]; // Set Positoon
	float Gantry_Target; // target position that must be set
	
};
