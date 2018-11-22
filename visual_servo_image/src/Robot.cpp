#include "Robot.h"
//#include <iostream>


RobotManager::RobotManager()
{
}

RobotManager::~RobotManager()
{
}

void RobotManager::Initialize()
{
    // Define of Publishers
    ros::Publisher chatter_pub0 = nh.advertise<std_msgs::Float64>("/visual_servo/joint0_position_controller/command", 10);
    ros::Publisher chatter_pub2 = nh.advertise<std_msgs::Float64>("/visual_servo/joint2_position_controller/command", 10);
    ros::Publisher chatter_pub3 = nh.advertise<std_msgs::Float64>("/visual_servo/joint3_position_controller/command", 10);
    ros::Publisher chatter_pub4 = nh.advertise<std_msgs::Float64>("/visual_servo/joint4_position_controller/command", 10);
    ros::Publisher chatter_pub5 = nh.advertise<std_msgs::Float64>("/visual_servo/joint5_position_controller/command", 10);
    ros::Publisher chatter_pub6 = nh.advertise<std_msgs::Float64>("/visual_servo/joint6_position_controller/command", 10);

    std_msgs::Float64 msg;
    // Initialize Default target values
    Gantry_Target = 0;
    for(int i = 0; i < 6; i++)
    {
        target[i] = 0;
    }
    ros::Rate loop_rate(10);
    while (ros::ok()) {

        msg.data = Gantry_Target/100.0;
        chatter_pub0.publish(msg); // Gantry Position(m)

        msg.data = target[0]*(3.14159265/180.0);
        chatter_pub2.publish(msg); // J1, based on markers on Real Robot

        msg.data = target[1]*(3.14159265/180.0);
        chatter_pub3.publish(msg); // J2

        msg.data = target[2]*(3.14159265/180.0);
        chatter_pub4.publish(msg); // j3

        msg.data = target[4]*(3.14159265/180.0);
        chatter_pub5.publish(msg); // j5

        msg.data = target[5]*(3.14159265/180.0);
        chatter_pub6.publish(msg); // j6, Camera joint

        loop_rate.sleep();
       // ros::spinOnce();
    }

}

void RobotManager::StartRobot()
{
	
    mThread = boost::thread(&RobotManager::Initialize, this);
}

//void RobotManager::RxWorkerFunction()
//{
	
//	char* byte = new char[1];
//	DWORD dwBytesTransferred;
	
//	this->Send(hFile4, "posa home\r", 10);
//	for(int i = 0; i < 3; )	// Read Response, empty buffer(use in ReadPosition method)
//	{
//		ReadFile(hFile4, byte, 1, &dwBytesTransferred, 0);
//		if (dwBytesTransferred == 1)
//			i++;
//	}
//	// wait for going to home. gantry is so slow
//	boost::this_thread::sleep( boost::posix_time::seconds(10));
//	while(1)
//	{
//		//std::cout << "Set Position to:" << std::endl;
//		//for(int j = 0; j < 6; j++)  // Print target joints
//		//	std::cout << target[j] << " , ";
//		//std::cout << std::endl;
//		this->SetPosition(); // Call SetPosition Method
//		boost::this_thread::sleep( boost::posix_time::milliseconds(1));
//		this->ReadPosition(); // Call ReadPosition Method
//		//std::cout << "Readed Position:" << std::endl;
//		//for(int j = 0; j < 6; j++) // Print Readed joints
//		//	std::cout << joints[j] << " , ";
//		//std::cout << std::endl;
//		//std::cout << "Readed Gantry Position:" << std::endl;
//		//std::cout << Gantry_Pos << std::endl;

//		boost::this_thread::sleep( boost::posix_time::milliseconds(1));
//	}
//}
