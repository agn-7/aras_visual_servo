#include <aras_visual_servo_controller/aras_visual_servo_controller.h>
#include <ros/ros.h>
#include "csignal"

void signalHandler( int signum )
{
    ROS_INFO("Interrupt signal (%d) received.\n",signum);
    exit(signum);
}

int main(int argc, char *argv[])
{
    using namespace std;
    ros::init(argc,argv,"aras_visual_servo_controller");
    VisualServoController *visual_servo_controller = new VisualServoController();
    signal(SIGINT,signalHandler);




    float joint_positions[6];
    joint_positions[0]=0.05;
    joint_positions[1]=0;
    joint_positions[2]=-0.09;
    joint_positions[3]=0.045;
    joint_positions[4]=0.045;
    joint_positions[5]=0;
    visual_servo_controller->forwardKinematic(joint_positions, visual_servo_controller->Target_x,
                                              visual_servo_controller->Target_y,
                                              visual_servo_controller->Target_z,
                                              visual_servo_controller->Target_yaw);
    visual_servo_controller->z_star = 0.91 - visual_servo_controller->Target_z;
    cout<<"z_star= "<<visual_servo_controller->z_star<<endl;    
    visual_servo_controller->setTargetPositions(joint_positions);
    //visual_servo_camera->Flag_Target_Position = false;
    joint_positions[0]=-0.1;//0.1;
    joint_positions[1]=0;//5.0*M_PI/180.0;
    joint_positions[2]=20.0*M_PI/180.0;
    joint_positions[3]=-15.0*M_PI/180.0;
    joint_positions[4]=-5.0*M_PI/180.0;
    joint_positions[5]=10*M_PI/180.0;
    visual_servo_controller->setInitialPositions(joint_positions);
//    char *Path = new char[50];
//	sprintf(Path, "D:\\Parisa\\SerialTest_begin again_sliding mode2\\Results\\Target_Image.jpg");
//	cvSaveImage(Path ,CAMERA.Thresholded2_Image);
    visual_servo_controller->executeControlAlgorithm();
    delete visual_servo_controller;
    return 0;
}
