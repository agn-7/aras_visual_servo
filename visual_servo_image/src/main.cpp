//#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "string"
#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"
#include <iostream>
#include <termios.h>
#include <math.h>
#include "visual_servo_image/Cam_Sensor.h"
#include "visual_servo_image/Controller.h"
#include "visual_servo_image/Robot.h"

using namespace std;
using namespace cv;

#define Precision 0.1

// Declaire CAMERA and ROBOT objects as global
// beacuse they are used in imageCallback and Joint_StateCallback.
CamSensorManager CAMERA;
// Global variables
Mat Image;
IplImage Ipl_Image;
float joints[6]; // Get Position
float Gantry_Pos; // Current Cantry Position
float Current_Position[6]; // for kinematic added by parisa

// These variables are used for debuging
int Counter_Image = 0;
int Counter_joint = 0;
FILE  *fp_Current_Joint;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
//        cout << "Image Updated" << Counter_Image++ << endl;
        Image = cv_bridge::toCvShare(msg, "bgr8")->image;
        Ipl_Image = (IplImage)Image;
        if (CAMERA.Image_Captured == false)
        {
            CAMERA.InitializeCamera();
            //CAMERA.StartCamera();
            CAMERA.Image_Captured = true;
        }
        CAMERA.ImageShow();

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void Joint_StateCallback(const sensor_msgs::JointStatePtr& msg)
{

    Gantry_Pos = (msg->position[0])*(100); //Gantry Position(cm)
    joints[0] = (msg->position[1])*180/3.14159265; //J1
    joints[1] = (msg->position[2])*180/3.14159265; //J2
    joints[2] = (msg->position[3])*180/3.14159265; //J3
    joints[3] = 0; //J4, Note used
    joints[4] = (msg->position[4])*180/3.14159265; //J5
    joints[5] = (msg->position[5])*180/3.14159265; //J6
    fprintf(fp_Current_Joint,"%f %f %f %f %f %f\n",joints[0], joints[1], joints[2],
                        joints[4], joints[5], Gantry_Pos);

//    cout << "Joint Updated" << Counter_joint++ << ": "<< joints[0]
//         << "  " << joints[1] << "  " << joints[2] << "  " << joints[3]
//         << "  " << joints[4] << "  " << joints[5] << "  " << Gantry_Pos
//         << endl;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  startWindowThread();
  image_transport::ImageTransport it(nh);

  // Define of Subscribers
  image_transport::Subscriber sub = it.subscribe("/labrob/camera/image_raw", 1, imageCallback);
  ros::Subscriber Join_State_Sub = nh.subscribe("/visual_servo/joint_states", 10, Joint_StateCallback);

  std_msgs::Float64 msg;

//  char Input_Key;
//  char Input_Joint_Value1[8],Input_Joint_Value2[8],Input_Joint_Value3[8];
//  char Input_Joint_Value5[8],Input_Joint_Value6[8],Input_Gantry[8];

  ControllerManager CONTROLLER;
  RobotManager ROBOT;
  CONTROLLER.program_define= 3;
  ROBOT.nh = nh;
  ROBOT.StartRobot();

  fp_Current_Joint = fopen("/home/parisa/catkin_ws/src/aras_visual_servo/Results/PID_Test.m","w");
  fprintf(fp_Current_Joint,"Joint = [   %%Joints value\n %%J1 J2 J3 J4 J5 J6 Gantry\n");

  // Get address of obtained image to CAMERA object
  CAMERA.Image = &Ipl_Image;
  CAMERA.Image_Captured = false;
  while (CAMERA.Image_Captured == false) {
      ros::spinOnce();
  }

  // Set Target position for ROBOT
  ROBOT.target[0] = 0;//J1
  ROBOT.target[1] = 0;//J2
  ROBOT.target[2] = 0;//J3
  ROBOT.target[3] = 0;// is zero
  ROBOT.target[4] = 0;
  ROBOT.target[5] = 0;
  ROBOT.Gantry_Target = 0;
  do
  {
      ros::spinOnce();
  }
  while( (abs(joints[0] - ROBOT.target[0]) > Precision) || (abs(joints[1] - ROBOT.target[1]) > Precision) ||
         (abs(joints[2] - ROBOT.target[2]) > Precision) || (abs(joints[3] - ROBOT.target[3]) > Precision) ||
         (abs(joints[4] - ROBOT.target[4]) > Precision) || (abs(joints[5] - ROBOT.target[5]) > Precision) ||
         (abs(Gantry_Pos - ROBOT.Gantry_Target) > Precision));
  boost::this_thread::sleep( boost::posix_time::milliseconds(2000));

  CONTROLLER.Kinematic(ROBOT.target[0], ROBOT.target[1], ROBOT.target[2], ROBOT.target[4],
      ROBOT.target[5], (ROBOT.Gantry_Target/10),Current_Position);
  cout<<"Current_Position= "<<Current_Position[0]<<" "<<Current_Position[1]<<" "<<Current_Position[2]<<
        " "<<Current_Position[3]<<" "<<Current_Position[4]<<" "<<Current_Position[5]<<endl;

      CONTROLLER.z_star = (101 - Current_Position[2])/100;


  // Providing requirment information for controlller
  CONTROLLER.Thresholded_Image = CAMERA.Thresholded_Image;
  CONTROLLER.Scale_Furier_Image = CAMERA.Scale_Furier_Image;
  CONTROLLER.Scale_Furier_Image_z = CAMERA.Scale_Furier_Image_z;

  // Set this Position as target
  for(int i = 0; i < 6; i++)
      CONTROLLER.Current_Joints[i] = joints[i];
  CONTROLLER.Current_Gantry_Pos = Gantry_Pos;
  CONTROLLER.SetTargetPosition();
  std::cout << "Set this position as Target" << std::endl;
  boost::this_thread::sleep( boost::posix_time::milliseconds(2000));

  char *Path = new char[100];
  sprintf(Path, "/home/parisa/catkin_ws/src/aras_visual_servo/Results/Target_Image.jpg");
  cvSaveImage(Path ,CAMERA.Thresholded_Image);

  // Set initialize Position for robot
  ROBOT.target[0] = 0;//0;
  ROBOT.target[1] = 10;//20;
  ROBOT.target[2] = -5;//25;//-10;
  ROBOT.target[3] = 0;
  ROBOT.target[4] = -5;//15;//-10;
  ROBOT.target[5] = 5;//7;
  ROBOT.Gantry_Target = 3;//2;
  do
  {
      ros::spinOnce();
  }
  while( (abs(joints[0] - ROBOT.target[0]) > Precision) || (abs(joints[1] - ROBOT.target[1]) > Precision) ||
         (abs(joints[2] - ROBOT.target[2]) > Precision) || (abs(joints[3] - ROBOT.target[3]) > Precision) ||
         (abs(joints[4] - ROBOT.target[4]) > Precision) || (abs(joints[5] - ROBOT.target[5]) > Precision) ||
         (abs(Gantry_Pos - ROBOT.Gantry_Target) > Precision));


  for(int i = 1; i < 10; i++)
  {
      boost::this_thread::sleep( boost::posix_time::milliseconds(300));
      ros::spinOnce();
  }
  char *Path2 = new char[100];
  sprintf(Path2, "/home/parisa/catkin_ws/src/aras_visual_servo/Results/Initial_Image.jpg");
      cvSaveImage(Path2 ,CAMERA.Thresholded_Image);
  fprintf(fp_Current_Joint,"];\n");
  //fclose(fp_Current_Joint);
  //cout << "End Of Test."<< endl;
 // cvDestroyAllWindows();
//  while(ros::ok());


  //ros::Rate loop_rate(1000);

      uint counter_loop = 0;
      while (counter_loop < 400)
      {
          boost::this_thread::sleep( boost::posix_time::milliseconds(100));
          counter_loop++;
          cout << "counter_loop:::" << counter_loop;
          //Take current joint and gantry from camera(Sensor)
          //and give them to the controller
          for( int i = 0; i < 6; i++)
              CONTROLLER.Current_Joints[i] = joints[i];
          CONTROLLER.Current_Gantry_Pos = Gantry_Pos;

          // Comput Control signal
          CONTROLLER.ComputeControlSignal();

          //Take target joint and gantry position and give them to Robot(Actuators)
          for(int i = 0; i<6; i++)
              ROBOT.target[i] = CONTROLLER.Target_Joints[i];
          ROBOT.Gantry_Target = CONTROLLER.Target_Gantry_Pos;
          //ros::spinOnce();
          //loop_rate.sleep();
          do
          {
              ros::spinOnce();
//              cout << "Joint Updated" << Counter_joint++ << ": "<< abs(joints[0] - ROBOT.target[0])
//                   << "  " << abs(joints[1] - ROBOT.target[1]) << "  " << abs(joints[2] - ROBOT.target[2]) << "  " << abs(joints[3] - ROBOT.target[3])
//                   << "  " << abs(joints[4] - ROBOT.target[4]) << "  " << abs(joints[5] - ROBOT.target[5]) << "  " << abs(Gantry_Pos - ROBOT.Gantry_Target)
//                   << endl;
          }
          while( (abs(joints[0] - ROBOT.target[0]) > Precision) || (abs(joints[1] - ROBOT.target[1]) > Precision) ||
                 (abs(joints[2] - ROBOT.target[2]) > Precision) || (abs(joints[3] - ROBOT.target[3]) > Precision) ||
                 (abs(joints[4] - ROBOT.target[4]) > Precision) || (abs(joints[5] - ROBOT.target[5]) > Precision) ||
                 (abs(Gantry_Pos - ROBOT.Gantry_Target) > Precision));
      }
      char *Path3 = new char[100];
      sprintf(Path3, "/home/parisa/catkin_ws/src/aras_visual_servo/Results/Final_Image.jpg");
      cvSaveImage(Path3 ,CAMERA.Thresholded_Image);
      CONTROLLER.Terminate();
      cout << "End Of Visual Servoing."<< endl;
      cvDestroyAllWindows();
      while(ros::ok());


}

