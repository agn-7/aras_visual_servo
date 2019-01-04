#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <strstream>
#include <sstream>
#include <exception>
#include <boost/thread.hpp>
#include <sensor_msgs/JointState.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
//#include <string>

#include "image_transport/subscriber.h"
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>

#define JOINTS_NUM 6
#define KERNEL_SIZE 4
#define DOF 4

#define CAMERA_DATA_SIZE 4
//#define IMAGE_ROW        240
//#define IMAGE_COLUMN     320

#define LAMBDA_X 140;//200
#define LAMBDA_Y 120;//200
#define LAMBDA_Z 60;//200
#define LAMBDA_THETA 30;//-0.01

#define L1  0.3
#define L2  0.25
#define L3  0.16
#define L5  0.072
#define SAMPLE_TIME 0.01
#define JOINTS_ERROR 0.01
class VisualServoController
{
public:
    VisualServoController();
    ~VisualServoController();
    bool Spin();
    /*set joints positions (in radian and m)*/
    bool setJointPositions(float target_positions[JOINTS_NUM]);
    float* getJointPositions();
    void setTargetPositions(float target_positions[JOINTS_NUM]);
    void setInitialPositions(float target_positions[JOINTS_NUM]);
    void hardSetJointPosition(float target_positions[JOINTS_NUM]);
    void executeControlAlgorithm();
    void forwardKinematic(float target_joints[JOINTS_NUM], float &x, float &y, float &z, float &a);
    float z_star;
    float m00_star;
    float Target_x;
    float Target_y;
    float Target_z;
    float Target_yaw;
private:
    bool camera_call_backed_;

    void initializeSubscribers();
    void initializePublishers();

    /* callback function for getting joint states */
    void jointStateCB(const sensor_msgs::JointStatePtr& joint_states);
    void cameraDataCB(const std_msgs::Float64MultiArray::ConstPtr &camera_data_arr);


    void inverseKinematic(float target_joints[JOINTS_NUM], float x, float y, float z,float a);

    void SingularityAvoidance(CvMat* xdesired_dot,float lgantry,float teta1,
                                                     float teta2,float teta3,float teta5,float teta6,
                                                     float gain,float k_bound,float lee,CvMat* qdesired_dot);


    void cvSubb_soli(CvMat* measurement,CvMat* nonlinear_measurement,CvMat* Z_G);
    float joints_position_[JOINTS_NUM];
    float target_joints_position_[JOINTS_NUM];

    FILE  *fp_s, *fp_v, *fp_es, *fp_joints, *fp_cartesian, *fp_sliding, *fp_sums, *fp_sdot;
    image_transport::Subscriber threshold_image_sub_;

    double target_kernel_[KERNEL_SIZE];
    cv::Mat jacobian_inverse_mat_;
    double kernel_[KERNEL_SIZE];

    ros::NodeHandle nh_;
    ros::Publisher joint_pub_[JOINTS_NUM];
    ros::Subscriber joint_state_sub_;
    ros::Subscriber camera_data_sub_;


};
