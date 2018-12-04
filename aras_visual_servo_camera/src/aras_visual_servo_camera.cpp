#include "aras_visual_servo_camera/aras_visual_servo_camera.h"
//#include "../aras_visual_servo_camera/include/aras_visual_servo_camera/aras_visual_servo_camera.h"
VisualServoCamera::VisualServoCamera()
{
    image_transport::ImageTransport it(nh_);
    image_sub_ = it.subscribe("/labrob/camera/image_raw", 1, &VisualServoCamera::imageCB,this);
    threshold_image_pub_ = it.advertise("/aras_visual_servo/camera/thresholded_image", 1);
    camera_data_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/aras_visual_servo/camera/data",1);
    cv::namedWindow( "Orginal Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow( "Thresholded Image", CV_WINDOW_AUTOSIZE );
    cv::moveWindow("Orginal Image",0,0);
    cv::moveWindow("Thresholded Image",0,320);
}

void VisualServoCamera::imageCB(const sensor_msgs::ImageConstPtr &image_msg)
{
    try
    {
        color_image_ = cv_bridge::toCvShare(image_msg, "bgr8")->image;
        cv::cvtColor(color_image_, grey_image_, CV_BGR2GRAY);
        cv::threshold( grey_image_, threshold_image_, THRESHOLD_VALUE, MAX_BINARY_VALUE ,cv::THRESH_BINARY_INV );

        //cv::Mat jacobian_mat(KERNEL_SIZE,DOF,cv::DataType<double>::type);
        //cv::Mat jacobian_inverse_mat(KERNEL_SIZE,DOF,cv::DataType<double>::type);
        float kernel[4];
        calculateKernel(&threshold_image_,kernel);
        publishCameraData(kernel);

        cv::imshow("Original Image",color_image_);
        cv::imshow("Thresholded Image",threshold_image_);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_msg->encoding.c_str());
    }

}

bool VisualServoCamera::calculateKernel(const cv::Mat *image, float kernel[KERNEL_SIZE])
{
    //using namespace std;
    float fx = 195.0507;
    float fy = 195.0992;
    float cx = 160;
    float cy = 120;
    float x,y,tan2;


    cv::Moments image_moments = cv::moments(*image);
    y                                      = image_moments.mu11;
    x                                      = image_moments.mu20- image_moments.mu02;
    //just in case
    kernel[0] = ((image_moments.m01/image_moments.m00)-cy)/fy;
    kernel[1] = ((image_moments.m10/image_moments.m00)-cx)/fx;
    kernel[2] = image_moments.m00;

    if(abs(x) <= 0.004 && abs(y) <= 0.00006)
    {
        tan2 = 0*(M_PI/180);
    }

    else if(abs(x) <= 0.00003 && y > 0.004)
    {
        tan2 = 45*(M_PI/180);
    }
    else if(abs(x) <= 0.00003 && y < -0.004)
    {
        tan2 = -45*(180/M_PI);
    }
    else if(x > 0.00003 && abs(y) <= 0.000001)
    {
        tan2 = 0*(M_PI/180);
    }
    else if(x < -0.004 && abs(y) <= 0.000001)
    {
        tan2 = -90*(M_PI/180);
    }
    else if(x > 0.004 && y > 0.004)
    {
        tan2 = 0.5*atan((2*y)/x);
    }
    else if(x > 0.004 && y < -0.004)
    {
        tan2 = 0.5*atan((2*y)/x);
    }
    else if(x < -0.004 && y > 0.004)
    {
        tan2 = 0.5*atan((2*y)/x) + 90*(M_PI/180);
    }
    else
    {
        tan2 = 0.5*atan((2*y)/x) - 90*(M_PI/180);
    }

    kernel[3] = tan2*0.1;
    //cout<<"kernel[0]: "<<kernel[0]<<endl;
    //cout<<"kernel[1]: "<<kernel[1]<<endl;
    //cout<<"kernel[2]: "<<kernel[2]<<endl;
    //cout<<"kernel[3]: "<<kernel[3]<<endl;

}

void VisualServoCamera::publishCameraData(const float kernel[KERNEL_SIZE])
{
    std_msgs::Float64MultiArray camera_data_msg;

//    for(int i=0;i<image_mat.rows;i++)
//   {
//        for(int j=0;j<image_mat.cols;j++)
//        {
//            camera_data_msg.data.push_back(image_mat.at<double>(i,j));
//        }
//   }
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        camera_data_msg.data.push_back(kernel[i]);
    }
    camera_data_pub_.publish(camera_data_msg);
}

VisualServoCamera::~VisualServoCamera()
{

}
