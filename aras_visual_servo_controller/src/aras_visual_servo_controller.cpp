#include "aras_visual_servo_controller/aras_visual_servo_controller.h"
//#include "../aras_visual_servo_controller/include/aras_visual_servo_controller/aras_visual_servo_controller.h"
VisualServoController::VisualServoController():jacobian_inverse_mat_(KERNEL_SIZE, DOF, cv::DataType<double>::type)
{
    initializeSubscribers();
    initializePublishers();
}

bool VisualServoController::setJointPositions(float target_positions[JOINTS_NUM])
{
    for(int i=0;i<JOINTS_NUM;i++)
    {
        std_msgs::Float64 msg;
        msg.data=target_positions[i];
        joint_pub_[i].publish(msg);
    }
    return true;
}

float *VisualServoController::getJointPositions()
{
    return joints_position_;
}

void VisualServoController::setTargetPositions(float target_positions[JOINTS_NUM])
{

    hardSetJointPosition(target_positions);
    camera_call_backed_ =false;
    while(camera_call_backed_ == false)
    {
        ros::spinOnce();
    }
    ROS_INFO("camera call backed");
//    for(int i=0;i<KERNEL_SIZE;i++)
//    {
//        target_kernel_[i] = kernel_[i];
//    }
    //char *Path = new char[50];
    //sprintf(Path, "/home/parisa/catkin_ws/src/aras_visual_servo/Results/Target_Image.jpg");
    //cvSaveImage(Path ,jacobian_inverse_mat_);
    //imwrite( "/home/parisa/catkin_ws/src/aras_visual_servo/Results/Target_Image.jpg", jacobian_inverse_mat_ );
    target_kernel_[0] = kernel_[0]*z_star;
    target_kernel_[1] = kernel_[1]*z_star;
    target_kernel_[2] = z_star;
    m00_star          = kernel_[2];
    target_kernel_[3] = kernel_[3];

    for(int i=0;i<KERNEL_SIZE;i++)
    {
        ROS_INFO("kernel target %d = %lf" ,i, kernel_[i]);
    }

    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
        ROS_INFO(homedir);
    }
    else{
        ROS_INFO(homedir);
    }

    std::string path1 = (std::string(homedir) + "/catkin_ws/src/aras_visual_servo/Results/error_s.m");
    std::string path2 = (std::string(homedir) + "/catkin_ws/src/aras_visual_servo/Results/velocity.m");
    std::string path3 = (std::string(homedir) + "/catkin_ws/src/aras_visual_servo/Results/cartesian.m");
    ROS_INFO_STREAM(path1);

    fp_es = fopen(path1.c_str(), "w");
    fprintf(fp_es,"Error_s = [   %%Error_S\n");

    fp_v = fopen(path2.c_str(), "w");
    fprintf(fp_v,"Velocity = [   %%Cartesian Velocity\n");

    fp_cartesian = fopen(path3.c_str(), "w");
    fprintf(fp_cartesian,"Cartesian = [   %%Cartesian trajectory\n");

    ROS_INFO("End Task.");

}

void VisualServoController::setInitialPositions(float target_positions[])
{
    hardSetJointPosition(target_positions);
    camera_call_backed_ =false;
    while(camera_call_backed_ == false)
    {
        ros::spinOnce();
    }
    for(int i=0;i<KERNEL_SIZE;i++)
    {
        ROS_INFO("kernel initial %d = %lf" ,i, kernel_[i]);
    }
}

void VisualServoController::hardSetJointPosition(float target_positions[JOINTS_NUM])
{
    bool close_enough=true;

    //TODO set timeout

    while(true)
    {
        setJointPositions(target_positions);
        ros::spinOnce();
        for(int i=0;i<JOINTS_NUM;i++)
        {
            if(fabs((target_positions[i]-joints_position_[i]))>JOINTS_ERROR)
            {
                close_enough =false;
//                ROS_INFO("%lf %d",fabs((target_positions[i]-joints_position_[i])),i);
//                ROS_INFO("%lf , %lf",target_positions[i],joints_position_[i]);
                break;
            }
        }
        if(close_enough == true)
        {
            return;
        }
        close_enough = true;
//        ROS_INFO("not close enough");
    }

    return;
}

void VisualServoController::initializeSubscribers()
{
    joint_state_sub_ = nh_.subscribe("/aras_visual_servo/joint_states", 1, &VisualServoController::jointStateCB,this);
    camera_data_sub_ = nh_.subscribe("/aras_visual_servo/camera/data" , 1, &VisualServoController::cameraDataCB,this);

}



void VisualServoController::initializePublishers()
{
    joint_pub_[0]=nh_.advertise<std_msgs::Float64>("/aras_visual_servo/gantry_position_controller/command",10);
    //because the joint 1 is constant we don't need to set it's position.
    for(int i=1;i<JOINTS_NUM;i++)
    {
        int k=i;
        if(i>=4)
            k++;
        std::stringstream pub_topic;
        pub_topic << "/aras_visual_servo/joint" << k << "_position_controller/command";
        joint_pub_[i]=nh_.advertise<std_msgs::Float64>(pub_topic.str().c_str(),10);
    }

}

void VisualServoController::jointStateCB(const sensor_msgs::JointStatePtr& joint_states)
{
    for(int i=0;i<JOINTS_NUM-1;i++)
    {
        joints_position_[i+1]= joint_states->position[i];
    }
    joints_position_[0] = joint_states->position[JOINTS_NUM-1];
}

void VisualServoController::cameraDataCB(const std_msgs::Float64MultiArray::ConstPtr &camera_data_arr)
{
    //using namespace std;
    if(camera_data_arr->data.size() != CAMERA_DATA_SIZE)
    {
        ROS_ERROR("Camera Data Size is not %d" , CAMERA_DATA_SIZE);
        return;
    }
    camera_call_backed_ = true;
//    for(int i=0;i<IMAGE_ROW;i++)
//    {
//        for(int j=0;j<IMAGE_COLUMN;j++)
//        {
//            jacobian_inverse_mat_.at<double>(i,j) = camera_data_arr->data[(i*IMAGE_COLUMN+j)];
//            cout<<camera_data_arr->data[(i*IMAGE_COLUMN+j)]<<endl;
//            cout<<jacobian_inverse_mat_.at<double>(i,j)<<endl;
//        }
//    }
    for(int i=CAMERA_DATA_SIZE-KERNEL_SIZE;i<CAMERA_DATA_SIZE;i++)
    {
        kernel_[i-(CAMERA_DATA_SIZE-KERNEL_SIZE)] = camera_data_arr->data[i];
    }
}

void VisualServoController::executeControlAlgorithm()
{
    using namespace std;
    CvMat* Desired_joint_velocity = cvCreateMat(6 , 1, CV_32FC1 );
    CvMat* Desired_cartesian_velocity = cvCreateMat(5, 1, CV_32FC1 );
    float LEE = (float)7.2;
    while(ros::ok())
    {
//        ROS_INFO("executeControlAlgorithm");
//       float velocity_x = 0, velocity_y = 0;


        cv::Mat error(KERNEL_SIZE,1, cv::DataType<double>::type);
        cv::Mat control_signal(DOF,1, cv::DataType<double>::type);
        cv::Mat jacobian_inverse_mat(KERNEL_SIZE,DOF,cv::DataType<double>::type);
        jacobian_inverse_mat.at<double>(0,0)=-1;
        jacobian_inverse_mat.at<double>(0,1)=0;
        jacobian_inverse_mat.at<double>(0,2)=0;

        jacobian_inverse_mat.at<double>(1,0)=0;
        jacobian_inverse_mat.at<double>(1,1)=-1;
        jacobian_inverse_mat.at<double>(1,2)=0;

        jacobian_inverse_mat.at<double>(2,0)=0;
        jacobian_inverse_mat.at<double>(2,1)=0;
        jacobian_inverse_mat.at<double>(2,2)=-1;
        jacobian_inverse_mat.at<double>(2,3)=0;

        jacobian_inverse_mat.at<double>(3,0)=0;
        jacobian_inverse_mat.at<double>(3,1)=0;
        jacobian_inverse_mat.at<double>(3,2)=0;
        jacobian_inverse_mat.at<double>(3,3)=-1;
        //!!!!!!!!yekbar etefagh biofad
        cv::Mat lambda(KERNEL_SIZE,KERNEL_SIZE, cv::DataType<double>::type);
        //TODO
        //replace it with memcpy

        for(int i=0;i<KERNEL_SIZE;i++)
        {
            for(int j=0;j<KERNEL_SIZE;j++)
            {
                lambda.at<double>(i,j) = 0;
            }
        }
        lambda.at<double>(0,0) = LAMBDA_X;
        lambda.at<double>(1,1) = LAMBDA_Y;
        lambda.at<double>(2,2) = LAMBDA_Z;
        lambda.at<double>(3,3) = LAMBDA_THETA;

        cout<<"lambda_y"<<lambda.at<double>(1,1)<<endl;


//        for(int i=0;i<KERNEL_SIZE;i++)
//        {
//            error.at<double>(i,0) = kernel_[i] - target_kernel_[i] ;
//        }
        //cout<<"kernel[0]: "<<kernel_[0]*z_star*sqrt(m00_star/kernel_[2])<<endl;
        //cout<<"kernel[1]: "<<kernel_[1]*z_star*sqrt(m00_star/kernel_[2])<<endl;
        //cout<<"kernel[2]: "<<z_star*sqrt(m00_star/kernel_[2])<<endl;
        //cout<<"kernel[3]: "<<kernel_[3]<<endl;

        error.at<double>(0,0) = kernel_[0]*z_star*sqrt(m00_star/kernel_[2]) - target_kernel_[0] ;
        error.at<double>(1,0) = kernel_[1]*z_star*sqrt(m00_star/kernel_[2]) - target_kernel_[1] ;
        error.at<double>(2,0) = z_star*sqrt(m00_star/kernel_[2]) - target_kernel_[2] ;
        error.at<double>(3,0) = kernel_[3] - target_kernel_[3];

        fprintf(fp_es,"%f %f %f %f\n",error.at<double>(0,0), error.at<double>(1,0), error.at<double>(2,0),
                error.at<double>(3,0));


        cout<<"error_x"<<error.at<double>(0,0)<<endl;
        cout<<"error_y"<<error.at<double>(1,0)<<endl;
        cout<<"error_z"<<error.at<double>(2,0)<<endl;
        cout<<"error_theta"<<error.at<double>(3,0)<<endl;


        jacobian_inverse_mat.at<double>(0,3)= -kernel_[1]*z_star*sqrt(m00_star/kernel_[2]);
        jacobian_inverse_mat.at<double>(1,3)=  kernel_[0]*z_star*sqrt(m00_star/kernel_[2]);



        control_signal = -1 * lambda * jacobian_inverse_mat * error;
        control_signal.at<double>(0,0) *= -1 ;
        cout<<"desired_control_sig"<<control_signal.at<double>(1,0)<<endl;
        fprintf(fp_v,"%f %f %f %f\n",control_signal.at<double>(0,0), control_signal.at<double>(1,0),
                control_signal.at<double>(2,0), control_signal.at<double>(3,0));



        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 0, 0) = control_signal.at<double>(0,0);
        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 1, 0) = control_signal.at<double>(1,0);
        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 2, 0) = control_signal.at<double>(2,0);
        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 3, 0) = control_signal.at<double>(3,0);
        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 4, 0) = 0;

        cout<<"desired_cartesian_velocity"<< CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 1, 0) <<endl;




        SingularityAvoidance(Desired_cartesian_velocity, -(90 -joints_position_[0]*100),
                                            joints_position_[1], joints_position_[2], joints_position_[3],
                                            joints_position_[4], joints_position_[5], 0.12, 0.5, LEE, Desired_joint_velocity);
cout<<"desired_joint_velocity: "<<CV_MAT_ELEM(*(Desired_joint_velocity), float, 0, 0)<<endl;
//        velocity_x =  control_signal.at<double>(0,0);
//        velocity_y =  control_signal.at<double>(1,0);
//        control_signal.at<double>(0,0) = velocity_x*cos(((double)joints_position_[1] + (double)joints_position_[5]))
//                - velocity_y*sin(((double)joints_position_[1] + (double)joints_position_[5]));
//        control_signal.at<double>(1,0) = velocity_x*sin(((double)joints_position_[1] + (double)joints_position_[5]))
//                + velocity_y*cos(((double)joints_position_[1] + (double)joints_position_[5]));
        float current_x,current_y,current_z,current_yaw;



        //float desired_x,desired_y,desired_z,desired_yaw;

        //Calcute current position
        forwardKinematic(joints_position_, current_x, current_y, current_z, current_yaw);
        fprintf(fp_cartesian,"%f %f %f %f %f %f %f %f\n",current_x, current_y, current_z, current_yaw, Target_x,
                Target_y, Target_z, Target_yaw);

        cout<<"current_y: "<<current_y<<endl;
//        for(int i=0;i<DOF;i++)
//        {
//            ROS_INFO("error :%lf",error.at<double>(i,0));
//        }

        if(abs(error.at<double>(0,0))<0.001  && abs(error.at<double>(1,0))<0.003  && abs(error.at<double>(2,0))<0.005 && abs(error.at<double>(3,0))<0.001)
        {
            fprintf(fp_es,"];\n");
            fprintf(fp_v,"];\n");
            fprintf(fp_cartesian,"];\n");
            fclose (fp_es);
            fclose (fp_v);
            fclose (fp_cartesian);
            while(1);
        }

        //Integral velocity to convert to Position;
        //desired_x = current_x + control_signal.at<double>(0,0)*SAMPLE_TIME;
        //desired_y = current_y + control_signal.at<double>(1,0)*SAMPLE_TIME;
        //desired_z = current_z + control_signal.at<double>(2,0)*SAMPLE_TIME;
        //desired_yaw = current_yaw + control_signal.at<double>(3,0);

        float target_position[JOINTS_NUM];
        // Compute Target joints value(Inverse Kinematic)

        //inverseKinematic(target_position, desired_x, desired_y, desired_z, desired_yaw);


        target_position[0]= (joints_position_[0]*100 +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 0, 0)*SAMPLE_TIME)/100;
        target_position[1]= joints_position_[1] +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 1, 0)*SAMPLE_TIME;
        target_position[2]= joints_position_[2] +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 2, 0)*SAMPLE_TIME;
        target_position[3]= joints_position_[3] +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 3, 0)*SAMPLE_TIME;
        target_position[4]= joints_position_[4] +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 4, 0)*SAMPLE_TIME;//-(target_position[2] + target_position[3]);//joints_position_[4] +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 4, 0)*SAMPLE_TIME;
        target_position[5]= joints_position_[5] +  CV_MAT_ELEM(*(Desired_joint_velocity), float, 5, 0)*SAMPLE_TIME;

        cout<<"target_position[0]"<<target_position[0]<<endl;
        cout<<"target_position[1]"<<target_position[1]<<endl;
        cout<<"target_position[2]"<<target_position[2]<<endl;
        cout<<"target_position[3]"<<target_position[3]<<endl;
        cout<<"target_position[4]"<<target_position[4]<<endl;
        cout<<"target_position[5]"<<target_position[5]<<endl;


        hardSetJointPosition(target_position);
        camera_call_backed_ =false;
        while(camera_call_backed_ == false)
        {
            ros::spinOnce();
        }
    }
}

void VisualServoController::inverseKinematic(float target_joints[JOINTS_NUM], float x, float y, float z, float a)
{
    using namespace std;
    float delta, c_theta3, s_theta3, c_theta2, s_theta2;
    delta = z - L5 - L1;
    c_theta3 = (x*x + delta*delta - L2*L2 - L3*L3)/(2*L2*L3);
    cout<<"C_theta3= "<<c_theta3<<endl;
    s_theta3 = - sqrt(1 - c_theta3*c_theta3);
    c_theta2 = (L3*s_theta3*x + (L2 + L3*c_theta3)*delta)/(L2*L2 + L3*L3 + 2*L2*L3*c_theta3);
    s_theta2 = ((L2 + L3*c_theta3)*x - L3*s_theta3*delta)/(L2*L2 + L3*L3 + 2*L2*L3*c_theta3);
    target_joints[0] = y;//gantry
    target_joints[1] = 0; //theta1 is zero
    target_joints[2] = atan2(s_theta2, c_theta2); //theta2, Degree
    cout<<"target_joints[2]"<<target_joints[2]<<endl;
    target_joints[3] = atan2(s_theta3, c_theta3); //theta3, Degree
    cout<<"target_joints[3]"<<target_joints[3]<<endl;
    target_joints[4] =  -(target_joints[2] + target_joints[3]); //theta5 = -(theta2 + theta3), Degree
    cout<<"target_joints[4]"<<target_joints[4]<<endl;
    target_joints[5] = a; //theta 1 is zero, so A change only with theta6
}
void VisualServoController::forwardKinematic(float target_joints[JOINTS_NUM], float &x, float &y, float &z, float &a)
{
    float theta1 = target_joints[1];
    float theta2 = target_joints[2];
    float theta3 = target_joints[3];
    float theta5 = target_joints[4];
    float theta6 = target_joints[5];
    x = cos(theta1)*(L2*sin(theta2) + L3*sin(theta2 + theta3) + L5*sin(theta2 + theta3 + theta5));//m
    y = sin(theta1)*(L2*sin(theta2) + L3*sin(theta2 + theta3) + L5*sin(theta2 + theta3 + theta5)) + (target_joints[0]);//m
    z = L1 + L2*cos(theta2) + L3*cos(theta2 + theta3) + L5*cos(theta2 + theta3 + theta5);//m
    a = (theta1 + theta6);//radian
}
void VisualServoController::SingularityAvoidance(CvMat* xdesired_dot,float lgantry,float teta1,
                                                 float teta2,float teta3,float teta5,float teta6,
                                                 float gain,float k_bound,float lee,CvMat* qdesired_dot)
                       {
                           //LEE IS NOT DEFINED!
                           int j;
                           CvMat* Jac = cvCreateMat(5 , 6, CV_32FC1 );  // Jac=zeros(5,6);
                           CvMat* Jac_trans = cvCreateMat(6 , 5, CV_32FC1 );
                           CvMat* costt = cvCreateMat(5 , 5, CV_32FC1 );
                           CvMat*  pinvcostt = cvCreateMat(5 , 5, CV_32FC1 );

                           *( (float*)CV_MAT_ELEM_PTR( *Jac,0,1) )= (float)(-25.00000*sin(teta2)*sin(teta1)-16.00000*sin(teta2)*sin(teta1)*cos(teta3)-16.00000*sin(teta1)*cos(teta2)*sin(teta3)-((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee) ;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,0,0) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,0,2) )= (float)(cos(teta1)*(25.00000*cos(teta2)+16.00000*cos(teta2)*cos(teta3)-16.00000*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee)) ;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,0,3) )= (float)(cos(teta1)*(16.00000*cos(teta2)*cos(teta3)-16.00000*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee));
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,0,4) )= (float)(cos(teta1)*((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee) ;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,0,5) )= 0 ;
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,1,0) )= 1.00000 ;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,1,1) )= (float)(25.00000*sin(teta2)*cos(teta1)+16.00000*sin(teta2)*cos(teta1)*cos(teta3)+16.00000*cos(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee);
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,1,2) )= (float)(sin(teta1)*(25.00000*cos(teta2)+16.00000*cos(teta2)*cos(teta3)-16.00000*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee));
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,1,3) )= (float)(sin(teta1)*(16.00000*cos(teta2)*cos(teta3)-16.00000*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee));
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,1,4) )= (float)(sin(teta1)*((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee);
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,1,5) )= 0;
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,2,0) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,2,1) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,2,2) )= (float)(-sin(teta1)*(25.00000*sin(teta2)*sin(teta1)+16.00000*sin(teta2)*sin(teta1)*cos(teta3)+16.00000*sin(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee)-cos(teta1)*(25.00000*sin(teta2)*cos(teta1)+16.00000*sin(teta2)*cos(teta1)*cos(teta3)+16.00000*cos(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee));
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,2,3) )= (float)(-sin(teta1)*(16.00000*sin(teta2)*sin(teta1)*cos(teta3)+16.00000*sin(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee)-cos(teta1)*(16.00000*sin(teta2)*cos(teta1)*cos(teta3)+16.00000*cos(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee));
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,2,4) )= (float)(-sin(teta1)*((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee-cos(teta1)*((sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee);
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,2,5) )= 0;
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,3,0) )=0  ;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,3,1) )= (float)(((-sin(teta2)*cos(teta3)-cos(teta2)*sin(teta3))*sin(teta5)+(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)));
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,3,2) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,3,3) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,3,4) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,3,5) )= 1.00;
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,4,0) )=  0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,4,1) )= 0;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,4,2) )= 1.00;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,4,3) )= 1.00;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,4,4) )= 1.00;
                           *( (float*)CV_MAT_ELEM_PTR( *Jac,4,5) )= 0;
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           float dif_lgantry[5][6]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                           //diff_lgantry=zeros(5,6);
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           float dif_teta1[5][6]={
                               0
                               ,-25*sin(teta2)*cos(teta1)-16*sin(teta2)*cos(teta1)*cos(teta3)-16*cos(teta1)*cos(teta2)*sin(teta3)-((sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,-sin(teta1)*(25*cos(teta2)+16*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,-sin(teta1)*(16*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,-sin(teta1)*((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,-25*sin(teta2)*sin(teta1)-16*sin(teta2)*sin(teta1)*cos(teta3)-16*sin(teta1)*cos(teta2)*sin(teta3)+((-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(sin(teta2)*sin(teta1)*sin(teta3)-sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,cos(teta1)*(25*cos(teta2)+16*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,cos(teta1)*(16*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta3)+((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,cos(teta1)*((cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*cos(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,0
                               ,-cos(teta1)*(25*sin(teta2)*sin(teta1)+16*sin(teta2)*sin(teta1)*cos(teta3)+16*sin(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee)-cos(teta1)*(-25*sin(teta2)*sin(teta1)-16*sin(teta2)*sin(teta1)*cos(teta3)-16*sin(teta1)*cos(teta2)*sin(teta3)+((-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(sin(teta2)*sin(teta1)*sin(teta3)-sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,-cos(teta1)*(16*sin(teta2)*sin(teta1)*cos(teta3)+16*sin(teta1)*cos(teta2)*sin(teta3)+((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee)-cos(teta1)*(-16*sin(teta2)*sin(teta1)*cos(teta3)-16*sin(teta1)*cos(teta2)*sin(teta3)+((-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(sin(teta2)*sin(teta1)*sin(teta3)-sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,-cos(teta1)*((sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee-cos(teta1)*((-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*cos(teta5)+(sin(teta2)*sin(teta1)*sin(teta3)-sin(teta1)*cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0};
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           float dif_teta2[5][6]={
                               0
                               ,-25*sin(teta1)*cos(teta2)-16*sin(teta1)*cos(teta2)*cos(teta3)+16*sin(teta2)*sin(teta1)*sin(teta3)-((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee
                               ,cos(teta1)*(-25*sin(teta2)-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,cos(teta1)*(-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,cos(teta1)*((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,25*cos(teta1)*cos(teta2)+16*cos(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*cos(teta1)*sin(teta3)+((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee
                               ,sin(teta1)*(-25*sin(teta2)-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,sin(teta1)*(-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,sin(teta1)*((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,0
                               ,-sin(teta1)*(25*sin(teta1)*cos(teta2)+16*sin(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta1)*sin(teta3)+((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee)-cos(teta1)*(25*cos(teta1)*cos(teta2)+16*cos(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*cos(teta1)*sin(teta3)+((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee)
                               ,-sin(teta1)*(16*sin(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta1)*sin(teta3)+((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee)-cos(teta1)*(16*cos(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*cos(teta1)*sin(teta3)+((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee)
                               ,-sin(teta1)*((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee-cos(teta1)*((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5)
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0};
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           float dif_teta3[5][6]={
                               0
                               ,16*sin(teta2)*sin(teta1)*sin(teta3)-16*sin(teta1)*cos(teta2)*cos(teta3)-((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee
                               ,cos(teta1)*(-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,cos(teta1)*(-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,cos(teta1)*((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,16*cos(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*cos(teta1)*sin(teta3)+((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee
                               ,sin(teta1)*(-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,sin(teta1)*(-16*sin(teta2)*cos(teta3)-16*cos(teta2)*sin(teta3)+((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee)
                               ,sin(teta1)*((-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,0
                               ,-sin(teta1)*(16*sin(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta1)*sin(teta3)+((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee)-cos(teta1)*(16*cos(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*cos(teta1)*sin(teta3)+((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee)
                               ,-sin(teta1)*(16*sin(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*sin(teta1)*sin(teta3)+((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee)-cos(teta1)*(16*cos(teta1)*cos(teta2)*cos(teta3)-16*sin(teta2)*cos(teta1)*sin(teta3)+((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee)
                               ,-sin(teta1)*((-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-sin(teta2)*sin(teta1)*cos(teta3)-sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5))*lee-cos(teta1)*((-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5)+(-cos(teta1)*cos(teta2)*sin(teta3)-sin(teta2)*cos(teta1)*cos(teta3))*sin(teta5))*lee
                               ,0
                               ,0
                               ,(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)+(sin(teta2)*sin(teta3)-cos(teta2)*cos(teta3))*sin(teta5)
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0};
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           float dif_teta5[5][6]={
                               0
                               ,-(-(sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee
                               ,cos(teta1)*(-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5))*lee
                               ,cos(teta1)*(-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5))*lee
                               ,cos(teta1)*(-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5))*lee
                               ,0
                               ,0
                               ,(-(sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee
                               ,sin(teta1)*(-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5))*lee
                               ,sin(teta1)*(-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5))*lee
                               ,sin(teta1)*(-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5))*lee
                               ,0
                               ,0
                               ,0
                               ,-sin(teta1)*(-(sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee-cos(teta1)*(-(sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee
                               ,-sin(teta1)*(-(sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee-cos(teta1)*(-(sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee
                               ,-sin(teta1)*(-(sin(teta2)*sin(teta1)*cos(teta3)+sin(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*sin(teta1)*sin(teta3)+sin(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee-cos(teta1)*(-(sin(teta2)*cos(teta1)*cos(teta3)+cos(teta1)*cos(teta2)*sin(teta3))*sin(teta5)+(-sin(teta2)*cos(teta1)*sin(teta3)+cos(teta1)*cos(teta2)*cos(teta3))*cos(teta5))*lee
                               ,0
                               ,0
                               ,-(cos(teta2)*cos(teta3)-sin(teta2)*sin(teta3))*sin(teta5)+(-cos(teta2)*sin(teta3)-sin(teta2)*cos(teta3))*cos(teta5)
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0
                               ,0};
                           //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                           float dif_teta6[5][6]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
                           //*******************************************************************
                           CvMat* diff_lgantry = cvCreateMat(5 , 6, CV_32FC1 );
                           CvMat* diff_teta1 = cvCreateMat(5 , 6, CV_32FC1 );
                           CvMat* diff_teta2 = cvCreateMat(5 , 6, CV_32FC1 );
                           CvMat* diff_teta3 = cvCreateMat(5 , 6, CV_32FC1 );
                           CvMat* diff_teta5 = cvCreateMat(5 , 6, CV_32FC1 );
                           CvMat* diff_teta6 = cvCreateMat(5 , 6, CV_32FC1 );
                           //**********************rikhtan maghadir moshtagh dar matrixha*****************************************
                           memcpy(diff_lgantry->data.fl, dif_lgantry, sizeof(dif_lgantry));
                           memcpy(diff_teta1->data.fl, dif_teta1, sizeof(dif_teta1));
                           memcpy(diff_teta2->data.fl, dif_teta2, sizeof(dif_teta2));
                           memcpy(diff_teta3->data.fl, dif_teta3, sizeof(dif_teta3));
                           memcpy(diff_teta5->data.fl, dif_teta5, sizeof(dif_teta5));
                           memcpy(diff_teta6->data.fl, dif_teta6, sizeof(dif_teta6));

                           //%%%%%%%%%%%%%%%%%%%%%
                           //costt=Jac*Jac';
                           cvTranspose (Jac,Jac_trans);
                           //cvMul (Jac ,Jac_trans, costt);
                           cvMatMulAdd (Jac,Jac_trans, 0,costt);
                           float detcost=(float)sqrt(fabs(cvDet(costt)));
                           float eye1[5][5]={
                                           (float)0.01,0,0,0,0,
                                           0,(float)0.01,0,0,0,
                                           0,0,(float)0.01,0,0,
                                           0,0,0,(float)0.01,0,
                                           0,0,0,0,(float)0.01
                           };
                           float eye0[5][5]={
                                            0,0,0,0,0,
                                            0,0,0,0,0,
                                            0,0,0,0,0,
                                            0,0,0,0,0,
                                            0,0,0,0,0
                                           };
                           CvMat* beta = cvCreateMat(5 , 5, CV_32FC1 );

                           if (fabs(detcost)<=0.01) {
                               memcpy(beta->data.fl, eye1, sizeof(eye1));
                               cvMatMulAdd (Jac,Jac_trans, beta,costt);
                           }
                           //beta=0.01*eye(5,5);
                           else
                               memcpy(beta->data.fl, eye0, sizeof(eye0));
                           //pinvcostt= pinv(costt);
                           cvInvert (costt,pinvcostt);
                           //siigma=zeros(6,1);
                           CvMat* siigma = cvCreateMat(6 , 1, CV_32FC1 );
                           //printf("\n%f",sqrt(cvDet(costt)));

                           float alfa,sum;
                           int ii,jj,ll;
                           for( ll=1;ll<7;ll++) {
                               switch(ll) {
                               case 1:
                                   alfa=0;
                                   for (ii=0;ii<5;ii++) {
                                       for (jj=0;jj<5;jj++) {
                                           sum=0;
                                           for(j=0;j<6;j++)
                                               sum=sum+CV_MAT_ELEM(*(diff_lgantry),float,ii,j)*CV_MAT_ELEM(*(Jac),float,jj,j)+ CV_MAT_ELEM(*(diff_lgantry),float,jj,j)*CV_MAT_ELEM(*(Jac),float,ii,j);
                                       alfa=alfa+ CV_MAT_ELEM(*(pinvcostt),float,ii,jj)*sum;
                                       }
                                   }
                                   CV_MAT_ELEM(*(siigma),float,ll-1,0)=(float)0.5*alfa*detcost;
                               break;
                               case 2:
                                   alfa=0;
                                   for (ii=0;ii<5;ii++) {
                                       for (jj=0;jj<5;jj++) {
                                           sum=0;
                                           for(j=0;j<6;j++)
                                               sum=sum+CV_MAT_ELEM(*(diff_teta1),float,ii,j)*CV_MAT_ELEM(*(Jac),float,jj,j)+ CV_MAT_ELEM(*(diff_teta1),float,jj,j)*CV_MAT_ELEM(*(Jac),float,ii,j);
                                           alfa=alfa+ CV_MAT_ELEM(*(pinvcostt),float,ii,jj)*sum;
                                       }
                                   }
                                   CV_MAT_ELEM(*(siigma),float,ll-1,0)=(float)0.5*alfa*detcost;
                               break;
                               case 3:
                                   alfa=0;
                                   for (ii=0;ii<5;ii++) {
                                       for (jj=0;jj<5;jj++) {
                                           sum=0;
                                           for(j=0;j<6;j++)
                                               sum=sum+CV_MAT_ELEM(*(diff_teta2),float,ii,j)*CV_MAT_ELEM(*(Jac),float,jj,j)+ CV_MAT_ELEM(*(diff_teta2),float,jj,j)*CV_MAT_ELEM(*(Jac),float,ii,j);
                                       alfa=alfa+ CV_MAT_ELEM(*(pinvcostt),float,ii,jj)*sum;
                                       //printf("\nalfa(%d,%d)=%f",ll,jj,alfa);
                                       }
                                   }
                                   CV_MAT_ELEM(*(siigma),float,ll-1,0)=(float)0.5*alfa*detcost;
                               break;
                               case 4:
                                   alfa=0;
                                   for (ii=0;ii<5;ii++) {
                                       for (jj=0;jj<5;jj++) {
                                           sum=0;
                                           for(j=0;j<6;j++)
                                               sum=sum+CV_MAT_ELEM(*(diff_teta3),float,ii,j)*CV_MAT_ELEM(*(Jac),float,jj,j)+ CV_MAT_ELEM(*(diff_teta3),float,jj,j)*CV_MAT_ELEM(*(Jac),float,ii,j);
                                           alfa=alfa+ CV_MAT_ELEM(*(pinvcostt),float,ii,jj)*sum;
                                       }
                                   }
                                   CV_MAT_ELEM(*(siigma),float,ll-1,0)=(float)0.5*alfa*detcost;
                               break;
                               case 5:
                                   alfa=0;
                                   for (ii=0;ii<5;ii++) {
                                       for (jj=0;jj<5;jj++) {
                                           sum=0;
                                           for(j=0;j<6;j++)
                                               sum=sum+CV_MAT_ELEM(*(diff_teta5),float,ii,j)*CV_MAT_ELEM(*(Jac),float,jj,j)+ CV_MAT_ELEM(*(diff_teta5),float,jj,j)*CV_MAT_ELEM(*(Jac),float,ii,j);
                                       alfa=alfa+ CV_MAT_ELEM(*(pinvcostt),float,ii,jj)*sum;
                                       }
                                   }
                                   CV_MAT_ELEM(*(siigma),float,ll-1,0)=(float)0.5*alfa*detcost;
                               break;
                               case 6:
                                   alfa=0;
                                   for (ii=0;ii<5;ii++) {
                                       for (jj=0;jj<5;jj++) {
                                           sum=0;
                                           for(j=0;j<6;j++)
                                               sum=sum+CV_MAT_ELEM(*(diff_teta6),float,ii,j)*CV_MAT_ELEM(*(Jac),float,jj,j)+ CV_MAT_ELEM(*(diff_teta6),float,jj,j)*CV_MAT_ELEM(*(Jac),float,ii,j);
                                           alfa=alfa+ CV_MAT_ELEM(*(pinvcostt),float,ii,jj)*sum;
                                       }
                                   }
                                   CV_MAT_ELEM(*(siigma),float,ll-1,0)=(float)0.5*alfa*detcost;
                               break;
                               }
                           }

                           if (fabs(detcost)<=0.01)
                               memcpy(beta->data.fl, eye1, sizeof(eye1));
                               //beta=0.01*eye(5,5);
                           else
                               memcpy(beta->data.fl, eye0, sizeof(eye0));
                               //beta=0*eye(5,5);
                           CvMat*  JJT = cvCreateMat(5 , 5, CV_32FC1 );
                           CvMat*  inv_JJT = cvCreateMat(5 , 5, CV_32FC1 );
                           CvMat*  Jacplus= cvCreateMat(6 , 5, CV_32FC1 );
                           cvMatMulAdd (Jac,Jac_trans, beta,JJT);
                           cvInvert (JJT,inv_JJT);

                           cvMatMulAdd (Jac_trans ,inv_JJT,0, Jacplus);
                           //%%%%%%%%%%%%%%%%%%%%
                           //%%Bound On Joints Value regarding Robot Constraints
                           //%Note: Bounds are set to 10 degree. It means Joint limits
                           //%will need a big avoidance if joint angles have a distance less than 10 degree to the bounds!!!
                           //%%%%%%%%%%%%%%%%%%%
                           float der_cost_lgantry=0;
                           float der_cost_teta1=0;
                           float der_cost_teta2=0;
                           float der_cost_teta3=0;
                           float der_cost_teta5=0;
                           float der_cost_teta6=0;
                           /*%%%%%%%%%%%%%%%
                           lgantry
                           %%%%%%%%%%%%%%%*/
                           float B1_lgantry=180;
                           float B2_lgantry=10;
                           float alfa_lgantry=2*10;

                           if(lgantry>=-B1_lgantry && lgantry<=-B2_lgantry)
                               der_cost_lgantry=0;
                           else if (lgantry>-B1_lgantry)
                               der_cost_lgantry=-(2/alfa_lgantry)*(lgantry+B1_lgantry);
                           else if (lgantry<-B2_lgantry)
                               der_cost_lgantry=-(2/alfa_lgantry)*(lgantry+B2_lgantry);
                           /*%%%%%%%%%%%%%%%
                           teta1
                           %%%%%%%%%%%%%%%*/
                           float B_teta1=(float)(2.53-.09);//135 deg;
                           float alfa_teta1=(float)(2*0.1745);//normalization factor

                           if(teta1>=-B_teta1 && teta1<=B_teta1)
                               der_cost_teta1=0;
                           else if (teta1>B_teta1)
                               der_cost_teta1=-(2/alfa_teta1)*(teta1-B_teta1);
                           else if (teta1<-B_teta1)
                               der_cost_teta1=-(2/alfa_teta1)*(teta1+B_teta1);
                           /*%%%%%%%%%%%%%%%
                           teta2
                           %%%%%%%%%%%%%%%*/
                           float B1_teta2=(float)(0.96-.09);
                           float B2_teta2=(float)(2-.09);
                           float alfa_teta2=2*0.1745;
                           if(teta2>=-B1_teta2 && teta2<=B2_teta2)
                               der_cost_teta2=0;
                           else if (teta2>B2_teta2)
                               der_cost_teta2=-(2/alfa_teta2)*(teta2-B2_teta2);
                           else if (teta2<-B1_teta2)
                               der_cost_teta2=-(2/alfa_teta2)*(teta2+B1_teta2);
                           /*%%%%%%%%%%%%%%%
                           teta3
                           %%%%%%%%%%%%%%%*/
                           float B1_teta3=(float)(1.83-0.09);
                           float B2_teta3=(float)(2-0.09);
                           float alfa_teta3=(float)(2*0.1745);
                           if(teta3>=-B1_teta3 && teta3<=B2_teta3)
                               der_cost_teta3=0;
                           else if(teta3>B2_teta3)
                               der_cost_teta3=-(2/alfa_teta3)*(teta3-B2_teta3);
                           else if (teta3<-B1_teta3)
                               der_cost_teta3=-(2/alfa_teta3)*(teta3+B1_teta3);
                           /*%%%%%%%%%%%%%%%
                           teta5
                           %%%%%%%%%%%%%%%*/
                           float B_teta5=(float)(1.483-0.09);
                           float alfa_teta5=(float)(2*0.1745);

                           if (teta5>=-B_teta5 && teta5<=B_teta5)
                               der_cost_teta5=0;
                           else if(teta5>B_teta5)
                               der_cost_teta5=-(2/alfa_teta5)*(teta5-B_teta5);
                           else if(teta5<-B_teta5)
                               der_cost_teta5=-(2/alfa_teta5)*(teta5+B_teta5);
                           /*%%%%%%%%%%%%%%%
                           teta6
                           %%%%%%%%%%%%%%%*/
                           float B_teta6=(float)(3.45-0.09);
                           float alfa_teta6=(float)(2*0.1745);

                           if (teta6>=-B_teta6 && teta6<=B_teta6)
                               der_cost_teta6=0;
                           else if (teta6>B_teta6)
                               der_cost_teta6=-(2/alfa_teta6)*(teta6-B_teta6);
                           else if (teta6<-B_teta6)
                               der_cost_teta6=-(2/alfa_teta6)*(teta6+B_teta6);
                           //%%%%%%%%%%%%%%%%%%%
                           float der_cost_bound1[6][1]={der_cost_lgantry,der_cost_teta1,der_cost_teta2,der_cost_teta3,der_cost_teta5,der_cost_teta6};
                           CvMat* der_cost_bound = cvCreateMat(6 , 1, CV_32FC1 );
                           memcpy(der_cost_bound->data.fl, der_cost_bound1, sizeof(der_cost_bound1));
                           //%%%%%%%%%%%%%%%%%%%%
                           CvMat* KD = cvCreateMat(6 , 1, CV_32FC1 );
                           CvMat* SGKD = cvCreateMat(6 , 1, CV_32FC1 );
                           float eye2[6][6]={
                                            1,0,0,0,0,0,
                                            0,1,0,0,0,0,
                                            0,0,1,0,0,0,
                                            0,0,0,1,0,0,
                                            0,0,0,0,1,0,
                                            0,0,0,0,0,1
                                           };
                           CvMat* eye = cvCreateMat(6 , 6, CV_32FC1 );
                           CvMat* ebterak = cvCreateMat(6 , 1, CV_32FC1 );
                           CvMat* eb = cvCreateMat(6 , 1, CV_32FC1 );
                           CvMat* JJ = cvCreateMat(6 , 6, CV_32FC1 );
                           CvMat* eJJ = cvCreateMat(6 , 6, CV_32FC1 );
                           memcpy(eye->data.fl,eye2, sizeof(eye2));
                           CvMat* k_bound1= cvCreateMat(1 , 1, CV_32FC1 );
                           *( (float*)CV_MAT_ELEM_PTR( *k_bound1,0,0) )=k_bound;
                           cvMatMulAdd (der_cost_bound,k_bound1 ,0 ,KD);
                           //cvMul (k_bound1 ,der_cost_bound, KD);
                           CvMat* gain1= cvCreateMat(1 , 1, CV_32FC1 );
                           *( (float*)CV_MAT_ELEM_PTR( *gain1,0,0) )=gain;
                           cvMatMulAdd (siigma,gain1, KD,SGKD);
                           cvMatMulAdd (Jacplus ,Jac, 0,JJ);
                           //cvMul (Jacplus ,Jac, JJ);
                           cvSubb_soli(eye,JJ,eJJ);

                           cvMatMulAdd (eJJ ,SGKD,0,ebterak);
                           /*	printf("\n\n");
                           for(i = 0; i<6; i++) {
                               for(j=0;j<5;j++)
                                   printf( "*%.4f ", CV_MAT_ELEM(*(Jacplus),float,i,j) );
                               printf("\n");
                           }
                           printf("\n\n");
                           for(i = 0; i<6; i++) {
                               //for(j=0;j<6;j++)
                                   printf( "SGKD=*%f ", CV_MAT_ELEM(*(siigma),float,i,0) );
                               printf("\n");
                           }
                           printf("\n\n");

                           for(i = 0; i<6; i++) {
                               //for(j=0;j<6;j++)
                                   printf( "ebt=*%f ", CV_MAT_ELEM(*(ebterak),float,i,0) );
                               printf("\n");
                           }*/
                           //cvMul (eJJ ,SGKD,ebterak);
                           cvMatMulAdd (Jacplus,xdesired_dot, 0,eb);
                           cvMatMulAdd (Jacplus,xdesired_dot, ebterak,qdesired_dot);
                           /*printf("\n");
                           for(i = 0; i<6; i++) {
                               //for(j=0;j<5;j++)
                                   printf( "qd=%f  ", CV_MAT_ELEM(*(qdesired_dot),float,i,0) );
                                   printf("\n");
                               }*/
                           /*
                           cvMatMulAdd (eJJ ,SGKD,eb,qdesired_dot);
                           printf("\n\n");
                           for(i = 0; i<6; i++) {
                               //for(j=0;j<5;j++)
                                   printf( "dovomi%f  ", CV_MAT_ELEM(*(qdesired_dot),float,i,0) );
                                   printf("\n");
                           }*/
                           //qdesired_dot=Jacplus*xdesired_dot+(eye(6,6)-Jacplus*Jac)*(siigma*gain+k_bound*der_cost_bound);
                           cvReleaseMat(&Jac);
                           cvReleaseMat(&Jac_trans);
                           cvReleaseMat(&costt);
                           cvReleaseMat(&pinvcostt);
                           cvReleaseMat(&diff_lgantry);
                           cvReleaseMat(&diff_teta1);
                           cvReleaseMat(&diff_teta2);
                           cvReleaseMat(&diff_teta3);
                           cvReleaseMat(&diff_teta5);
                           cvReleaseMat(&diff_teta6);
                           cvReleaseMat(&beta);
                           cvReleaseMat(&siigma);
                           cvReleaseMat(&JJT);
                           cvReleaseMat(&inv_JJT);
                           cvReleaseMat(&Jacplus);
                           cvReleaseMat(&der_cost_bound);
                           cvReleaseMat(&KD);
                           cvReleaseMat(&SGKD);
                           cvReleaseMat(&eye);
                           cvReleaseMat(&ebterak);
                           cvReleaseMat(&eb);
                           cvReleaseMat(&JJ);
                           cvReleaseMat(&eJJ);
                           cvReleaseMat(&k_bound1);
                           cvReleaseMat(&gain1);
                       }
void VisualServoController::cvSubb_soli(CvMat* measurement,CvMat* nonlinear_measurement,CvMat* Z_G)
{
    int i, j;
    float a, b, c;
    for(i = 0; i < measurement->rows; i++)
        for(j = 0; j < measurement->cols; j++)
        {
            a = CV_MAT_ELEM(*(measurement), float, i, j);
            b = CV_MAT_ELEM(*(nonlinear_measurement), float, i, j);
            c = a - b;
            CV_MAT_ELEM(*(Z_G), float, i, j) = c;
        }
}
VisualServoController::~VisualServoController()
{

}
