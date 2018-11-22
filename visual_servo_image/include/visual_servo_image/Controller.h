#pragma once

#include "boost/thread.hpp"
#include "boost/thread/mutex.hpp"
//#include <Windows.h>
#ifdef _CH_
#pragma package <opencv>
#endif
//#include "stdafx.h"
#ifndef _EiC
#include <cv.h>
#include <highgui.h>
#include "cv.h"
#include "highgui.h"
#include <math.h>
#include <stdio.h>
#include <cxcore.h>
//#include <cxtypes.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cxcore.hpp>
using namespace std;
#endif

#define pi 3.14159
class ControllerManager
{
	public:
		
		ControllerManager();
		
		~ControllerManager();

		void ControllerInitialize();
		
		// Kernel_XY get image and compute kesi_XY, Ger and Jacobian_XY in x and y direction
		void Kernel_XY(CvMat* image, CvMat* kesi_XY , CvMat* Ger, CvMat* Jacobian_XY);

		//Kernel_Z get Fimage and compute kesi_z and Gerz in x and y direction
		void Kernel_Z(CvMat* Fimage, float &kesiz , float &Gerz);

		//Kernel_Theta get Fimage and compute kesi_Theta and Gerr in x and y direction
		void Kernel_Theta(CvMat* Fimage, float &kesi_Theta , float &Gerr);
		
		void cvSubb_soli(CvMat* measurement,CvMat* nonlinear_measurement,CvMat* Z_G);
		
		// this function get xdesired_dot, current theta,lgantry and compute qdesired_dot
		// so that we can reach xdesired_dot without singularity.
		void SingularityAvoidance(CvMat* xdesired_dot,float lgantry,float teta1,float teta2,
									float teta3,float teta5,float teta6,float gain,float k_bound,
									float lee,CvMat* qdesired_dot);

		// this function generate signal control(xdesired_dot) based on KVBS method
		void ControlSignal_KBVS(CvMat* vel_des, CvMat* e_kesi_XY, float e_kesi_Z, float e_kesi_Theta,
								CvMat* Jacobian_XY, float Jacobian_Z, float Jacobian_Theta, int temp);

		// 
		void SlidingSurface(CvMat* S, CvMat* Lambda, CvMat* e_kesi_XY, float e_kesi_Z, float e_kesi_Theta,
							CvMat* Integral_e_Kesi, float InnerLoopDuration, int temp);

		//
		void ControlSignal_SMC(CvMat* vel_des, CvMat* S, CvMat* Lambda, CvMat* C, CvMat* e_kesi_XY, float e_kesi_Z,
								float e_kesi_Theta, CvMat* Jacobian_XY, float Jacobian_Z, float Jacobian_Theta,
								CvMat* Integral_e_Kesi, float InnerLoopDuration, int temp);

		void StartController();
		
		void Terminate();
		
		void SetTargetPosition();
		
		void ComputeControlSignal();

        //************added by parisa*************************************
        void ControlSignal_P(CvMat* vel_des);

        void mugrad_calculator(CvMat* image, float c, int p, int q, float mu_grad[4]);

        void mgrad_calculator(CvMat* image, float c, int p, int q, float m_grad[4]);

        void L_calculator(CvMat* image, CvMat* m00_star, CvMat* L_interaction,CvMat* invL_interaction);

        void s_moment_calculator(CvMat* imgt, CvMat* m00_star, CvMat* s_moment);

        void Kinematic(float teta1,float teta2,float teta3,float teta5,float teta6,float lgantry,float *position);

        unsigned int  program_define;
        //***************************************************************

        CvMat* Thresholded_Image; //Thresholded image

		CvMat* Scale_Furier_Image; //Scaled Furier transform of image

		CvMat* Scale_Furier_Image_z; //Scaled Furier transform of image in z derection

		float Current_Joints[6]; // Get from Robot
		float Target_Joints[6]; // Set Positoon for Robot
		float Current_Gantry_Pos; // Get from Robot
		float Target_Gantry_Pos; // Set Position for Robot
        float z_star; //added by parisa
	
	private:
		//Kernel_XY
		/*int i,j,z,k;
		double kx, ky, sumx, sumy, sumdotx, sumdoty, d;
		float sigx, sigy, mux, muy, Jazr2Pi;
		CvMat* ger;*/

		// Kernel_Z
		/*int a, v1, v2;
		float kz, b, a1, sumz;*/
		
		//Kernel_Theta
		/*int a2, kr;
		float sumr, t1, t2;
		CvMat* gerr;*/
		
		//
		CvMat* kesi_XY_Target;
		CvMat* kesi_XY;
		CvMat* Error_kesi_XY;
		CvMat* Jacobian_XY;
		CvMat* GER;
        CvMat* s_moment_Target;
        CvMat* s_moment;
        CvMat* L_interaction;
        CvMat* invL_interaction;
        CvMat* Error_s_moment;
        CvMat* m00_star;
        CvMat* Control_Signal;
		
		float kesi_Z_Target, kesi_Z, Error_kesi_Z, Jacobian_Z;

		float kesi_Theta_Target, kesi_Theta, Error_kesi_Theta, Jacobian_Theta;

		CvMat* Desired_cartesian_velocity;
		CvMat* Desired_joint_velocity;
		float LEE, SampleTime,InnerLoopDuration;
		int Error_Status;

		boost::thread mThread;
		boost::mutex mMutex;

		CvMat* S;
		CvMat* Lambda;
		CvMat* C;
		CvMat* Integral_e_Kesi;

        //FILE  *fp_s, *fp_v, *fp_sums, *fp_kesi, *fp_ekesi, *fp_joints, *fp_targets, *fp_jacobians;
        FILE  *fp_s, *fp_v, *fp_es, *fp_joints;

};
