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
#define pipi 0 

class CamSensorManager
{
	public:
		CamSensorManager();

		~CamSensorManager();
		
        void InitializeCamera();
		
		void Calibration (CvMat* img,CvMat* calibrated_img);
		
		void ImageShow();

		void Thresh(CvMat* img,CvMat* tr_img);

		void FFTofImage(CvMat* imgt, CvMat* imgf, CvMat* imgf_scale, CvMat* imgf_scale_z);
		
		void StartCamera();

		IplImage* Image; // Original image

		IplImage* Gray_Image; // Gray scale image
	
		CvCapture* Capture; // Handle for camera

		CvMat* Matrix_Image; // Matrixed image

		CvMat* Calibrated_Image; // Calibrated image

		CvMat* Thresholded_Image; // Thresholded image

		CvMat* Furier_Image; //Furier transform of image

		CvMat* Scale_Furier_Image; //Scaled Furier transform of image

		CvMat* Scale_Furier_Image_z; //Scaled Furier transform of image in z derection

        bool Image_Captured;
	
	private:
	
        int dft_M, dft_N;

		CvMat* dft_A, tmp;

		CvMat* imgf_z;

		IplImage* realInput;

		IplImage* imaginaryInput;

		IplImage* complexInput;

		IplImage* image_Re;

		IplImage* image_Im;

		IplImage* image_magnitude;

		boost::thread mThread;

		boost::mutex mMutex;

};
