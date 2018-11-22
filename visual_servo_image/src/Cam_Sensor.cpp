#include "Cam_Sensor.h"






CamSensorManager::CamSensorManager()
{
}

CamSensorManager::~CamSensorManager()
{
};

void CamSensorManager::InitializeCamera()
{
    //Capture = cvCaptureFromCAM(CV_CAP_ANY); // Open Caamera
    //Image = cvRetrieveFrame(Capture); // Give image

    Matrix_Image = cvCreateMat(Image->height, Image->width, CV_32FC1 );
    Calibrated_Image = cvCreateMat(Image->height, Image->width, CV_32FC1 );
    Thresholded_Image = cvCreateMat(Image->height, Image->width, CV_32FC1 );
    Furier_Image = cvCreateMat(Image->height, Image->width, CV_32FC1 );
    Scale_Furier_Image = cvCreateMat(Image->height, Image->width, CV_32FC1 );
    Scale_Furier_Image_z =  cvCreateMat(Image->height, Image->width, CV_32FC1 );
    imgf_z = cvCreateMat(Thresholded_Image->height, Thresholded_Image->width, CV_32FC1 ); // It is for fft in z direction

    dft_M = cvGetOptimalDFTSize( Thresholded_Image->height - 1 );
    dft_N = cvGetOptimalDFTSize( Thresholded_Image->width - 1 );

    dft_A = cvCreateMat( dft_M, dft_N, CV_64FC2 );
    image_Re = cvCreateImage( cvSize(dft_N, dft_M), IPL_DEPTH_64F, 1);
    image_Im = cvCreateImage( cvSize(dft_N, dft_M), IPL_DEPTH_64F, 1);
    image_magnitude = cvCreateImage( cvSize(dft_N, dft_M), IPL_DEPTH_64F, 1);

    Gray_Image = cvCreateImage( cvSize(Image->width,Image->height), 8, 1 );
    realInput = cvCreateImage( cvGetSize(Thresholded_Image), IPL_DEPTH_64F, 1);
    imaginaryInput = cvCreateImage( cvGetSize(Thresholded_Image), IPL_DEPTH_64F, 1);
    complexInput = cvCreateImage( cvGetSize(Thresholded_Image), IPL_DEPTH_64F, 2);


    cvNamedWindow("Original Image", CV_WINDOW_AUTOSIZE); // Create new window for orginal image
    cvMoveWindow("Original Image", 1253 - 2*Image->width, 0); // move to propriate position
    cvNamedWindow("Gray Sclae Image", CV_WINDOW_AUTOSIZE);// Create new window gray scale image
    cvMoveWindow("Gray Sclae Image", 1266 - Image->width, 0);
    cvNamedWindow("Threshold Image", CV_WINDOW_AUTOSIZE); // Create new window for threshold image
    cvResizeWindow("Threshold Image", Image->width, Image->height );
    cvMoveWindow("Threshold Image", 1253 - 2*Image->width, Image->height + 39); // move to propriate position
    cvNamedWindow("Furier Image", CV_WINDOW_AUTOSIZE);// Create new window calibrated image
    cvResizeWindow("Furier Image", Image->width, Image->height );
    cvMoveWindow("Furier Image", 1266 - Image->width, Image->height + 39);
}

void CamSensorManager::Calibration(CvMat* img,CvMat* calibrated_img)
{
    float k1=(float)-0.33060;float k2=(float)0.11170;
    //float xc=162.4085;float yc=127.1973;
    float cx = (float)((img->height)/2);
    float cy = (float)((img->width)/2);
    float fx=(float)195.0507;
    float fy=(float)195.0992;
    float x,y,k,r2,r4;
    int i,j,u,v;
    float X,Y,W,xp,yp,xz,yz;
    int a,b;
    CvScalar ss;

    for(i=0;i<img->height;i++)
        for(j=0;j<img->width;j++) {
            //d = CV_MAT_ELEM(*(img),float,j,i);
            u = i;
            v = j;
            x=(u-cx)/fx;
            y=(v-cy)/fy;
            X=x;
            Y=y;
            W=1;
            xp = X/W;
            yp = Y/W;

            r2=(x*x)+(y*y);
            r4=r2*r2;
            k=1+k1*r2+k2*r4;

            xz = xp*k;
            yz = yp*k;
            a =  cvRound( xz*fx+cx );
            b =  cvRound( yz*fy+cy );

            if(a>0 && b>0)
                //CV_MAT_ELEM(*(calibrated_img),float,b,a)=CV_MAT_ELEM(*(img),float,j,i);
            {
                ss=cvGet2D(img,a,b);
                CV_MAT_ELEM(*(calibrated_img),float,i,j)=(float)ss.val[0];
            }

        //*( (float*)CV_MAT_ELEM_PTR( *calibrated_points,  2*i, 0 ) ) =(float)(x*fx) ;
        //*( (float*)CV_MAT_ELEM_PTR( *calibrated_points,  (2*i)+1, 0 ) ) =(float)((y)*fy) ;
        }
}

void CamSensorManager::Thresh(CvMat* img,CvMat* tr_img)
{
    CvScalar s;
    int i,j;
    for(i = 0; i < img->height; i++)
        for(j = 0; j < img->width; j++) {
            s = cvGet2D(img,i,j);
            if (((float)s.val[0]) >= 180)
                CV_MAT_ELEM(*(tr_img),float,i,j) = 0;
            else
                CV_MAT_ELEM(*(tr_img),float,i,j) = 255;
    }
}

void CamSensorManager::FFTofImage(CvMat* imgt, CvMat* imgf, CvMat* imgf_scale, CvMat* imgf_scale_z)
{

    cvScale(imgt, realInput, 1.0, 0.0);
    cvZero(imaginaryInput);
    cvMerge(realInput, imaginaryInput, NULL, NULL, complexInput);

    // copy A to dft_A and pad dft_A with zeros
    cvGetSubRect( dft_A, &tmp, cvRect(0,0, imgt->width, imgt->height));
    cvCopy( complexInput, &tmp, NULL );
    if( dft_A->cols > imgt->width )
    {
        cvGetSubRect( dft_A, &tmp, cvRect(imgt->width,0, dft_A->cols - imgt->width, imgt->height));
        cvZero( &tmp );
    }

    cvDFT( dft_A, dft_A, CV_DXT_FORWARD, complexInput->height ); // no need to pad bottom part of dft_A with zeros because of
    // use nonzero_rows parameter in cvDFT() call below
    cvSplit( dft_A, image_Re, image_Im, 0, 0 ); // Split Fourier in real and imaginary parts
    // Compute the magnitude of the spectrum Mag = sqrt(Re^2 + Im^2)
    cvPow( image_Re, image_Re, 2.0);
    cvPow( image_Im, image_Im, 2.0);
    cvAdd( image_Re, image_Im,image_magnitude, NULL);
    cvPow( image_magnitude, image_magnitude, 0.5 );

    cvConvert(image_magnitude,imgf); //convert an image to a matrix
    for(int i=0;i<imgf->height;i++)
    {
            for(int j=0;j<imgf->width;j++)
            {
                CV_MAT_ELEM(*(imgf_scale),float,i,j) = (float)0.00001 * CV_MAT_ELEM(*(imgf),float,i,j);
                //CV_MAT_ELEM(*(imgf_new),float,i,j) = log(CV_MAT_ELEM(*(dft_A_scale),float,i,j)+1);
            }
    }
    cvDFT(imgt, imgf_z, pipi, 320);
    for(int i=0;i<imgf_z->height;i++)
    {
        for(int j=0;j<imgf_z->width;j++)
        {
            CV_MAT_ELEM(*(imgf_scale_z),float,i,j) = (float)0.0001*CV_MAT_ELEM(*(imgf_z),float,i,j);
            CV_MAT_ELEM(*(imgf_scale_z),float,i,j) = abs(CV_MAT_ELEM(*(imgf_scale_z),float,i,j));
        }
    }
}

void CamSensorManager::StartCamera()
{
    mThread = boost::thread(&CamSensorManager::ImageShow, this);
}
void CamSensorManager::ImageShow()
{
    //this->InitializeCamera();
    //while(1)
    //{
        //Image = cvRetrieveFrame(Capture); // Give image
        cvCvtColor( Image, Gray_Image, CV_BGR2GRAY ); // Convert image to gray scale
        cvConvert(Gray_Image,Matrix_Image);   // Convert an image to a matrix
        //this->Calibration(Matrix_Image , Calibrated_Image);	// Calibration of Image
        this->Thresh(Matrix_Image , Thresholded_Image);		// Transform into black & white
        this->FFTofImage(Thresholded_Image, Furier_Image,Scale_Furier_Image, Scale_Furier_Image_z);

        cvShowImage("Furier Image", Scale_Furier_Image);
        cvWaitKey(1);
        cvShowImage("Original Image", Image);
        cvWaitKey(1);
        cvShowImage("Gray Sclae Image", Gray_Image);
        cvWaitKey(1);
        cvShowImage("Threshold Image", Thresholded_Image);
        cvWaitKey(1);
    //}
}
