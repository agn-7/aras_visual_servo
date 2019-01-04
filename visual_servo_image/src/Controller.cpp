#include "visual_servo_image/Controller.h"


ControllerManager::ControllerManager()
{
}

ControllerManager::~ControllerManager()
{
   // this->Terminate();
  //  cout << "End of Visual Servoing" << endl;
}
void ControllerManager::ControllerInitialize()
{
    //********************added by parisa**************************
    L_interaction     = cvCreateMat(4, 4, CV_32FC1 );
    invL_interaction  = cvCreateMat(4, 4, CV_32FC1 );
    s_moment_Target   = cvCreateMat(4, 1, CV_32FC1 );
    s_moment          = cvCreateMat(4, 1, CV_32FC1 );
    Error_s_moment    = cvCreateMat(4, 1, CV_32FC1 );
    m00_star          = cvCreateMat(1, 1, CV_32FC1 );
    Control_Signal    = cvCreateMat(4, 1, CV_32FC1 );
    //************************************************************
    kesi_XY_Target    = cvCreateMat(2, 1, CV_32FC1 );
    kesi_XY           = cvCreateMat(2, 1, CV_32FC1 );
    Error_kesi_XY     = cvCreateMat(2, 1, CV_32FC1 );
    Jacobian_XY       = cvCreateMat(2, 2, CV_32FC1 );
    GER               = cvCreateMat(2 , 2, CV_32FC1 );
	
	kesi_Z_Target = 0; kesi_Z = 0; Error_kesi_Z = 0; Jacobian_Z = 0;

	kesi_Theta_Target = 0; kesi_Theta = 0; Error_kesi_Theta = 0; Jacobian_Theta = 0;
	
	Desired_cartesian_velocity = cvCreateMat(5, 1, CV_32FC1 );
    Desired_joint_velocity     = cvCreateMat(6 , 1, CV_32FC1 );
    LEE                        = (float)(7.2+(18-14.03+0.253+0.46));
    SampleTime                 = 0.002;

    S                          = cvCreateMat(5, 1, CV_32FC1 );
    Lambda                     = cvCreateMat(4, 1, CV_32FC1 );
    C                          = cvCreateMat(4, 1, CV_32FC1 );
    Integral_e_Kesi            = cvCreateMat(5, 1, CV_32FC1 );

    if(program_define== 1)
    {
        CV_MAT_ELEM(*(Lambda),float,0,0) = 1; //0.5;//0.5;//1 for other experiments and 2
        //for robust method	// Lambda in X Direction
        CV_MAT_ELEM(*(Lambda),float,1,0) = 1;   					// Lambda in Y Direction
        CV_MAT_ELEM(*(Lambda),float,2,0) = 20; //30;//30;//*2;		// Lambda in Z Direction
        CV_MAT_ELEM(*(Lambda),float,3,0) = 3.5;//0.5;//0.05;//3.5;	// Lambda in Theta Orientation
    }
    else
    {
        CV_MAT_ELEM(*(Lambda),float,0,0) = 300;		// Lambda in X Direction
        CV_MAT_ELEM(*(Lambda),float,1,0) = 300;   	// Lambda in Y Direction
        CV_MAT_ELEM(*(Lambda),float,2,0) = 500;		// Lambda in Z Direction
        CV_MAT_ELEM(*(Lambda),float,3,0) = 100;		// Lambda in Theta Orientation.
    }

	CV_MAT_ELEM(*(C),float,0,0) = 1;		// C in X Direction
	CV_MAT_ELEM(*(C),float,1,0) = 1;		// C in Y Direction
	CV_MAT_ELEM(*(C),float,2,0) = -1;		// C in Z Direction
	CV_MAT_ELEM(*(C),float,3,0) = 1;		// C in Theta Orientation
	InnerLoopDuration = (float)(0.001);

    const char *homedir;

    if ((homedir = getenv("HOME")) == NULL) {
        homedir = getpwuid(getuid())->pw_dir;
    }

    //aGn_packages
    std::string path1 = (std::string(homedir) + "/catkin_ws/src/aras_visual_servo/Results/s.m");
    std::string path2 = (std::string(homedir) + "catkin_ws/src/aras_visual_servo/Results/velocity.m");
    std::string path3 = (std::string(homedir) + "/catkin_ws/src/aras_visual_servo/Results/error_s.m");
    std::string path4 = (std::string(homedir) + "/catkin_ws/src/aras_visual_servo/Results/joints.m");

    fp_s = fopen(path1.c_str(), "w");
    fprintf(fp_s,"S = [   %%S_MOMENT\n");

    //fprintf(fp_s,"S = [   %%Sliding Surface\n");
    //fp_sums = fopen("/home/parisa/catkin_ws/src/aras_visual_servo/Results/sum_s.m","w");
    //fprintf(fp_sums,"Sum_s = [   %%integral of Kesi\n");

    fp_v = fopen(path2.c_str(), "w");
	fprintf(fp_v,"Velocity = [   %%Cartesian Velocity\n");

    //fp_kesi = fopen("/home/parisa/catkin_ws/src/aras_visual_servo/Results/kesi.m","w");
    //fprintf(fp_kesi,"Kesi = [   %%Kesi\n");
    //fp_ekesi = fopen("/home/parisa/catkin_ws/src/aras_visual_servo/Results/errorofkesi.m","w");
    //fprintf(fp_ekesi,"Errorofkesi = [   %%Error of Kesi\n");

    fp_es = fopen(path3.c_str(), "w");
    fprintf(fp_es,"Error_S = [   %%Error_S\n");


    fp_joints = fopen(path4.c_str(), "w");
	fprintf(fp_joints,"Joint = [   %%Joints value\n");

    //fp_jacobians = fopen("/home/parisa/catkin_ws/src/aras_visual_servo/Results/jacobians.m","w");
    //fprintf(fp_jacobians,"Jacobian = [   %%Jacobians value\n");
	
}

void ControllerManager::Kernel_XY(CvMat* image, CvMat* kesi_XY , CvMat* Ger, CvMat* Jacobian_XY)
{
	int i,j,z,k;
	double kx, ky, sumx, sumy, sumdotx, sumdoty, d;
	float sigx, sigy, mux, muy, Jazr2Pi;
	CvMat* ger = cvCreateMat(2 , 2, CV_32FC1 );
	kx = 0; ky = 0; sumx = 0; sumy = 0; sumdotx = 0; sumdoty = 0;
	sigx = 70; sigy = 70; mux = -100; muy = -100; Jazr2Pi = sqrt(2*pi); d = 0;
	CV_MAT_ELEM(*(ger),float,1,0) = 0;	CV_MAT_ELEM(*(ger),float,0,1) = 0;

	for(i = 0; i<image->height; i++) 
	{ 
		kx = (1/(sigx * Jazr2Pi))* exp(-(((i+1)-mux)*((i+1)-mux))/(2*sigx*sigx));
		CV_MAT_ELEM(*(ger),float,0,0) = (float)(kx * (-1) * (i+1-mux)/(sigx*sigx));
		for(j = 0; j<image->width; j++) 
		{
			ky = (1/(sigy * Jazr2Pi))* exp(-(((j+1)-muy)*((j+1)-muy))/(2*sigy*sigy));
			CV_MAT_ELEM(*(ger),float,1,1) = (float)(ky * (-1) * (j+1-muy)/(sigy*sigy));
			//d = cvGetReal2D(image,j,i);
			d = CV_MAT_ELEM(*(image),float,i,j);
			sumx = sumx + kx * d;
			sumy = sumy + ky * d;
			sumdotx=sumdotx+CV_MAT_ELEM(*(ger),float,0,0)* d;//added by mahsa
			sumdoty=sumdoty+CV_MAT_ELEM(*(ger),float,1,1)* d;//added by mahsa
			for(z = 0; z<2; z++)
				for(k = 0; k<2; k++)
					CV_MAT_ELEM(*(Ger),float,z,k) = (float)d*CV_MAT_ELEM(*(ger),float,z,k)+ CV_MAT_ELEM(*(Ger),float,z,k);
			//cvAddWeighted(Ger, 1 , ger , d , 0 , Ger);
		}
	}
    //CV_MAT_ELEM(*(kesi_XY),float,0,0) = (float) sumx;
    //CV_MAT_ELEM(*(kesi_XY),float,1,0) = (float) sumy;

    //CV_MAT_ELEM(*(Jacobian_XY),float,0,0) = (float) sumdotx;	//added by mahsa
    //CV_MAT_ELEM(*(Jacobian_XY),float,0,1) = 0;				//added by mahsa
    //CV_MAT_ELEM(*(Jacobian_XY),float,1,0) = 0;				//added by mahsa
    //CV_MAT_ELEM(*(Jacobian_XY),float,1,1) = (float) sumdoty;	//added by mahsa
	cvReleaseMat(&ger);
}

void ControllerManager::Kernel_Z(CvMat* Fimage, float &kesiz , float &Gerz)
{
	int a, v1, v2;
	float kz, b, a1, sumz, d;
	a1 = 0; sumz = 0; d = 0;
	Gerz = 0; // Added by Javad
	for(v1 = 0; v1<Fimage->height; v1++)
		for(v2 = 0; v2<Fimage->width; v2++) 
		{
			a = (v1+1)*(v1+1)+(v2+1)*(v2+1);
			kz = (float)expl(-(0.125) * a);
			a1=sqrt((float)a);
			
			d = CV_MAT_ELEM(*(Fimage),float,v1,v2);
			sumz = sumz + kz * d;
			b = (float)0.25*a1*kz;
			Gerz = Gerz + d * b ;	
		}
	kesiz = sumz;
}

void ControllerManager::Kernel_Theta(CvMat* Fimage, float &kesi_Theta , float &Gerr)
{
    int a1, a2, v1, v2;
	float kr, sumr, t1, t2, d, b;
	CvMat* gerr = cvCreateMat(2, 1, CV_32FC1 );
	sumr = 0; t1 = 0; t2 = 0; d = 0; b = 0;
	Gerr = 0; // Added by Javad
	for(v1 = 0; v1<Fimage->height; v1++)
		for(v2 = 0; v2<Fimage->width; v2++) 
		{
			a1 = (v1+1)*(v1+1);
			a2 = (v2+1)*(v2+1);
			kr = (float)(expl(-(0.125) * a1) + expl(-(0.125) * a2));
			 
			//CV_MAT_ELEM(*(gerr),float,0,0) = expl(-(0.125) * a1) * -0.25 * (v1+1);
			//CV_MAT_ELEM(*(gerr),float,1,0) = expl(-(0.125) * a2) * -0.25 * (v2+1);
			d = CV_MAT_ELEM(*(Fimage),float,v1,v2);
			sumr = sumr + kr * d;
			//b = ((CV_MAT_ELEM(*(gerr),float,0,0)) * (-(v2+1)) + (CV_MAT_ELEM(*(gerr),float,1,0)) * (v1+1) );
			t1 = (float)expl(-(0.125) * a1) * (v1+1);
			t2 = (float)expl(-(0.125) * a2) * (v2+1);
			//b = (0.25) * ((v2+1) * t1 - (v1+1) * t2);
			b = -(float)0.25 * (t1/(v1+1) + t2/(v2+1));
			Gerr = Gerr + d * b ;
			
		}
	kesi_Theta = sumr;
	cvReleaseMat(&gerr);
}

void ControllerManager::cvSubb_soli(CvMat* measurement,CvMat* nonlinear_measurement,CvMat* Z_G)
{
	int i,j;
	float a,b,c;
		for(i=0;i<measurement->rows;i++) 
			for(j=0;j<measurement->cols;j++) 
			{
				a = (CV_MAT_ELEM(*(measurement), float,i,j));
				b = (CV_MAT_ELEM(*(nonlinear_measurement), float,i,j));
				c=a-b;
				*( (float*)CV_MAT_ELEM_PTR( *Z_G,i,j) )= c;
			}
}
void ControllerManager::SingularityAvoidance(CvMat* xdesired_dot,float lgantry,float teta1,
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
    cvMatMulAdd (Jacplus,xdesired_dot, 0,qdesired_dot);
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
void ControllerManager::ControlSignal_KBVS(CvMat* vel_des, CvMat* e_kesi_XY, float e_kesi_Z, float e_kesi_Theta,
										   CvMat* Jacobian_XY, float Jacobian_Z, float Jacobian_Theta, int temp)
{
	CvMat* v_xy = cvCreateMat(2, 1, CV_32FC1 );

		switch (temp) 
		{
		case 1: 
				for(int i = 0; i<2; i++)
					CV_MAT_ELEM(*(vel_des),float,i,0)=0.0;											// Control Signal in X-Y Directions	
				CV_MAT_ELEM(*(vel_des),float,2,0)=-Jacobian_Z * e_kesi_Z * (float)0.001;				// Control Signal in Z Direction
				CV_MAT_ELEM(*(vel_des),float,3,0)=0.0;												// Control Signal in Theta Orientation
				break;
		case 2: 
				for(int i = 0; i<2; i++)
					CV_MAT_ELEM(*(vel_des),float,i,0)=0.0;											// Control Signal in X-Y Directions	
				CV_MAT_ELEM(*(vel_des),float,2,0)=0.0;											    // Control Signal in Z Direction
				CV_MAT_ELEM(*(vel_des),float,3,0)=-e_kesi_Theta * Jacobian_Theta * float(0.00001);			// Control Signal in Theta Orientation				
				break;
		case 3: 
				cvMatMulAdd (Jacobian_XY ,e_kesi_XY,0,v_xy); 
				CV_MAT_ELEM(*(vel_des),float,0,0) = -(float)1.2*CV_MAT_ELEM(*(v_xy),float,0,0);		// Control Signal in X Direction
				CV_MAT_ELEM(*(vel_des),float,1,0) = (float)10*CV_MAT_ELEM(*(v_xy),float,1,0);		// Control Signal in Y Direction
				CV_MAT_ELEM(*(vel_des),float,2,0)=0.0;												// Control Signal in Z Direction
				CV_MAT_ELEM(*(vel_des),float,3,0)=0.0;												// Control Signal in Theta Orientation	
				break;
		}
		CV_MAT_ELEM(*(vel_des),float,4,0)=0.0;														// Control Signal in B Orientation
		cvReleaseMat(&v_xy);		
}

void ControllerManager::SlidingSurface(CvMat* S, CvMat* Lambda, CvMat* e_kesi_XY, float e_kesi_Z, float e_kesi_Theta,
							CvMat* Integral_e_Kesi, float InnerLoopDuration, int temp)
{
    //float C_X, C_Y, C_Z, C_Theta;

	float Lambda_X = CV_MAT_ELEM(*(Lambda),float,0,0);
	float Lambda_Y = CV_MAT_ELEM(*(Lambda),float,1,0);
	float Lambda_Z = CV_MAT_ELEM(*(Lambda),float,2,0);
	float Lambda_Theta = CV_MAT_ELEM(*(Lambda),float,3,0);

	switch (temp) {
	case 1: {
			CV_MAT_ELEM(*(S),float,2,0) = e_kesi_Z + Lambda_Z * CV_MAT_ELEM(*(Integral_e_Kesi),float,2,0);							  // Sliding Surface in Z Direction	
						
			//CV_MAT_ELEM(*(Integral_e_Kesi),float,2,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,2,0) + InnerLoopDuration*e_kesi_Z;				// Integral_e_Kesi in Z Direction
			}
	break;
	case 2: {
			CV_MAT_ELEM(*(S),float,3,0) = e_kesi_Theta + Lambda_Theta * CV_MAT_ELEM(*(Integral_e_Kesi),float,3,0);					  // Sliding Surface in Theta Orientation
						
			//CV_MAT_ELEM(*(Integral_e_Kesi),float,3,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,3,0) + InnerLoopDuration*e_kesi_Theta;			// Integral_e_Kesi in Theta Orientation
			}
	break;
	case 3: {
			
			CV_MAT_ELEM(*(S),float,0,0) = CV_MAT_ELEM(*(e_kesi_XY),float,0,0) + Lambda_X * CV_MAT_ELEM(*(Integral_e_Kesi),float,0,0); // Sliding Surface in X Direction	
			CV_MAT_ELEM(*(S),float,1,0) = CV_MAT_ELEM(*(e_kesi_XY),float,1,0) + Lambda_Y * CV_MAT_ELEM(*(Integral_e_Kesi),float,1,0); // Sliding Surface in Y Direction
						
			//CV_MAT_ELEM(*(Integral_e_Kesi),float,0,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,0,0) + CV_MAT_ELEM(*(e_kesi_XY),float,0,0);	// Integral_e_Kesi in X Direction
			//CV_MAT_ELEM(*(Integral_e_Kesi),float,1,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,1,0) + CV_MAT_ELEM(*(e_kesi_XY),float,1,0);	// Integral_e_Kesi in Y Direction
			}
	}
	CV_MAT_ELEM(*(S),float,4,0) = 0;
}

void ControllerManager::ControlSignal_SMC(CvMat* vel_des, CvMat* S, CvMat* Lambda, CvMat* C, CvMat* e_kesi_XY, float e_kesi_Z,
								float e_kesi_Theta, CvMat* Jacobian_XY, float Jacobian_Z, float Jacobian_Theta,
								CvMat* Integral_e_Kesi, float InnerLoopDuration, int temp)
{
	CvMat* v_des = cvCreateMat(2, 1, CV_32FC1 );
	CvMat* J = cvCreateMat(2 , 2, CV_32FC1 );
	float vx_des, vy_des, vz_des, vA_des;
	float C_X, C_Y, C_Z, C_Theta;
    int i;

	float Lambda_X = CV_MAT_ELEM(*(Lambda),float,0,0);
	float Lambda_Y = CV_MAT_ELEM(*(Lambda),float,1,0);
	float Lambda_Z = CV_MAT_ELEM(*(Lambda),float,2,0);
	float Lambda_Theta = CV_MAT_ELEM(*(Lambda),float,3,0);

	C_X = CV_MAT_ELEM(*(C),float,0,0);
	C_Y = CV_MAT_ELEM(*(C),float,1,0);
	C_Z = CV_MAT_ELEM(*(C),float,2,0);
	C_Theta = CV_MAT_ELEM(*(C),float,3,0);
	
	switch (temp) {
	case 1: {
			vz_des = -Lambda_Z*e_kesi_Z/Jacobian_Z + C_Z*(CV_MAT_ELEM(*(S),float,2,0)/abs(CV_MAT_ELEM(*(S),float,2,0)));
			
            for(i = 0; i < 2; i++)
				CV_MAT_ELEM(*(vel_des),float,i,0) = 0.0;						// Control Signal in X-Y Directions
			CV_MAT_ELEM(*(vel_des),float,2,0) = vz_des;  	//20*vz_des;(Javad) // Control Signal in Z Direction
			CV_MAT_ELEM(*(vel_des),float,3,0) = 0.0;							// Control Signal in Theta Orientation
			}
	break;
	case 2: {
			vA_des = -Lambda_Theta*e_kesi_Theta/Jacobian_Theta + C_Theta*(CV_MAT_ELEM(*(S),float,3,0)/abs(CV_MAT_ELEM(*(S),float,3,0)));
			
            for( i = 0; i < 2; i++)
               CV_MAT_ELEM(*(vel_des),float,i,0) = 0.0;							// Control Signal in X-Y Directions
			CV_MAT_ELEM(*(vel_des),float,2,0) = 0.0;								// Control Signal in Z Direction
			CV_MAT_ELEM(*(vel_des),float,3,0) = vA_des;								// Control Signal in Theta Orientation
			}
	break;
	case 3: {
			
			cvInvert (Jacobian_XY,J); 
			cvMatMulAdd (J, e_kesi_XY, 0, v_des);
			vx_des = -Lambda_X*CV_MAT_ELEM(*(v_des),float,0,0) + C_X*(CV_MAT_ELEM(*(S),float,0,0)/abs(CV_MAT_ELEM(*(S),float,0,0)));
			vy_des = -Lambda_Y*CV_MAT_ELEM(*(v_des),float,1,0) + C_Y*(CV_MAT_ELEM(*(S),float,1,0)/abs(CV_MAT_ELEM(*(S),float,1,0)));
			
			CV_MAT_ELEM(*(vel_des),float,0,0) = (float)1.2*vx_des;							// Control Signal in X Direction
			CV_MAT_ELEM(*(vel_des),float,1,0) = -(float)1*vy_des;						    // Control Signal in Y Direction
			CV_MAT_ELEM(*(vel_des),float,3,0) = 0.0;										// Control Signal in Theta Orientation
			CV_MAT_ELEM(*(vel_des),float,2,0) = 0.0;										// Control Signal in Z Direction

			}
	}
	CV_MAT_ELEM(*(vel_des),float,4,0) = 0.0;														// Control Signal in B Orientation

	cvReleaseMat(&v_des);
	cvReleaseMat(&J);

}
void ControllerManager::SetTargetPosition()
{
	this->ControllerInitialize();
    if(program_define== 1)
    {
        this->Kernel_XY(Thresholded_Image, kesi_XY_Target, GER, Jacobian_XY);
        this->Kernel_Z(Scale_Furier_Image_z, kesi_Z_Target, Jacobian_Z);
        this->Kernel_Theta(Scale_Furier_Image, kesi_Theta_Target, Jacobian_Theta);
    }
    else if(program_define== 3)
    {
        this->L_calculator(Thresholded_Image, m00_star, L_interaction, invL_interaction);
        this->s_moment_calculator(Thresholded_Image, m00_star, s_moment_Target);
    }
    else
    {

    }
//  fp_targets = fopen("/home/parisa/Myworkspace/Results/targets.m","w");
//	fprintf(fp_targets,"%%target values\n");
//	fprintf(fp_targets,"Kesi_target = [%f %f %f %f];\n",CV_MAT_ELEM( *kesi_XY_Target,float,0,0)
//					  ,CV_MAT_ELEM( *kesi_XY_Target,float,1,0), kesi_Z_Target, kesi_Theta_Target);
//	fprintf(fp_targets,"Joint_target=[%f %f %f %f %f %f];\n",Current_Joints[0], Current_Joints[1], Current_Joints[2]
//					  ,Current_Joints[4], Current_Joints[5], Current_Gantry_Pos);
//	fclose(fp_targets);

}

void ControllerManager::ComputeControlSignal()
{
    //while(1){
    float velocity_x = 0, velocity_y = 0;
    float Current_Position[6];
    float s1[4],s2[4],e[4]; //,w[7];
 //   float x,y;

    if(program_define== 1) //Mahsa program
    {
        // Compute kesies and Jacobians
        this->Kernel_XY(Thresholded_Image, kesi_XY, GER, Jacobian_XY);
        this->Kernel_Z(Scale_Furier_Image_z, kesi_Z, Jacobian_Z);
        this->Kernel_Theta(Scale_Furier_Image, kesi_Theta, Jacobian_Theta);

        // Compute Errors in kesis
        this->cvSubb_soli(kesi_XY, kesi_XY_Target, Error_kesi_XY);
        Error_kesi_Z = kesi_Z - kesi_Z_Target; // error of Kesi in z direction
        Error_kesi_Theta = kesi_Theta - kesi_Theta_Target; // error of Kesi in theta orientation

        if ( abs(Error_kesi_Z) >= 1)//10.26
            Error_Status = 1;
        else if ( abs(Error_kesi_Theta) >= 20)//25 18.28
            Error_Status = 2;
        else if ( abs(CV_MAT_ELEM(*(Error_kesi_XY),float,0,0)) >= 0.05||
                  abs(CV_MAT_ELEM(*(Error_kesi_XY),float,1,0)) >= 0.05 ) //1.91 0.29 0.5 0.2
            Error_Status = 3;
        else
            Error_Status = 0;
        // Compute Control signal(xdesired_dot) Kernel Based Visual Servoing Method
        //this->ControlSignal_KBVS(Desired_cartesian_velocity, Error_kesi_XY, Error_kesi_Z, Error_kesi_Theta,
        //		Jacobian_XY, Jacobian_Z, Jacobian_Theta, Error_Status);

        // Compute Control signal(xdesired_dot) Sliding Mode Method
        SlidingSurface(S, Lambda, Error_kesi_XY, Error_kesi_Z, Error_kesi_Theta, Integral_e_Kesi,
                       InnerLoopDuration, Error_Status);

        ControlSignal_SMC(Desired_cartesian_velocity, S, Lambda, C, Error_kesi_XY, Error_kesi_Z, Error_kesi_Theta,
                          Jacobian_XY, Jacobian_Z, Jacobian_Theta, Integral_e_Kesi, InnerLoopDuration, Error_Status);

        CV_MAT_ELEM(*(Integral_e_Kesi),float,0,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,0,0) + 0.01*CV_MAT_ELEM(*(Error_kesi_XY),float,0,0);	// Integral_e_Kesi in X Direction
        CV_MAT_ELEM(*(Integral_e_Kesi),float,1,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,1,0) + 0.01*CV_MAT_ELEM(*(Error_kesi_XY),float,1,0);	// Integral_e_Kesi in Y Direction
        CV_MAT_ELEM(*(Integral_e_Kesi),float,2,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,2,0) + 0.01*Error_kesi_Z;								// Integral_e_Kesi in Z Direction
        CV_MAT_ELEM(*(Integral_e_Kesi),float,3,0) = CV_MAT_ELEM(*(Integral_e_Kesi),float,3,0) + 0.01*Error_kesi_Theta;							// Integral_e_Kesi in Theta Orientation
        CV_MAT_ELEM(*(Integral_e_Kesi),float,4,0) = 0.0;
    }

    else if(program_define==3) //Parisa prgram
    {
        // Compute s of moments
        this->s_moment_calculator(Thresholded_Image, m00_star, s_moment);

        // Compute Errors in S_Moments
        this->cvSubb_soli(s_moment, s_moment_Target, Error_s_moment);
        s1[0] = CV_MAT_ELEM(*(s_moment), float, 0, 0);
        s1[1] = CV_MAT_ELEM(*(s_moment), float, 1, 0);
        s1[2] = CV_MAT_ELEM(*(s_moment), float, 2, 0);
        s1[3] = CV_MAT_ELEM(*(s_moment), float, 3, 0);

        s2[0] = CV_MAT_ELEM(*(s_moment_Target), float, 0, 0);
        s2[1] = CV_MAT_ELEM(*(s_moment_Target), float, 1, 0);
        s2[2] = CV_MAT_ELEM(*(s_moment_Target), float, 2, 0);
        s2[3] = CV_MAT_ELEM(*(s_moment_Target), float, 3, 0);

        e[0] = CV_MAT_ELEM(*(Error_s_moment), float, 0, 0);
        e[1] = CV_MAT_ELEM(*(Error_s_moment), float, 1, 0);
        e[2] = CV_MAT_ELEM(*(Error_s_moment), float, 2, 0);
        e[3] = CV_MAT_ELEM(*(Error_s_moment), float, 3, 0);

        // Compute Errors in S_Moments
        this->cvSubb_soli(s_moment, s_moment_Target, Error_s_moment);
        s1[0] = CV_MAT_ELEM(*(s_moment), float, 0, 0);
        s1[1] = CV_MAT_ELEM(*(s_moment), float, 1, 0);
        s1[2] = CV_MAT_ELEM(*(s_moment), float, 2, 0);
        s1[3] = CV_MAT_ELEM(*(s_moment), float, 3, 0);

        s2[0] = CV_MAT_ELEM(*(s_moment_Target), float, 0, 0);
        s2[1] = CV_MAT_ELEM(*(s_moment_Target), float, 1, 0);
        s2[2] = CV_MAT_ELEM(*(s_moment_Target), float, 2, 0);
        s2[3] = CV_MAT_ELEM(*(s_moment_Target), float, 3, 0);

        e[0] = CV_MAT_ELEM(*(Error_s_moment), float, 0, 0);
        e[1] = CV_MAT_ELEM(*(Error_s_moment), float, 1, 0);
        e[2] = CV_MAT_ELEM(*(Error_s_moment), float, 2, 0);
        e[3] = CV_MAT_ELEM(*(Error_s_moment), float, 3, 0);

        cout << "s moment target: ";
        for(int i = 0; i < 4; i++)
            cout << CV_MAT_ELEM(*(s_moment_Target),float,i,0) << " ";
        cout << endl;

        cout << "s moment: ";
        for(int i = 0; i < 4; i++)
            cout << CV_MAT_ELEM(*(s_moment),float,i,0) << " ";
        cout << endl;

        cout << "error: ";
        for(int i = 0; i < 4; i++)
            cout << CV_MAT_ELEM(*(Error_s_moment),float,i,0) << " ";
        cout << endl;

        this->ControlSignal_P(Desired_cartesian_velocity);
        // Velocity's obtained from Controller have described in camera coordinate,
        // so we must express them in world coordinate
        velocity_x = CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 0, 0);
        velocity_y = CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 1, 0);

        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 0, 0) = -velocity_y;
        CV_MAT_ELEM(*(Desired_cartesian_velocity), float, 1, 0) = velocity_x;


        cout << "Cartesian velocity: ";
        for(int i = 0; i < 5; i++)
            cout << CV_MAT_ELEM(*(Desired_cartesian_velocity),float,i,0) << " ";
        cout << endl;

    }
    else
    {
    }




		//Convert  Desired Cartesian velocity to desired joint velocity
		//%gain, k & lee are not defined
		// Desired joint velocity Format: [l_gant, theta1, theta2, theta3, theta5, theta6]
        this->SingularityAvoidance(Desired_cartesian_velocity, Current_Gantry_Pos,
                                    Current_Joints[0]*(pi/180), Current_Joints[1]*(pi/180), Current_Joints[2]*(pi/180),
									Current_Joints[4]*(pi/180), Current_Joints[5]*(pi/180), 0.12, 0.5, LEE, Desired_joint_velocity);

		
		// Compute Target joints value
		Target_Joints[0] = Current_Joints[0] + (CV_MAT_ELEM(*(Desired_joint_velocity), float, 1,0)*(180/pi)*SampleTime);
		Target_Joints[1] = Current_Joints[1] + (CV_MAT_ELEM(*(Desired_joint_velocity), float, 2,0)*(180/pi)*SampleTime);
		Target_Joints[2] = Current_Joints[2] + (CV_MAT_ELEM(*(Desired_joint_velocity), float, 3,0)*(180/pi)*SampleTime);
		Target_Joints[3] = 0;
		Target_Joints[4] = Current_Joints[4] + (CV_MAT_ELEM(*(Desired_joint_velocity), float, 4,0)*(180/pi)*SampleTime);
		Target_Joints[5] = Current_Joints[5] + (CV_MAT_ELEM(*(Desired_joint_velocity), float, 5,0)*(180/pi)*SampleTime);
		// minus sign may be vierd !!!!, but it is because of opposite direction of y axis and increase of gantry length
        Target_Gantry_Pos = Current_Gantry_Pos + (CV_MAT_ELEM(*(Desired_joint_velocity), float, 0,0)*SampleTime);

		// Saving
        //fprintf(fp_s,"%f %f %f %f %f\n",CV_MAT_ELEM( *S,float,0,0), CV_MAT_ELEM( *S,float,1,0),
        //					CV_MAT_ELEM( *S,float,2,0), CV_MAT_ELEM( *S,float,3,0),CV_MAT_ELEM( *S,float,4,0));
        fprintf(fp_s,"%f %f %f %f %f %f %f %f\n",CV_MAT_ELEM( *(s_moment),float,0,0), CV_MAT_ELEM( *(s_moment),float,1,0),
                                    CV_MAT_ELEM( *(s_moment),float,2,0),CV_MAT_ELEM( *(s_moment),float,3,0),
                                    CV_MAT_ELEM( *(s_moment_Target),float,0,0), CV_MAT_ELEM( *(s_moment_Target),float,1,0),
                                    CV_MAT_ELEM( *(s_moment_Target),float,2,0),CV_MAT_ELEM( *(s_moment_Target),float,3,0));

        fprintf(fp_v,"%f %f %f %f %f\n",CV_MAT_ELEM( *Desired_cartesian_velocity,float,0,0),
							CV_MAT_ELEM( *Desired_cartesian_velocity,float,1,0), CV_MAT_ELEM( *Desired_cartesian_velocity,float,2,0),
							CV_MAT_ELEM( *Desired_cartesian_velocity,float,3,0), CV_MAT_ELEM( *Desired_cartesian_velocity,float,4,0));

        fprintf(fp_es,"%f %f %f %f\n",CV_MAT_ELEM( *(Error_s_moment),float,0,0), CV_MAT_ELEM( *(Error_s_moment),float,1,0),
                                    CV_MAT_ELEM( *(Error_s_moment),float,2,0),CV_MAT_ELEM( *(Error_s_moment),float,3,0));
        //fprintf(fp_sums,"%f %f %f %f %f\n",CV_MAT_ELEM( *Integral_e_Kesi,float,0,0), CV_MAT_ELEM( *Integral_e_Kesi,float,1,0),
        //					CV_MAT_ELEM( *Integral_e_Kesi,float,2,0), CV_MAT_ELEM( *Integral_e_Kesi,float,3,0),
        //					CV_MAT_ELEM( *Integral_e_Kesi,float,4,0));
        //fprintf(fp_kesi,"%f %f %f %f\n",CV_MAT_ELEM( *kesi_XY,float,0,0), CV_MAT_ELEM( *kesi_XY,float,1,0), kesi_Z,
        //					kesi_Theta);
        //fprintf(fp_ekesi,"%f %f %f %f\n",CV_MAT_ELEM( *Error_kesi_XY,float,0,0), CV_MAT_ELEM( *Error_kesi_XY,float,1,0),
        //					Error_kesi_Z, Error_kesi_Theta);
		fprintf(fp_joints,"%f %f %f %f %f %f\n",Target_Joints[0], Target_Joints[1], Target_Joints[2],
							Target_Joints[4], Target_Joints[5], Target_Gantry_Pos);
        //fprintf(fp_jacobians,"%f %f %f %f\n",CV_MAT_ELEM( *Jacobian_XY,float,0,0), CV_MAT_ELEM( *Jacobian_XY,float,1,1),
        //					Jacobian_Z, Jacobian_Theta);
									
		// Prints
		cout << "************* Error_Status: " << Error_Status << "*************" << endl;
		//cout << "Kesi_x: " << CV_MAT_ELEM(*(kesi_XY),float,0,0) << " Kesi_y: " << CV_MAT_ELEM(*(kesi_XY),float,1,0)
		//	<< " kesi_z: " << kesi_Z << " kesi_Theta: " << kesi_Theta << endl;

		//cout << "Target_Kesi_x: " << CV_MAT_ELEM(*(kesi_XY_Target),float,0,0) << " Target_Kesi_y: " 
		//	 << CV_MAT_ELEM(*(kesi_XY_Target),float,1,0) << " Target_kesi_z: " << kesi_Z_Target 
		//	 << " Target_kesi_Theta: " << kesi_Theta_Target << endl;

		//cout << "eKesi_x: " << CV_MAT_ELEM(*(Error_kesi_XY),float,0,0) << " eKesi_y: " << CV_MAT_ELEM(*(Error_kesi_XY),float,1,0)
		//	<< " ekesi_z: " << Error_kesi_Z << " ekesi_Theta: " << Error_kesi_Theta << endl;
		//
//        cout << "Cartesian velocity: ";
//        for(int i = 0; i < 5; i++)
//            cout << CV_MAT_ELEM(*(Desired_cartesian_velocity),float,i,0) << " ";
//        cout << endl;

        cout << "Joint velocity: ";
        for(int i = 0; i < 6; i++)
            cout << CV_MAT_ELEM(*(Desired_joint_velocity),float,i,0) << " ";
        cout << endl;

		//cout << "Current:";
		//for(int j = 0; j < 6; j++) // Print Readed joints
		//	cout << Current_Joints[j] << " ";
		//std::cout << std::endl;
		//
        cout << "Target:";
        for(int j = 0; j < 6; j++) // Print Target joints
            cout << Target_Joints[j] << " ";
        std::cout << std::endl;

        cout << "Target Gantary:" << Target_Gantry_Pos << endl;
	//}
		
}

void ControllerManager::StartController()
{
	this->ControllerInitialize();
	mThread = boost::thread(&ControllerManager::ComputeControlSignal, this);
}

void ControllerManager::Terminate()
{
	fprintf(fp_s,"];\n");
    //fprintf(fp_sums,"];\n");
	fprintf(fp_v,"];\n");
    fprintf(fp_es,"];\n");
    //fprintf(fp_kesi,"];\n");
    //fprintf(fp_ekesi,"];\n");
    fprintf(fp_joints,"];\n");
    //fprintf(fp_jacobians,"];\n");
    fclose(fp_s);
    //fclose(fp_sums);
    fclose(fp_v);
    //fclose(fp_kesi);
    //fclose(fp_ekesi);
    fclose(fp_es);
    fclose(fp_joints);
    //fclose(fp_jacobians);
}

// **************************************************************************
//**********************************Parisa***********************************
//***************************************************************************
void ControllerManager::s_moment_calculator(CvMat* imgt, CvMat* m00_star, CvMat* s_moment)
{
    float an,xg,yg,y,x;
    float s[4];
    float m00_star_target;
    m00_star_target = CV_MAT_ELEM(*(m00_star),float,0,0);
    CvMoments MOMENTS;
    cvMoments(imgt, &MOMENTS, true);
       an = z_star * sqrt(m00_star_target/MOMENTS.m00);
       xg = MOMENTS.m10/MOMENTS.m00;
       yg = MOMENTS.m01/MOMENTS.m00;

//        cout << "m00:" << MOMENTS.m00 << std::endl;
//        cout << "m10:" << MOMENTS.m10 << std::endl;
//        cout << "m01:" << MOMENTS.m01 << std::endl;

CV_MAT_ELEM(*(s_moment),float,0,0)   = (float)xg*an;//sx
CV_MAT_ELEM(*(s_moment),float,1,0)   = (float)yg*an;//sy
CV_MAT_ELEM(*(s_moment),float,2,0)   = (float)an;   //sz
y =(float)(2*MOMENTS.mu11);
x = (MOMENTS.mu02+MOMENTS.mu20);
CV_MAT_ELEM(*(s_moment),float,3,0)   = atan2(y,x); //sw

s[0] = CV_MAT_ELEM(*(s_moment),float,0,0);
s[1] = CV_MAT_ELEM(*(s_moment),float,1,0);
s[2] = CV_MAT_ELEM(*(s_moment),float,2,0);
s[3] = CV_MAT_ELEM(*(s_moment),float,3,0);

}

void ControllerManager::L_calculator(CvMat* image, CvMat* m00_star, CvMat* L_interaction,CvMat* invL_interaction)
{
        float s_star[4];
        float m00_dot[4],m10_dot[4],m01_dot[4],mu11_dot[4],mu20_dot[4],mu02_dot[4];
        float L[4][4];
        float L1[4][4];

        CvMat* s_moment =  cvCreateMat(4, 1, CV_32FC1 );
        CvMoments MOMENTS;
        cvMoments(image, &MOMENTS, true);
        CV_MAT_ELEM(*(m00_star),float,0,0) = MOMENTS.m00;
        s_moment_calculator(image, m00_star, s_moment);

        s_star[0] = CV_MAT_ELEM(*(s_moment),float,0,0);
        s_star[1] = CV_MAT_ELEM(*(s_moment),float,1,0);
        s_star[2] = CV_MAT_ELEM(*(s_moment),float,2,0);
        s_star[3] = CV_MAT_ELEM(*(s_moment),float,3,0);

        //calculate s_dot
        mgrad_calculator(image, (1/z_star),0 ,0 ,m00_dot);
        mgrad_calculator(image,(1/z_star),1 ,0 , m10_dot);
        mgrad_calculator(image,(1/z_star),0 ,1 , m01_dot);

        mugrad_calculator(image, (1/z_star),1 ,1 ,mu11_dot);
        mugrad_calculator(image,(1/z_star),2 ,0 ,mu20_dot);
        mugrad_calculator(image,(1/z_star),0 ,2 ,mu02_dot);



        float m00 = MOMENTS.m00;
        float m10 = MOMENTS.m10;
        float m01 = MOMENTS.m01;



        float mu11 = MOMENTS.mu11;
        float mu20 = MOMENTS.mu20;
        float mu02 = MOMENTS.mu02;


    L[0][0] = (float) (z_star*((m00*m10_dot[0] - 1.5*m10*m00_dot[0])/(m00*m00)));
    L[0][1] = (float) z_star*((m00*m10_dot[1] - 1.5*m10*m00_dot[1])/(m00*m00));
    L[0][2] = (float)z_star*((m00*m10_dot[2] - 1.5*m10*m00_dot[2])/(m00*m00));
    L[0][3] = (float)z_star*((m00*m10_dot[3] - 1.5*m10*m00_dot[3])/(m00*m00));

    L[1][0]  = (float)z_star*((m00*m01_dot[0] - 1.5*m01*m00_dot[0])/(m00*m00));
    L[1][1]  = (float)z_star*((m00*m01_dot[1] - 1.5*m01*m00_dot[1])/(m00*m00));
    L[1][2]  = (float)z_star*((m00*m01_dot[2] - 1.5*m01*m00_dot[2])/(m00*m00));
    L[1][3]  = (float)z_star*((m00*m01_dot[3] - 1.5*m01*m00_dot[3])/(m00*m00));

    L[2][0]  = (float)0.5*z_star*((-m00_dot[0])/(m00));
    L[2][1]  = (float)0.5*z_star*((-m00_dot[1])/(m00));
    L[2][2]  = (float)0.5*z_star*((-m00_dot[2])/(m00));
    L[2][3]  = (float)0.5*z_star*((-m00_dot[3])/(m00));

    L[3][0]  = (float)(2*mu11_dot[0]*(mu20 - mu02)-2*mu11*(mu20_dot[0]-mu02_dot[0]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);
    L[3][1]  = (float)(2*mu11_dot[1]*(mu20 - mu02)-2*mu11*(mu20_dot[1]-mu02_dot[1]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);
    L[3][2]  = (float)(2*mu11_dot[2]*(mu20 - mu02)-2*mu11*(mu20_dot[2]-mu02_dot[2]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);
    L[3][3]  = (float)(2*mu11_dot[3]*(mu20 - mu02)-2*mu11*(mu20_dot[3]-mu02_dot[3]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);

    CV_MAT_ELEM(*(L_interaction),float,0,0) = (float) (z_star*((m00*m10_dot[0] - 1.5*m10*m00_dot[0])/(m00*m00)));
    CV_MAT_ELEM(*(L_interaction),float,0,1) = (float) z_star*((m00*m10_dot[1] - 1.5*m10*m00_dot[1])/(m00*m00));
    CV_MAT_ELEM(*(L_interaction),float,0,2) = (float)z_star*((m00*m10_dot[2] - 1.5*m10*m00_dot[2])/(m00*m00));
    CV_MAT_ELEM(*(L_interaction),float,0,3) = (float)z_star*((m00*m10_dot[3] - 1.5*m10*m00_dot[3])/(m00*m00));

    CV_MAT_ELEM(*(L_interaction),float,1,0) = (float)z_star*((m00*m01_dot[0] - 1.5*m01*m00_dot[0])/(m00*m00));
    CV_MAT_ELEM(*(L_interaction),float,1,1) = (float)z_star*((m00*m01_dot[1] - 1.5*m01*m00_dot[1])/(m00*m00));
    CV_MAT_ELEM(*(L_interaction),float,1,2) = (float)z_star*((m00*m01_dot[2] - 1.5*m01*m00_dot[2])/(m00*m00));
    CV_MAT_ELEM(*(L_interaction),float,1,3) = (float)z_star*((m00*m01_dot[3] - 1.5*m01*m00_dot[3])/(m00*m00));

    CV_MAT_ELEM(*(L_interaction),float,2,0) = (float)0.5*z_star*((-m00_dot[0])/(m00));
    CV_MAT_ELEM(*(L_interaction),float,2,1) = (float)0.5*z_star*((-m00_dot[1])/(m00));
    CV_MAT_ELEM(*(L_interaction),float,2,2) = (float)0.5*z_star*((-m00_dot[2])/(m00));
    CV_MAT_ELEM(*(L_interaction),float,2,3) = (float)0.5*z_star*((-m00_dot[3])/(m00));

    CV_MAT_ELEM(*(L_interaction),float,3,0) = (float)(mu11_dot[0]*(mu20 - mu02)   - mu11*(mu20_dot[0]-mu02_dot[0]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);
    CV_MAT_ELEM(*(L_interaction),float,3,1) = (float)(mu11_dot[1]*(mu20 - mu02) - mu11*(mu20_dot[1]-mu02_dot[1]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);
    CV_MAT_ELEM(*(L_interaction),float,3,2) = (float)(mu11_dot[2]*(mu20 - mu02) - mu11*(mu20_dot[2]-mu02_dot[2]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);
    CV_MAT_ELEM(*(L_interaction),float,3,3) = (float)(mu11_dot[3]*(mu20 - mu02) - mu11*(mu20_dot[3]-mu02_dot[3]))/((mu20-mu02)*(mu20-mu20)+4*mu11*mu11);

    cvInvert(L_interaction, invL_interaction, CV_LU);

    L1[0][0] = CV_MAT_ELEM(*(invL_interaction),float,0,0);
    L1[0][1] = CV_MAT_ELEM(*(invL_interaction),float,0,1);
    L1[0][2] = CV_MAT_ELEM(*(invL_interaction),float,0,2);
    L1[0][3] = CV_MAT_ELEM(*(invL_interaction),float,0,3);

    L1[1][0]  = CV_MAT_ELEM(*(invL_interaction),float,1,0);
    L1[1][1]  = CV_MAT_ELEM(*(invL_interaction),float,1,1);
    L1[1][2]  = CV_MAT_ELEM(*(invL_interaction),float,1,2);
    L1[1][3]  = CV_MAT_ELEM(*(invL_interaction),float,1,3);

    L1[2][0]  = CV_MAT_ELEM(*(invL_interaction),float,2,0);
    L1[2][1]  = CV_MAT_ELEM(*(invL_interaction),float,2,1);
    L1[2][2]  = CV_MAT_ELEM(*(invL_interaction),float,2,2);
    L1[2][3]  = CV_MAT_ELEM(*(invL_interaction),float,2,3);

    L1[3][0]  = CV_MAT_ELEM(*(invL_interaction),float,3,0);
    L1[3][1]  = CV_MAT_ELEM(*(invL_interaction),float,3,1);
    L1[3][2]  = CV_MAT_ELEM(*(invL_interaction),float,3,2);
    L1[3][3]  = CV_MAT_ELEM(*(invL_interaction),float,3,3);

}

void ControllerManager::mgrad_calculator(CvMat* image, float c, int p, int q, float m_grad[4])
{
    float mvx,mvy,mvz,mvw;
    CvMoments MOMENTS;
    cvMoments(image, &MOMENTS, true);
    if (p==0 && q==0)
    {
    mvx = 0;
    mvy = 0;
    mvz = (float)cvGetSpatialMoment( &MOMENTS, p, q);
    mvw = 0;
    }
    else if (p==0)
    {
    mvx = 0;
    mvy = (float)cvGetSpatialMoment( &MOMENTS, p, q-1);
    mvz = (float)cvGetSpatialMoment( &MOMENTS, p, q);
    mvw = (float)(- q*cvGetSpatialMoment( &MOMENTS, p+1, q-1));
    }
    else if(q == 0)
    {
    mvx = (float)cvGetSpatialMoment( &MOMENTS, p-1, q);
    mvy = 0;
    mvz = (float)cvGetSpatialMoment( &MOMENTS, p, q);
    mvw = (float)(p*cvGetSpatialMoment( &MOMENTS, p-1, q+1));

    }
    else
    {
    mvx = (float)cvGetSpatialMoment( &MOMENTS, p-1, q);
    mvy = (float)cvGetSpatialMoment( &MOMENTS, p, q-1);
    mvz = (float)cvGetSpatialMoment( &MOMENTS, p, q);
    mvw = (float)(p*cvGetSpatialMoment( &MOMENTS, p-1, q+1) - q*cvGetSpatialMoment( &MOMENTS, p+1, q-1));
    }

m_grad[0] = (float)-c*p*mvx; //mvx
m_grad[1] = (float)-c*q*mvy; //mvy
m_grad[2] = (float)c*(p+q+2)*mvz;//mvz
m_grad[3] = (float)mvw; //mvw


}

void ControllerManager::mugrad_calculator(CvMat* image, float c, int p, int q, float mu_grad[4])
{
    float muvx,muvy,muvz,muvw;
    CvMoments MOMENTS;
    cvMoments(image, &MOMENTS, true);
    if(p==0 && q==0)
    {
        muvx  = 0;//mux
        muvy  = 0;//muy
        muvz  = (float)c*(p+q+2)*cvGetCentralMoment( &MOMENTS, p, q);//muz
        muvw  = 0;//muw
    }
    else if(p==0)
    {
        muvx  = 0;//mux
        muvy  = 0;//muy
        muvz  = (float)c*(p+q+2)*cvGetCentralMoment( &MOMENTS, p, q);//muz
        muvw  = (float)( - q*cvGetCentralMoment( &MOMENTS, p+1, q-1));//muw
    }
    else if(q==0)
    {
        muvx   = 0;//mux
        muvy   = 0;//muy
        muvz   = (float)c*(p+q+2)*cvGetCentralMoment( &MOMENTS, p, q);//muz
        muvw   = (float)p*cvGetCentralMoment( &MOMENTS, p-1, q+1);//muw
    }
    else
    {
        muvx   = 0;//mux
        muvy   = 0;//muy
        muvz   = (float)c*(p+q+2)*cvGetCentralMoment( &MOMENTS, p, q);//muz
        muvw   = (float)p*cvGetCentralMoment( &MOMENTS, p-1, q+1) - q*cvGetCentralMoment( &MOMENTS, p+1, q-1);//muw
    }

        mu_grad[0] = muvx;//mux
        mu_grad[1] = muvy;//muy
        mu_grad[2] = muvz;//muz
        mu_grad[3] = muvw;//muw

}

void ControllerManager::ControlSignal_P(CvMat* vel_des)
{
    float Lambda_X = CV_MAT_ELEM(*(Lambda), float, 0, 0);
    float Lambda_Y = CV_MAT_ELEM(*(Lambda), float, 1, 0);
    float Lambda_Z = CV_MAT_ELEM(*(Lambda), float, 2, 0);
    float Lambda_Theta = CV_MAT_ELEM(*(Lambda), float, 3, 0);
    float C1[4];
    cvMatMulAdd(invL_interaction, Error_s_moment, 0, Control_Signal);

    C1[0] = CV_MAT_ELEM(*(Control_Signal), float, 0, 0);
    C1[1] = CV_MAT_ELEM(*(Control_Signal), float, 1, 0);
    C1[2] = CV_MAT_ELEM(*(Control_Signal), float, 2, 0);
    C1[3] = CV_MAT_ELEM(*(Control_Signal), float, 3, 0);


    CV_MAT_ELEM(*(vel_des), float, 0, 0) = -Lambda_X*CV_MAT_ELEM(*(Control_Signal), float, 0, 0);
    CV_MAT_ELEM(*(vel_des), float, 1, 0) = -Lambda_Y*CV_MAT_ELEM(*(Control_Signal), float, 1, 0);
    CV_MAT_ELEM(*(vel_des), float, 2, 0) = -Lambda_Z*CV_MAT_ELEM(*(Control_Signal), float, 2, 0);
    CV_MAT_ELEM(*(vel_des), float, 3, 0) = -Lambda_Theta*CV_MAT_ELEM(*(Control_Signal), float, 3, 0);
    CV_MAT_ELEM(*(vel_des), float, 4, 0) = 0.0;
}

void ControllerManager::Kinematic(float teta1,float teta2,float teta3,float teta5,float teta6,float lgantry,float *position)
{
    float l1=30,l2=25,l3=16,l5=0,lg=0,lee=(float)(7.2+18+(18-14.03+0.253+0.46));
    int i,j,k;
    float sum=0;
    //Note that lgantry is a negative value.
    float a0[4][4]={1,0,0,0,0,1,0,-lg,0,0,1,0,0,0,0,1};
    float a1[4][4]={cos(teta1),-sin(teta1),0,lg*sin(teta1),sin(teta1),cos(teta1),0,-lg*(1-cos(teta1)),0,0,1,0,0,0,0,1};
    float a2[4][4]={cos(teta2),0,sin(teta2),-l1*sin(teta2),0,1,0,0,-sin(teta2),0,cos(teta2),l1*(1-cos(teta2)),0,0,0,1};
    float a3[4][4]={cos(teta3),0,sin(teta3),-(l1+l2)*sin(teta3),0,1,0,0,-sin(teta3),0,cos(teta3),(l1+l2)*(1-cos(teta3)),0,0,0,1};
    float a5[4][4]={cos(teta5),0,sin(teta5),-(l1+l2+l3)*sin(teta5),0,1,0,0,-sin(teta5),0,cos(teta5),(l1+l2+l3)*(1-cos(teta5)),0,0,0,1};
    float a6[4][4]={cos(teta6),-sin(teta6),0,lg*sin(teta6),sin(teta6),cos(teta6),0,-lg*(1-cos(teta6)),0,0,1,0,0,0,0,1};
    float p0[4][1]={0,lg,l1+l2+l3+l5+lee,1};
    float p[4][1];
    float paux1[4][4];
    float paux2[4][4];
    //p=a0*a1*a2*a3*a5*p0+[0;lgantry;0;0];
    for(i=0;i<4;i++)
        for(j=0;j<4;j++) {
            sum=0;
            for(k=0;k<4;k++)
                sum=sum+a0[i][k]*a1[k][j];
            paux1[i][j]=sum;
    }
    for(i=0;i<4;i++)
        for(j=0;j<4;j++) {
            sum=0;
            for(k=0;k<4;k++)
                sum=sum+paux1[i][k]*a2[k][j];
            paux2[i][j]=sum;
    }
    for(i=0;i<4;i++)
        for(j=0;j<4;j++) {
            sum=0;
            for(k=0;k<4;k++)
                sum=sum+paux2[i][k]*a3[k][j];
            paux1[i][j]=sum;
    }
    for(i=0;i<4;i++)
        for(j=0;j<4;j++) {
            sum=0;
            for(k=0;k<4;k++)
                sum=sum+paux1[i][k]*a5[k][j];
            paux2[i][j]=sum;
    }
    for(i=0;i<4;i++)
        for(j=0;j<1;j++) {
            sum=0;
            for(k=0;k<4;k++)
                sum=sum+paux2[i][k]*p0[k][j];
            p[i][j]=sum;
    }
    p[1][0]=p[1][0]+lgantry;

    position[0]=p[0][0];
    position[1]=p[1][0];
    position[2]=p[2][0];
    //u0=[1;0;0];
    //v0=[0;1;0];
    //w0=[0;0;1];

    float r0[3][3]={1,0,0,0,1,0,0,0,1};
    float r1[3][3]={cos(teta1),-sin(teta1),0,sin(teta1),cos(teta1),0,0,0,1};
    float r2[3][3]={cos(teta2),0,sin(teta2),0,1,0,-sin(teta2),0,cos(teta2)};
    float r3[3][3]={cos(teta3),0,sin(teta3),0,1,0,-sin(teta3),0,cos(teta3)};
    float r5[3][3]={cos(teta5),0,sin(teta5),0,1,0,-sin(teta5),0,cos(teta5)};
    float r6[3][3]={cos(teta6),-sin(teta6),0,sin(teta6),cos(teta6),0,0,0,1};
    //u=r0*r1*r2*r3*r5*r6*u0;
    //v=r0*r1*r2*r3*r5*r6*v0;
    //w=r0*r1*r2*r3*r5*r6*w0;
    /* A=[u v w];
    %Beta give us B which is rotation around y axis. But we don't use alfa &
    %say because the robot gives us just one rotation around z so we should
    %find another solution instead of using alfa & say
    % beta=(acos(A(3,3)))*180/pi;
    % alfa=(atan2(A(2,3)/sin(beta),A(1,3)/sin(beta)))*180/pi;
    % say=(atan2(A(2,3)/sin(beta),-A(3,1)/sin(beta)))*180/pi;
    %A & B are found in this way:*/
    float ry[3][3]={0,0,0,0,0,0,0,0,0};	//=r2*r3*r5;
    float prery[3][3]={0,0,0,0,0,0,0,0,0};
    for(i=0;i<3;i++)
        for(j=0;j<3;j++) {
            sum=0;
            for(k=0;k<3;k++)
                sum=sum+r2[i][k]*r3[k][j];
            prery[i][j]=sum;
    }
    for(i=0;i<3;i++)
        for(j=0;j<3;j++) {
            sum=0;
            for(k=0;k<3;k++)
                sum=sum+prery[i][k]*r5[k][j];
            ry[i][j]=sum;
    }
    // rteta1=ry*teta1;%rteta1(3,3) is the effect of teta1 on A.
    position[3]=((ry[2][2]*teta1+teta6));//*180)/pi;  //A
    position[4]=((teta2+teta3+teta5));//*180)/pi;  //B
    position[5]=0;
}
