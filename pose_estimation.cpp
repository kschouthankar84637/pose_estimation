#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<iostream>
#include<opencv/cv.h>
#include<stdlib.h>
#include<stdio.h>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/calib3d/calib3d.hpp>
#include<math.h>
#include <tf/transform_broadcaster.h>

 #include <geometry_msgs/Pose.h>
using namespace cv;
using namespace std;

//variable declaration
Mat src,src_gray,src_helipad,finale1,helipad_temp,helipad_temp2,rot,trans,R,element;
Mat helipad,all_contours,all_ellipses,thr,detected_circles,mask_image,mask_image2;
Mat camera_matrix = (cv::Mat_<double>(3,3) << 703.829255,0,347.478533,0,704.458084,269.738627, 0, 0, 1);
Mat dist_coeffs =   (cv::Mat_<double>(5,1) << 0.049866,-0.402991,-0.007041,0.003021,0.0);
//Mat rotworld = (cv::Mat_<double>(3,3) << 1,0,0,0,-1,0,0,0,-1);
vector<Point2f> src_helipad_corners,src_helipad_corners_ex;
vector<Point3f> model_points;
float sy;
int thresh=100;
int thresh_corn=12;
int i; //increment operator in a 'for' loop
vector< vector<Point> > contours,contours1,contours2;
vector<Vec4i> hierarchy,hierarchy1,hierarchy2;
double minArea,translation_distance;
//int index;
double qualityLevel = 0.15;
double minDistance = 8;
int blockSize =27;
bool useHarrisDetector = true;
double k = 0.061;
int ext_left_index,ext_right_index,ext_top_index,ext_bottom_index;
double m;
Point2f temp_point;
char str[5];
Vec3f eulerangles;

tf::Quaternion q;
geometry_msgs::Pose pose;
geometry_msgs::Quaternion odom_quat;

//function declarations
void detect_helipad();
void detect_corners();
void filter_extreme_corners();
inline void find_min_x();
inline void find_min_y();
inline void find_max_x();
inline void find_max_y();
inline double dist(Point2f &a,Point2f &b);
void rotationMatrixToEulerAngles();
void printpose();


class msgtoimg
{
	
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
	
public:
	
  msgtoimg()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1, &msgtoimg::imageCb, this);
    pose_pub_ = nh_.advertise <geometry_msgs::Pose>("my_pose",5);
cv::namedWindow("source",CV_WINDOW_NORMAL);
	cv::namedWindow("finale1",CV_WINDOW_NORMAL);
    model_points.push_back(Point3f(0,7.3,0));
model_points.push_back(Point3f(10.86,7.3,0));
model_points.push_back(Point3f(10.86,0,0));
model_points.push_back(Point3f(0,0,0));	
  }

  ~msgtoimg()
  {
    cv::destroyWindow("source");
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	cv_ptr->image.copyTo(src);
   
    
	cvtColor(src,src_gray,CV_BGR2GRAY);
	detect_helipad(); 	
	detect_corners(); 	
	if(src_helipad_corners.size()==12)
		{
		filter_extreme_corners();		

		printpose();	
		}
pose_pub_.publish(pose);

    cv::imshow("source",src);
    cv::waitKey(30);
	
  }
};


int main(int argc, char** argv)
{
 ros::init(argc,argv,"pose_estimator"); //pose_estimator=name of node
 msgtoimg ic;
 ros::spin();
 return 0;
}


void printpose()
{
solvePnPRansac(model_points,src_helipad_corners_ex,camera_matrix,dist_coeffs,rot,trans,false,CV_EPNP);
		Rodrigues(rot,R);
		//R=R*rotworld;
		R=R.t();
		trans= -1*(R*trans);
		rotationMatrixToEulerAngles();
//translation_distance=sqrt((trans.at<double>(0,0)*trans.at<double>(0,0))+(trans.at<double>(0,1)*trans.at<double>(0,1))+(trans.at<double>(0,2)*trans.at<double>(0,2)));	

ROS_INFO(" \n\n Translation distance: [%f] [%f] [%f]",trans.at<double>(0,0),trans.at<double>(0,1),trans.at<double>(0,2));
		//cout<<"\n\ntranslation distance:"<< translation_distance;		
		cout<<"\n\nEuler Angles: "<<" Roll:  "<<eulerangles[2]*57.296<<" Pitch: "<<eulerangles[1]*57.296<<" Yaw: "<<eulerangles[0]*57.296;


q.setRPY(eulerangles[2],eulerangles[1],eulerangles[0]);
tf::quaternionTFToMsg(q,odom_quat);
pose.position.x=trans.at<double>(0,0);
pose.position.y=trans.at<double>(0,1);
pose.position.z=trans.at<double>(0,2);
pose.orientation=odom_quat;



}

void rotationMatrixToEulerAngles()
{	eulerangles[0]=0.0;eulerangles[1]=0.0;eulerangles[2]=0.0;
	sy=sqrt((R.at<double>(2,0) * R.at<double>(2,0)) +  (R.at<double>(2,2) * R.at<double>(2,2)));
	eulerangles[0] = atan2(-R.at<double>(2,0) , R.at<double>(2,2)); //about camera's z    //yaw
        eulerangles[1] = atan2(R.at<double>(2,1),sy); //about camera's y  //pitch
        eulerangles[2] = atan2(-R.at<double>(0,1), R.at<double>(1,1));//about camera's x //roll
					
					if(eulerangles[2] < 0 )
						eulerangles[2]=eulerangles[2]+3.14159;
                                        else
						eulerangles[2]=eulerangles[2]-3.14159;


					if(eulerangles[0] < 0 )
						eulerangles[0]=eulerangles[0]+3.14159;
                                        else
						eulerangles[0]=eulerangles[0]-3.14159;

					if(eulerangles[1] < 0 )
						eulerangles[1]=-1.57079-eulerangles[1];
                                        else
						eulerangles[0]=1.57059-eulerangles[1];


}



void detect_helipad()
{
threshold(src_gray,thr,thresh,255,CV_THRESH_BINARY);

all_contours=Mat::zeros(thr.size(),thr.type());
all_ellipses=Mat::zeros(thr.size(),thr.type());
helipad=Mat::zeros(thr.size(),thr.type());
detected_circles=Mat::zeros(thr.size(),thr.type());
mask_image=Mat::zeros(thr.size(),thr.type());
mask_image2=Mat::zeros(thr.size(),thr.type());


findContours(thr,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);

vector<RotatedRect> minEllipse(contours.size());

for(i=0;i<contours.size();++i)
	{
	  drawContours(all_contours,contours,i,Scalar(255,255,255),1.5,8,hierarchy);
	  if(contours[i].size()>5) {minEllipse[i]=fitEllipse(Mat(contours[i]));}
	}

for(i=0; i<contours.size();++i)
	{
	 ellipse(all_ellipses,minEllipse[i],Scalar(255,255,255),2,8);
	}



bitwise_and(all_contours,all_ellipses,detected_circles);

findContours(detected_circles,contours1,hierarchy1,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);

for(i=0;i<contours1.size();++i)
	{
		if(hierarchy1[i][0]==-1 && hierarchy1[i][1]==-1 && hierarchy1[i][2]!=-1 && hierarchy1[i][3]!=-1)
			{drawContours(mask_image,contours1,i,Scalar(255,255,255),2,8);}	
	}



findContours(mask_image,contours2,hierarchy2,CV_RETR_TREE,CV_CHAIN_APPROX_NONE);

minArea=0; 
int index=0;

for(i=0;i<contours2.size();++i)
	{
	if(minArea==0.0||contourArea(contours2[i])<minArea)
		{minArea=contourArea(contours2[i]); index=i;}	
	}

drawContours(mask_image2,contours2,index,Scalar(255,255,255),-1,8,hierarchy2);


element=getStructuringElement(MORPH_RECT,Size(7,7),Point2f(-1,-1));
erode(mask_image2,mask_image2,element);

thr.copyTo(helipad,mask_image2);

threshold(helipad,helipad,02,255,CV_THRESH_BINARY);
helipad=255-helipad;
}


void detect_corners()
{

resize(helipad,helipad_temp,Size(),3,3,INTER_LINEAR);
GaussianBlur(helipad_temp,helipad_temp2,Size(7,7),11);
if(thresh_corn < 1 ) { thresh_corn = 1; }


  src_helipad_corners.clear();
 

  /// Apply corner detection
  goodFeaturesToTrack( helipad_temp2,
               src_helipad_corners,
               thresh_corn,
               qualityLevel,
               minDistance,
               Mat(),
               blockSize,
               useHarrisDetector,
               k );


/*
helipad_temp.copyTo(finale1);
int r =7;
  for( i = 0; i < src_helipad_corners.size(); ++i )
     { circle(finale1, src_helipad_corners[i], r, Scalar(0,0,0), -1, 8, 0 );
	//sprintf(str,"%d",i);
	//putText(finale1,str,src_helipad_corners_ex[i],FONT_HERSHEY_PLAIN, 7,  Scalar(0,0,0));
     }*/
//imshow("finale1",finale1);


	
}


void filter_extreme_corners()
{
find_min_x();
find_min_y();
find_max_x();
find_max_y();

src_helipad_corners_ex.clear();

if (ext_bottom_index!=ext_left_index && ext_bottom_index!=ext_top_index && ext_bottom_index!=ext_right_index && ext_left_index!=ext_top_index && ext_left_index!=ext_right_index && ext_top_index!=ext_right_index)
{


	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_top_index].x/3,src_helipad_corners[ext_top_index].y/3)); //top most
	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_right_index].x/3,src_helipad_corners[ext_right_index].y/3)); //right most
	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_bottom_index].x/3,src_helipad_corners[ext_bottom_index].y/3)); //bottom most
	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_left_index].x/3,src_helipad_corners[ext_left_index].y/3)); //left most
}

else
{

	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_left_index].x/3,src_helipad_corners[ext_top_index].y/3));//0,0
	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_right_index].x/3,src_helipad_corners[ext_top_index].y/3));//10.85,0
	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_right_index].x/3,src_helipad_corners[ext_bottom_index].y/3));//10.85,7.74
	src_helipad_corners_ex.push_back(Point2f(src_helipad_corners[ext_left_index].x/3,src_helipad_corners[ext_bottom_index].y/3));//0,7.74

}

///////////
if (dist(src_helipad_corners_ex[0],src_helipad_corners_ex[3])>dist(src_helipad_corners_ex[1],src_helipad_corners_ex[0]))
	{
	temp_point=src_helipad_corners_ex[3];
	src_helipad_corners_ex[3]=src_helipad_corners_ex[2];
	src_helipad_corners_ex[2]=src_helipad_corners_ex[1];
	src_helipad_corners_ex[1]=src_helipad_corners_ex[0];
	src_helipad_corners_ex[0]=temp_point;
	}


finale1=Mat::zeros(src.size(),src.type());
src.copyTo(finale1);


int r =3;
  for( i = 0; i < src_helipad_corners_ex.size(); ++i )
     { circle(finale1, src_helipad_corners_ex[i], r, Scalar(255,255,255), 2, 8, 0 );
	sprintf(str,"%d",i);
	putText(finale1,str,src_helipad_corners_ex[i],FONT_HERSHEY_PLAIN, 7,  Scalar(0,0,0));
     }
circle(finale1, Point2f(320,240), r, Scalar(255,255,255), 2, 8, 0 );
imshow("finale1",finale1);
}


inline void find_min_x()
{
ext_left_index=0;
m=src_helipad_corners[0].x;
for(i=0;i<src_helipad_corners.size();++i)
	{
		if(src_helipad_corners[i].x<m)
      			{m=src_helipad_corners[i].x;
			ext_left_index=i;
			}
	}	
}

inline void find_min_y()
{
ext_top_index=0;
m=src_helipad_corners[0].y;
for(i=0;i<src_helipad_corners.size();++i)
	{
		if(src_helipad_corners[i].y<m)
      			{m=src_helipad_corners[i].y;
			ext_top_index=i;}
	}	

}

inline void find_max_x()
{
ext_right_index=0;
m=src_helipad_corners[0].x;
for(i=0;i<src_helipad_corners.size();++i)
	{
		if(src_helipad_corners[i].x>m)
      			{m=src_helipad_corners[i].x;
			ext_right_index=i;}
	}	

}

inline void find_max_y()
{
ext_bottom_index=0;
m=src_helipad_corners[0].y;
for(i=0;i<src_helipad_corners.size();++i)
	{
		if(src_helipad_corners[i].y>m)
      			{m=src_helipad_corners[i].y;
			ext_bottom_index=i;}
	}	

}


inline double dist(Point2f &a,Point2f &b)
{
	double d=(((a.x-b.x)*(a.x-b.x))+((a.y-b.y)*(a.y-b.y)));
	return d;
	
}













