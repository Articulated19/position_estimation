#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv/cv.h>
#include "DistanceMeasurement.h"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>


using namespace cv;
using namespace std;


int main(int argc, char *argv[])
{
        //VideoWriter video("sign.avi", CV_FOURCC('M','J','P','G'),10,Size(640,480), true);
        DistanceMeasurement d;
        std_msgs::String msg;
        int counter = 0;
        ros::init(argc, argv, "distanceMeasurement");
        ros::NodeHandle n;
        ros::Publisher chatter_pub = n.advertise<std_msgs::String>("feature_detection", 1000);
        ros::Rate loop_rate(20);
	d.openCap(); // uses the movie yellow2.mp4
	//namedWindow("Window", 1);
        d.linePointSettings();
        d.focalLength = -1;
        int diff = 0;
        while (ros::ok() /*&& d.cap.isOpened()*/){
	   Mat image, work_image, cannyOut;
           bool temp = d.cap.read(image);
           if(!temp){
              cout << "cam not working\n";
	      break;
	   }
	   
           work_image = image.clone();
	   d.pTransform(work_image, work_image);
           vector<vector<Point> > contours;
           d.setContours(work_image, cannyOut, contours);
	   
	   vector<float> res;
           d.drawLines(work_image, contours);
           ++counter;
           if(d.focalLength != -1 && counter >=  10){
              counter = 0;
              std::stringstream ss;
              ss  << d.focalLength << ";" << d.truckAngle << ";" << d.globalAngle;
              msg.data = ss.str();
              ROS_INFO("%s", msg.data.c_str());
	      chatter_pub.publish(msg);
           }
           //video.write(image);
           ros::spinOnce();
           
           if (waitKey(30) == 27){
              cout << "waitkey wrong\n";
    	      exit(0); 
	  }
           d.focalLength = -1;
	   //imshow("Window", work_image);
           //waitKey(0);
        }
        d.closeCap();
        return(0);
}












