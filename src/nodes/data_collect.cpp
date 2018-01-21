#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDrive.h"
#include "custom_msgs/GulliViewPositions.h"
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <condition_variable>
#include <mutex>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
custom_msgs::GulliViewPositions position;
std_msgs::Float32MultiArray kalmanData;
float gullitimes;

VideoCapture cap;
VideoWriter writer;
ofstream ackerman;
ofstream gulliview;
//ofstream kalman;

bool existmessage; //, existKalman;

void close_operation(int s);

/*
  Stores positions and the time at which they were received.
*/
void callback(const ackermann_msgs::AckermannDrive a) {
  if(cap.isOpened() && existmessage){ // && existKalman) {
    Mat frame;
    if(cap.read(frame)) {
      ackerman << a.steering_angle << " " << a.speed << endl;
      gulliview << position.p1.x << " " << position.p1.y << endl;
      writer.write(frame);
      existmessage = false;
      //ROS_INFO("Stored one entry in the output logs");
    } else {
      ROS_INFO("Error reading frame");
    }
  } /*else {
    cout << cap.isOpened() << endl;;
  }*/
}

/*
  Writing the position to the same slot makes sure that we only get the most recent one, always.
*/
void gulliback(const custom_msgs::GulliViewPositions p) {
  position = p;
  gullitimes = ros::Time::now().toSec();
  existmessage = true;
}


//void kalmanback(const std_msgs::Float32MultiArray a){
//  kalmanData = a;
//  existKalman = true;

//}

int main(int argc, char **argv)
{
  /*
    Set up interrupt handler, this will close all streams when executed.
  */
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = close_operation;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  /*
    Set up the camera and the image writer and initialize them.
  */
  cap.open(0);
  int codec = CV_FOURCC('X','V','I','D');
  int fps = 20;
  writer.open("output.avi",codec,fps, Size(640,480), true);
  existmessage = false;

  /*
    Open the two textfiles we will write ackermann and gulliview to.
  */
  ackerman.open("ackerman.txt");
  gulliview.open("gulliview.txt");
  //kalman.open("kalman.txt");
  /*
    Launch the two nodes.
  */
  ros::init(argc, argv, "data_collect");
  ros::NodeHandle n;
  ros::Subscriber data_sub = n.subscribe("master_drive",1000,callback);
  ros::Subscriber gulli_sub = n.subscribe("gv_positions",1,gulliback);
  //ros::Subscriber kalman_sub = n.subscribe("kalman_topic",1,kalmanback);

  ros::spin();
  return 0;
}

/*
  Interrupt handler.
*/
void close_operation(int s) {
    ackerman.close();
    gulliview.close();
    cap.release();
    writer.release();
    exit(1);
}
