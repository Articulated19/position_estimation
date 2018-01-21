#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "custom_msgs/GulliViewPositions.h"




#include <condition_variable>
#include <mutex>
#include <iostream>
#include <fstream>


using namespace std;
custom_msgs::GulliViewPositions position;
float gullitimes;

ofstream gulliview;
ofstream kalman;

bool existmessage;

void close_operation(int s);

/*
  Stores positions and the time at which they were received.
*/
void kalmancallback(const std_msgs::Float32MultiArray a) {
  if(existmessage) {
      gulliview << position.p1.x << " " << position.p1.y << endl;
      kalman << a.data[0] << " " << a.data[1] << " " << a.data[2] << " " << a.data[3] << endl;
      ROS_INFO("Stored one entry in the output logs");
  }
}

/*
  Writing the position to the same slot makes sure that we only get the most recent one, always.
*/
void gulliback(const custom_msgs::GulliViewPositions p) {
  position = p;
  gullitimes = ros::Time::now().toSec();
  existmessage = true;
}

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

      
  existmessage = false;

  /*
    Open the two textfiles we will write ackermann and gulliview to.
  */
  gulliview.open("gulliview.txt");
  kalman.open("kalman.txt");
  /*
    Launch the two nodes.
  */
  ros::init(argc, argv, "data_collect");
  ros::NodeHandle n;
  ros::Subscriber gulli_sub = n.subscribe("gv_positions",1,gulliback);
  ros::Subscriber kalman_sub = n.subscribe("kalman_topic",1,kalmancallback);


  ros::spin();
  return 0;
}

/*
  Interrupt handler.
*/
void close_operation(int s) {
    kalman.close();
    gulliview.close();
    exit(1);
}
