#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include "opencv/cv.h"
#include <unistd.h> 
#include <iostream>

using namespace cv;
using namespace std;


#define lowerColor Scalar(40,60,0)
#define upperColor Scalar(75,255,255)

int font = FONT_HERSHEY_SCRIPT_SIMPLEX;
int pixel_thresh = 9000;



void detectColor(Mat cameraFeed, Mat &result, Scalar lowerBoundColor, Scalar upperBoundColor){
      Mat res, hsvImage;
      cvtColor(cameraFeed, hsvImage, COLOR_BGR2HSV);
      Mat mask = Mat::zeros(cameraFeed.size(), cameraFeed.type());


      inRange(hsvImage, lowerBoundColor, upperBoundColor, mask);
      bitwise_and(cameraFeed, cameraFeed, res, mask);
      int nonZeroCount = countNonZero(mask);
    
      putText(cameraFeed, "Yellow pixel count: ", Point(10, 50), font, 1, Scalar(0,0,255), 8);
      putText(cameraFeed,  "Pixel " + to_string(nonZeroCount), Point(20,80), font, 1, Scalar(255,0,0), 1);


      if(nonZeroCount > pixel_thresh){
         putText(cameraFeed,"Truck at turn! ", Point(100,200), font, 2, Scalar(0,0,255), 8);
      }

      result = cameraFeed;

}


int main(int argc, char *argv[]){
   VideoCapture cap;
   Mat cameraFeed, res;
   cap.open(0);
   namedWindow("Webcam", 1);
   while(cap.isOpened()){
      cap.read(cameraFeed);
      detectColor(cameraFeed, res, Scalar(10,120,120), Scalar(40,255,255));
      /*cvtColor(cameraFeed, hsvImage, COLOR_BGR2HSV);
      Mat mask = Mat::zeros(cameraFeed.size(), cameraFeed.type());
      Mat res;

      inRange(hsvImage, Scalar(20,120,120), Scalar(40,255,255), mask);
      bitwise_and(cameraFeed, cameraFeed, res, mask);
      int nonZeroCount = countNonZero(mask);
    
      putText(cameraFeed, "Yellow pixel count: ", Point(10, 50), font, 1, Scalar(0,0,255), 8);
      putText(cameraFeed,  "Pixel " + to_string(nonZeroCount), Point(20,80), font, 1, Scalar(255,0,0), 1);


      if(nonZeroCount > pixel_thresh){
         putText(cameraFeed,"Truck at turn! ", Point(100,200), font, 2, Scalar(0,0,255), 8);
      }*/
      //imshow("hsv_img", hsvImage);
      //imshow("mask", mask);
      //imshow("Filtered color only", res);
      imshow("Webcam", res);

      switch(waitKey(1)){
         case 27:
            exit(0);
      }
   }
}





