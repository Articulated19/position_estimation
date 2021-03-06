#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include "DistanceMeasurement.h"
#include <iostream>
#include <unistd.h>
using namespace cv;
using namespace std;




// perspevtive transform function
void DistanceMeasurement::pTransform(Mat frame, float yTop, Mat &result){
   Mat transformedImage;
   int width = frame.size().width;
   int height = frame.size().height;
   Point2f pointsA[4] = {Point2f(0, width), 
                         Point2f(0, yTop), 
                         Point2f(height, yTop), 
                         Point2f(height, width)};


   Point2f pointsB[4] = {Point2f(50.0 / 128 * height, width), 
                          Point2f(0 , 0), 
                          Point2f(height, 0), 
                          Point2f(78.0 / 128 * height, width)};
   transformedImage = getPerspectiveTransform(pointsA, pointsB);
   warpPerspective(frame, result, transformedImage, result.size());
}




int main(int argc, char *argv[]){
   VideoCapture cap;
   DistanceMeasurement d;
   Mat cameraFeed, res;
   cap.open("/home/hawre/catkin_ws/src/position_estimation/src/yellow1.mp4");
   namedWindow("Webcam", 1);
   while(cap.isOpened()){
      cap.read(cameraFeed);
      d.pTransform(cameraFeed, 335.0 / 720 * cameraFeed.size().width, cameraFeed);
      imshow("Webcam", cameraFeed);

      switch(waitKey(1)){
         case 27:
            exit(0);
      }
   }
}
