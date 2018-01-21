
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include "DistanceMeasurement.h"





int main(int argc, char *argv[]){
   
   VideoCapture cap;
   Mat cameraFeed, res;
   cap.open("/home/hawre/catkin_ws/src/position_estimation/src/output.avi");
   namedWindow("Webcam", 1);
   while(cap.isOpened()){
      cap.read(cameraFeed);
      perspective_transform(cameraFeed, 335.0 / 720 * cameraFeed.size().width, cameraFeed);
      imshow("Webcam", cameraFeed);

      switch(waitKey(1)){
         case 27:
            exit(0);
      }
   }
}
