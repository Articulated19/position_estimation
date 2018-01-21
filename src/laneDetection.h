#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include<unistd.h>
#include<iostream>
using namespace std;
using namespace cv;

#ifndef LaneDetection_H
#define LaneDetection_H


class LaneDetection{
   public:
      VideoCapture cap;
      double error;
      double runLD(Mat frame);
      void openCap();
      void closeCap();
      void detectColor(Mat cameraFeed, Mat &result, Scalar lowerBoundColor, Scalar upperBoundColor, int pixelThresh);
      
    private:
      int font = FONT_HERSHEY_SCRIPT_SIMPLEX;
      int pixelThreshYellow = 9000;
      int pixelThreshGreen = 4000;
      void setROI(Mat img_full, Mat & res);
      void drawLanes(Mat cameraFeed, vector<Vec4i> lines, double thresholdSlope, double trapH);
      void estimateOptimalLane(vector<Vec4i> lines, double& slope, double& lineConst, bool& drawLine);
      void findSlopes(vector<Vec4i> lines, double thresholdSlope, vector<float>& s, vector<Vec4i>& tmp);
      void linearFit(double &slope, double &m, vector<int>& xValues, vector<int>& yValues);

};






#endif
