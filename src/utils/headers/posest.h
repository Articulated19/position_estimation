#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <unistd.h>
#include <iostream>
namespace posest {
    void setROI(cv::Point **corners, int numpoint, cv::Mat & img, cv::Mat & res);
    void setROIColorImageGray(cv::Point **corners, int numpoint, cv::Mat & img, cv::Mat & res);
    void setROIColorImageBin(cv::Point **corners, int numpoint, cv::Mat & img, cv::Mat & res);
}
