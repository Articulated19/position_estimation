#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <unistd.h>
#include <iostream>
namespace assert {
    bool assertDoubleEquals(double one, double two);
    bool assertMatEquals(cv::Mat *one, cv::Mat *two);
}
