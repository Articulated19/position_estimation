#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <unistd.h>
#include <iostream>
namespace assert {
    /*
      given two doubles, returns diff < 0.000001
    */
    bool assertDoubleEquals(double one, double two) {
        double diff = ((one - two) < 0) ? -(one - two) : (one - two); /* simple abs(one-two) */
        return (diff < 0.000001);
    }

    /*
      Given pointers to two cv::Mat, returns true if they are equal.
    */
    bool assertMatEquals(cv::Mat *one, cv::Mat *two) {
        if(one->rows == two->rows && one->cols == two->cols) {
          cv::Mat temp;
          cv::bitwise_xor(*one, *two, temp);
          return !(cv::countNonZero(temp));
        }
        return false;
    }
}
