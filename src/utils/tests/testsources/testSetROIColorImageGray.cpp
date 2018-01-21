#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <unistd.h>
#include <iostream>
#include "../../headers/posest.h"
#include "../utils/assert.h"
using namespace std;
using namespace posest;

bool testSmallROIColor() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,150,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageGray(pp, 6, original, res);
    return cv::countNonZero(res) > 0;
}

bool testSmallROIColorDim() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,150,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageGray(pp, 6, original, res);
    return cv::countNonZero(res) > 0 && res.rows == 12 && res.cols == 20;
}

/*
  This test does not index and check specific cells, but it does check that the number of black pixels are right. Just not that they are in the right place.
*/
bool testSmallROIColorValues() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,150,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageGray(pp, 6, original, res);
    return (12 * 20 - cv::countNonZero(res)) == 69;
}

void testSetROIColorImageTests() {
    cout << (testSmallROIColor()       ? "testSmallROIColor       holds" : "testSmallROIColor       fails") << endl;
    cout << (testSmallROIColorDim()    ? "testSmallROIColorDim    holds" : "testSmallROIColorDim    fails") << endl;
    cout << (testSmallROIColorValues() ? "testSmallROIColorValues holds" : "testSmallROIColorValues fails") << endl;
}
