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

bool testSmallROIColorBin() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,150,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageBin(pp, 6, original, res);
    return cv::countNonZero(res) == 0;
}

/* Test same as above, but pixels should end up white */
bool testSmallROIColorBinWhite() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,255,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageBin(pp, 6, original, res);
    return cv::countNonZero(res) > 0;
}

bool testSmallROIColorDimBin() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,255,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageBin(pp, 6, original, res);
    return cv::countNonZero(res) > 0 && res.rows == 12 && res.cols == 20;
}

bool testSmallROIColorDimBinBlack() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,150,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageBin(pp, 6, original, res);
    return cv::countNonZero(res) == 0 && res.rows == 12 && res.cols == 20;
}

/*
  This test does not index and check specific cells, but it does check that the number of black pixels are right. Just not that they are in the right place.
*/
bool testSmallROIColorValuesBin() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC3, cv::Scalar(255,255,0));
    cv::Mat res;

    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};
    
    /* setROI and check that there actually are pixels that are not black */
    setROIColorImageBin(pp, 6, original, res);
    return (12 * 20 - cv::countNonZero(res)) == 69;
}

void testSetROIColorImageBinTests() {
    cout << (testSmallROIColorBin()         ? "testSmallROIColorBin         holds" : "testSmallROIColorBin         fails") << endl;
    cout << (testSmallROIColorBinWhite()    ? "testSmallROIColorBinWhite    holds" : "testSmallROIColorBinWhite    fails") << endl;
    cout << (testSmallROIColorDimBin()      ? "testSmallROIColorDimBin      holds" : "testSmallROIColorDimBin      fails") << endl;
    cout << (testSmallROIColorDimBinBlack() ? "testSmallROIColorDimBinBlack holds" : "testSmallROIColorDimBinBlack fails") << endl;
    cout << (testSmallROIColorValuesBin()   ? "testSmallROIColorValuesBin   holds" : "testSmallROIColorValuesBin   fails") << endl;
}
