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

bool testSmallSetROI() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC1, cv::Scalar(255));
    cv::Mat res;
    
    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,14),cv::Point(10,8),cv::Point(10,8),cv::Point(20,14),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};

    /* setROI and check that there actually are pixels that are not black */
    setROI(pp, 6, original, res);
    return cv::countNonZero(res) > 0;
}

bool testSetROIDim() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC1, cv::Scalar(255));
    cv::Mat res;
    
    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,19),cv::Point(10,18),cv::Point(10,18),cv::Point(20,19),cv::Point(20,20)}};
    cv::Point * pp[1] = {roi_corners[0]};

    /* setROI and check that there actually are pixels that are not black */
    setROI(pp, 6, original, res);
    return cv::countNonZero(res) > 0 && res.rows == 2 && res.cols == 20;
}

bool testActualValues() {
    /* Create an all while image */
    cv::Mat original = cv::Mat(20, 20, CV_8UC1, cv::Scalar(255));
    cv::Mat res;
    
    /* Define region of interest to be smaller than the entire image */
    cv::Point roi_corners[][6] = {{cv::Point(0,20),cv::Point(0,10),cv::Point(10,8),cv::Point(10,8),cv::Point(20,10),cv::Point(20,20)}};
    cv::Point *pp[1] = {roi_corners[0]};   
 
    /* setROI and check that there actually are pixels that are not black */
    setROI(pp, 6, original, res);
    return (12 * 20 - cv::countNonZero(res)) == 29;
}

void testSetROITests() {
  cout << ((testSmallSetROI())    ? "testSmallSetROI  holds" : "testSmallSetROI  fails") << endl;
  cout << ((testSetROIDim())      ? "testSetROIDim    holds" : "testSetROIDim    fails") << endl;
  cout << ((testActualValues())   ? "testActualValues holds" : "testActualValues fails") << endl;
}
