#include "../../laneDetection.h"
#include <stdlib.h>
#include <stdio.h>
#include<unistd.h>
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <iostream>
#include <fstream>
#include <string>
using namespace cv;
using namespace std;

#define x1 1161
#define x2 1163
#define y1 2399
#define y2 7405

void parseGulliview(string line, int *x, int *y);
void whatSideOfLine(int gulx, int guly, bool *leftside);

ifstream gulliview;
VideoCapture cap;
LaneDetection ld;


/*
  Evaluate LaneDetection. Use positions from gulliview to get the vehicles actual position.
  Check on what side of the midline the vehicle is. Compare what side the vehicle is at to
  which side lanedetection says the vehicle is at.

  The midline goes from the point (1161,2399) to (1163,7405).
*/
int main(int argc, char **argv) {
  /* Declare locals */
  string line;
  Mat frame;
  double laneerror;
  int nrframes = 0;
  int faultyframes; // Number of frames where lanedetection gave a false value.
  int gulx,guly;
  double x,slope,intercept;
  bool leftside; // true = left and false if it is on or to the right of the line.

  /* Open streams */
  gulliview.open("gulliview.txt");
  cap.open("output.avi");

  /* Meat of the test */
  while(getline(gulliview,line)) {
    if(cap.isOpened()) {
      /* Read a frame and process it to find out what side lane detection thinks we're on. */
      cap.read(frame);
      laneerror = ld.runLD(frame);

      /* Parse the gulliviewposition and check what side the position is of the midline. */
      parseGulliview(line, &gulx, &guly);
      whatSideOfLine(gulx, guly, &leftside);

      if((laneerror != 9999) && leftside && (laneerror < 0)) {
        nrframes++;
      } else {
        nrframes++;
        faultyframes++;
      }
    } else {
      cout << "Error: cap not opened" << endl;
      return 0;
    }
    nrframes++;
  }

  cout << "The evaluated algorithm for detecting lanes gave a false reading from " << faultyframes << " frames out of " << nrframes << " frames. The ratio of good processings are (nrframes - faultyframes)/nrframes = " << (((double)nrframes - (double)faultyframes)/(double)nrframes) << "." << endl;
  return 1;
}

/*
  Parses a line of text representing a gulliview message into two ints, x and y. These are then put in the supplied integer pointers.
*/
void parseGulliview(string line, int *x, int *y) {
  int gx, gy;
  int index;

  index = line.find(' ');
  gx = stoi(line.substr(0,index));
  gy = stoi(line.substr(index+1,line.length()-index));
  *x = gx; *y = gy;
}

/*
  Checks if a given point is to the left or right of the given line.
*/
void whatSideOfLine(int gulx, int guly, bool *leftside) {
  double reference = (x1-1-x1)*(y2-y1)-(y1-y1)*(x2-x1);
  double pointsign = (gulx-x1)*(y2-y1)-(guly-y1)*(x1-x2);
  
  *leftside = ((reference < 0) ? ( pointsign < 0) : ((reference > 0) ? (pointsign > 0) : (pointsign == 0)));
}
