
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv/cv.h"
#include <unistd.h>
#include <iostream>
//#include <ctime>
using namespace std;
using namespace cv;

//#include <chrono>


float length_points(Point a , Point b);
bool triangle_ok(vector<Point> points, int edges);
bool size_trianlge(Point a, Point b, Point c);


// tha maximum differance of the sides in the triangle for it to be a correct triangle. 
#define triangle_size_difference 20

// all the sides in the triangle must be larger than this value for it to be a triangle. 
#define triangle_length 80

#define x_start 800
#define y_start 100

#define w_roi 450
#define h_roi 250



        //g++ -o main main.cpp `pkg-config opencv --cflags --libs`  -std=gnu++11



int main(int argc, char *argv[])
{

	VideoCapture cap;
         cap.open("/home/hawre/catkin_ws/src/position_estimation/src/yellow1.mp4"); // uses the movie yellow.mp4
	
	namedWindow("Window", WINDOW_AUTOSIZE);

	// the width and height of the image.
        //cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
        //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 320); 
	double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);



	// the colors for the found lines and triangles. 
	Scalar color(0,255,0);
	Scalar color_2(255,0,0);

	// we want to find triangles. therfore three edges. 
	int edges = 3;
	
	
	// Point to move the visual lines. 
	Point move(x_start,y_start);

	// center down point of the image. 
	Point start_point(dWidth/2, dHeight);


	while (1)
	{

		

	   	Mat image, clonedImage, canny_out;

		bool temp = cap.read(image);

		if(!temp)
		{
			break;
		}


               
		
		clonedImage = image.clone();
                Point xs(x_start, y_start);

		Rect RectToSelect(x_start,y_start,w_roi,h_roi);

		if (0 <= RectToSelect.x && 0 <= RectToSelect.width
                    && RectToSelect.x + RectToSelect.width <= clonedImage.cols
                    && 0 <= RectToSelect.y
                    && 0 <= RectToSelect.height
                    && RectToSelect.y + RectToSelect.height <= clonedImage.rows){
                    // box within the image plane
                    
		}else{
                  cout << "Rect x: " << RectToSelect.x    
                       << " \nRect y: " << RectToSelect.y
                       << " \nRect width: " << RectToSelect.width 
                       << " \nRect height: " << RectToSelect.height 
                       << " \nimage clos: " << clonedImage.cols 
                       << " \nimage rows: " << clonedImage.rows << "\n";
                  

                  exit(0);
                  // box out of image plane, do something...
                 }
		clonedImage = clonedImage(RectToSelect);


                cvtColor(clonedImage,clonedImage,COLOR_BGR2GRAY);
                
	   	Canny(clonedImage,canny_out,100,200,3);
	   	vector<vector<Point> > contours;
	   	vector<Vec4i> hierarchy;

	   	vector<vector<Point> > shape;

		findContours( canny_out, contours, hierarchy,
				CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
		int i;
		for(i = 0; i < contours.size(); i++)
		{
			vector<Point> result;

			approxPolyDP(contours[i], result, 10, true);

			if(triangle_ok(result, 3))
			{

				// here we have found a sign. 

				// this is just for drawing. 
				for(int j = 0; j < edges; j++)
   		 		{
   		 			if(j+1 <  edges)
   		 			{
   		 			
   		 				line(image,result[j] + move, result[j+1] + move, color, 5,8,0);
						

   		 			}
   		 			else
   		 			{
					 	line(image,result[j] + move, result[0]+ move, color, 5,8,0);
   		 			}

					line(image,result[j] + move , start_point, color_2, 2,8,0);	
   				
   		 		}	
			}

		}
	   	


		switch(waitKey(1)) 
		{
    			case  27:
                          exit(0);
		}
		
		
		imshow("Window", image);


	}

	cap.release();


	return(0);
}


// checks if the polygon is a tringale and also if the triangle has the right dimensions 
bool triangle_ok(vector<Point> points, int edges)
{
	if(points.size() == edges && size_trianlge(points[0], points[1], points[2]) )
	{
		return true;
	}
	return false;
} 



// returns the length between two points in 
float length_points(Point a , Point b)
{

	return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));

}



// returns true if the difference of the three sides is smaller than triangle_length
bool size_trianlge(Point a, Point b, Point c)
{
	int length_1 = length_points(a,b);
	int length_2 = length_points(b,c);
	int length_3 = length_points(c,a);

	if((abs(length_1 - length_2) < triangle_size_difference) && 
		(abs(length_2 - length_3) < triangle_size_difference) && 
		(abs(length_3 - length_1) < triangle_size_difference))
	{
		if((length_1 >= triangle_length) && (length_2 >= triangle_length) && (length_3 >= triangle_length))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	return false;
}


