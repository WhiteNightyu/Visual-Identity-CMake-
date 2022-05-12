#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<cmath>

using namespace std;
using namespace cv; 

void Draw_Rect(Mat& img, Point2f* p, Scalar scalar);	//画出矩形
void drawapp(Mat app, Mat img);		//画出多边形逼近的图形
