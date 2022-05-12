#include "local.h"
#include "limit.h"
#include "count.h"

int all=0,success=0;
Mat k = getStructuringElement(1, Size(5, 5));
int main()
{
	VideoCapture video(0);
	video.set(CAP_PROP_EXPOSURE, 0.0008);
	video.set(CAP_PROP_FRAME_HEIGHT, 288);
	video.set(CAP_PROP_FRAME_WIDTH, 352);

	Point2f p[4], center, last_center=Point(-1,-1);
	Mat img, src, dst;
	vector<Mat> temp;
	vector<Point2f> cen_point;
	vector<vector<Point>> contours, temp_contours;
	vector< RotatedRect> rect_arr;

	while (cv::waitKey(1)!=27)
	{
     all++;
		video >> img;
		int64 t = getTickCount();
		src = img.clone();
		
		fillContours(img);

		split(img, temp);
		dst = temp[2] * 1.5;
		dst -= temp[0] + temp[1];
		//morphologyEx(dst, dst, MORPH_DILATE, k, Point(), 1);
		threshold(dst, dst, 150, 255, THRESH_OTSU);
		findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
		
		int n = contours.size();
		if (n < 2)
			goto label;

		limit_area(contours);
		n = contours.size();
		if (n < 2)
			goto label;

		limit_len_wid_ratio(contours, cen_point, rect_arr);
		n = contours.size();

		if (cen_point.size() < 1)
			goto label;

		//距离限制，在距离限制函数内运行最后两重限制的函数
		limit_distance(src, cen_point, rect_arr, last_center);
		
		
	label:
		rect_arr.clear();
		contours.clear();
		temp_contours.clear();
		cen_point.clear();
		temp.clear();

		cv::imshow("dst", src);

		double time = (double)(getTickCount() - t) / getTickFrequency();
		cout << "time" << time << "sec" << endl;
		cout << "FPS:"<< 1/time<<endl;
	   cout<<"success rate:"<<(double)success/all<<endl;
	   if (all == 30)
	   {
		   all = 0;
		   success = 0;
	   }
		cout << "------------------------" << endl;
		
	}
	
}

void Draw_Rect(Mat& img, Point2f* p, Scalar scalar)
{
	line(img, p[0], p[1], scalar, 2);
	line(img, p[1], p[2], scalar, 2);
	line(img, p[2], p[3], scalar, 2);
	line(img, p[3], p[0], scalar, 2);
}
void drawapp(Mat app, Mat img) {
	for (int i = 0; i < app.rows; i++) {
		if (i == app.rows - 1)
		{
			Vec2i point1 = app.at<Vec2i>(i);
			Vec2i point2 = app.at<Vec2i>(0);
			line(img, point1, point2, Scalar(255, 0, 0), 2, 8, 0);
			break;
		}
		Vec2i point1 = app.at<Vec2i>(i);
		Vec2i point2 = app.at<Vec2i>(i + 1);
		line(img, point1, point2, Scalar(255, 0, 0), 2, 8, 0);
	}
}

