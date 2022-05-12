#include "limit.h"
#include "count.h"

extern Mat k;


void fillContours(Mat inputMat) {
	Mat dst;
	vector<Mat> temp;
	vector<vector<Point>> contours;

	split(inputMat, temp);
	dst = (temp[2] * 0.5 + 0.5 * temp[1]) * 1.3;//通道相减，排除白色以及其他非目标颜色的干扰
	dst -= temp[0];
	morphologyEx(dst, dst, MORPH_DILATE, k, Point(), 1);
	threshold(dst, dst, 150, 255, THRESH_OTSU);
	findContours(dst, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	drawContours(inputMat, contours, -1, Scalar(0, 0, 255), -1);
	//imshow("1", inputMat);
}
void limit_area(vector<vector<Point>>& contours) {
	int n = contours.size(), maxArea;
	vector<vector<Point>> temp_contours;

	maxArea = contourArea(contours[0]);
	for (int i = 1; i < n; i++)
		if (contourArea(contours[i]) > maxArea)
			maxArea = contourArea(contours[i]);

	for (int i = 0; i < n; i++)		//相对面积限制
		if (contourArea(contours[i]) > maxArea * 0.15)
			temp_contours.push_back(contours[i]);

	swap(contours, temp_contours);
}
void limit_len_wid_ratio(vector<vector<Point>>& contours, vector<Point2f>& cen_point, vector< RotatedRect>& rect_arr) {
	int n = contours.size();
	vector<vector<Point>> temp_contours;
	Point2f p[4];

	cout << "长宽比：";
	for (int i = 0; i < n; i++)		//长宽比限制
	{
		RotatedRect rect = minAreaRect(contours[i]);
		rect.points(p);
		double temp = count_len_wid_ratio(p);

		cout << temp << "   ";
		if (temp >= 1.5 && temp <= 5.3)
		{
			Mat temp;
			approxPolyDP(contours[i], temp, 12, 1);
			//cout << temp.rows << "   " << endl;
			if (temp.rows > 1 && temp.rows < 7) {	//多边形逼近限制
				//drawapp(temp, src);
				temp_contours.push_back(contours[i]);

				cen_point.push_back((Point2f)(rect.center));
				rect_arr.push_back(rect);
			}
		}
	}
	cout << endl;

	swap(contours, temp_contours);
}
void limit_distance(Mat src, vector<Point2f>& cen_point, vector< RotatedRect>& rect_arr, Point2f& last_center) {
	double distance;
	Point2f center, p[4], t1[4], t2[4], next_center;
	extern int success;

	//cout << cen_point.size() << endl;
	if (cen_point.size() == 2) {
		center.x = (cen_point[0].x + cen_point[1].x) * 0.5;
		center.y = (cen_point[0].y + cen_point[1].y) * 0.5;
		
		next_center = kalman_predict(center);

		rect_arr[0].points(t1);
		rect_arr[1].points(t2);

		double ratio = limit_dis_to_len_ratio(rect_arr[0], center);
		if (ratio < 0.7 || ratio > 1.6) {	//两个目标中心坐标，与其中一个目标的中心点，求二者之间的距离与目标长度的比值，以此来当做限制条件
			last_center = Point2f(-1, -1);
			return;
		}

		double delta_k = limit_delta_k(t1, t2);
		if (delta_k > 0.4) {		//两个可能是装甲板灯条的目标，二者的最小外接四边形的宽的斜率之差
			last_center = Point2f(-1, -1);
			return;
		}

		/*	测试用代码
		if (last_center != Point2f(-1, -1)) {
			distance = count_dis(last_center, center);
			cout <<"与上一中心点的距离："<< distance << "    " << endl;
		}*/

		last_center = center;
		Draw_Rect(src, t1, Scalar(0, 0, 255));
		Draw_Rect(src, t2, Scalar(0, 0, 255));
		cv::circle(src, center, 3, Scalar(255, 0, 0), -1);
		cv::circle(src, next_center, 3, Scalar(0, 0, 255), -1);
		success++;
	}
	else if (last_center != Point2f(-1, -1))	//程序开始时，或者上一次没能顺利检测到目标时，不再执行筛选
	{
		int m = cen_point.size();
		bool temp = 1;
		double ratio;

		for (int i = 0; i < m - 1; i++)
			for (int j = i + 1; j < m; j++)
			{
				center.x = (cen_point[i].x + cen_point[j].x) * 0.5;
				center.y = (cen_point[i].y + cen_point[j].y) * 0.5;
				distance = count_dis(last_center, center);
				cout << "与上一中心点的距离：" << distance << "    ";
				if (distance > 75)
					continue;

				ratio = limit_dis_to_len_ratio(rect_arr[i], center);
				if (ratio < 0.7 || ratio > 1.5)
					continue;

				rect_arr[0].points(t1);
				rect_arr[1].points(t2);
				double delta_k = limit_delta_k(t1, t2);
				if (delta_k > 0.3)
					continue;


				next_center.x = 2 * center.x - last_center.x;
				next_center.y = 2 * center.y - last_center.y;
				cv::circle(src, next_center, 3, Scalar(0, 255, 0), -1);
				last_center = center;
				Draw_Rect(src, t1, Scalar(0, 0, 255));
				Draw_Rect(src, t2, Scalar(0, 0, 255));
				cv::circle(src, center, 10, Scalar(255, 0, 0), -1);
				
				i = m;
				temp = 0;
				break;
			}

		cout << endl;
		if (temp)	//没有检测到目标时执行
			last_center = Point2f(-1, -1);
		else
			success++;
	}
}
double limit_dis_to_len_ratio(RotatedRect rect, Point2f center) {
	Point2f p[4];
	double ratio;
	rect.points(p);
	double len = count_dis(p[0], p[1]), wid = count_dis(p[2], p[1]), dis = count_dis(center, rect.center);
	if (wid > len)
		swap(wid, len);
	ratio = dis / len;
	cout << "距离：长度" << ratio << endl;

	return ratio;
}
double limit_delta_k(Point2f t1[4], Point2f t2[4]) {
	double k1, k2, delta_k;
	k1 = count_k_wid(t1);
	k2 = count_k_wid(t2);

	delta_k = fabs(k1 - k2);
	cout << "delta K:" << delta_k << endl;
	return delta_k;
};