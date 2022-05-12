#include "local.h"

void fillContours(Mat inputMat);	//颜色限制，然后用红色填充轮廓，提高识别成功率
void limit_area(vector<vector<Point>>& contours);	//面积限制
void limit_len_wid_ratio(vector<vector<Point>>& contours, vector<Point2f>& cen_point, vector< RotatedRect>& rect_arr);	//长宽比限制&轮廓逼近多边形限制
void limit_distance(Mat src, vector<Point2f>& cen_point, vector< RotatedRect>& rect_arr, Point2f& last_center);
double limit_dis_to_len_ratio(RotatedRect rect, Point2f center);	//可能的装甲板的中心坐标，与可能的装甲板灯条，二者之间的距离，与可能的灯条的最小外接四边形的长的比值限制
double limit_delta_k(Point2f t1[4], Point2f t2[4]);		//两个可能是装甲板灯条的目标，二者的最小外接四边形的宽的斜率之差限制