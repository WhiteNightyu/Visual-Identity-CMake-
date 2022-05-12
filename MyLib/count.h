#include "local.h"



double count_dis(Point2f a, Point2f b);	 //计算两点距离
double count_len_wid_ratio(Point2f* p);	//计算矩形长宽比                     
double count_k_wid(Point2f* p);		//计算最小外接四边形的宽的斜率
Point2f kalman_predict(Point2f center);	//使用卡尔曼预测