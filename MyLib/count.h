#include "local.h"



double count_dis(Point2f a, Point2f b);	 //�����������
double count_len_wid_ratio(Point2f* p);	//������γ����                     
double count_k_wid(Point2f* p);		//������С����ı��εĿ��б��
Point2f kalman_predict(Point2f center);	//ʹ�ÿ�����Ԥ��