#include "local.h"

void fillContours(Mat inputMat);	//��ɫ���ƣ�Ȼ���ú�ɫ������������ʶ��ɹ���
void limit_area(vector<vector<Point>>& contours);	//�������
void limit_len_wid_ratio(vector<vector<Point>>& contours, vector<Point2f>& cen_point, vector< RotatedRect>& rect_arr);	//���������&�����ƽ����������
void limit_distance(Mat src, vector<Point2f>& cen_point, vector< RotatedRect>& rect_arr, Point2f& last_center);
double limit_dis_to_len_ratio(RotatedRect rect, Point2f center);	//���ܵ�װ�װ���������꣬����ܵ�װ�װ����������֮��ľ��룬����ܵĵ�������С����ı��εĳ��ı�ֵ����
double limit_delta_k(Point2f t1[4], Point2f t2[4]);		//����������װ�װ������Ŀ�꣬���ߵ���С����ı��εĿ��б��֮������