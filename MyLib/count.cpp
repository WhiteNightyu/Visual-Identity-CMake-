#include "limit.h"
#include "count.h"

extern Mat k;
const int stateNum = 4;         //״ֵ̬4��1����(x,y,��x,��y)  
const int measureNum = 2;    //����ֵ2��1����(x,y)    
KalmanFilter KF(stateNum, measureNum, 0);       //���忨�����˲���

double count_dis(Point2f a, Point2f b) {
	double temp = (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
	temp = sqrt(temp);
	return temp;
}
double count_len_wid_ratio(Point2f* p) {
	double len = count_dis(p[0], p[1]), wid = count_dis(p[2], p[1]);
	if (wid > len)
		swap(wid, len);
	return len / wid;
}

double count_k_wid(Point2f* p) {
	double len = count_dis(p[0], p[1]), wid = count_dis(p[2], p[1]);
	Point2f temp[2] = { p[2],p[1] };
	double k;
	if (wid > len)
		temp[0] = p[0];

	k = (temp[1].y - temp[0].y) / (temp[1].x - temp[0].x);
	return fabs(k);
}

Point2f kalman_predict(Point2f center) {
	KF.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);        //ת�ƾ���A  
	setIdentity(KF.measurementMatrix);                                                                  //��������H  
	setIdentity(KF.processNoiseCov, Scalar::all(1));                                                    //ϵͳ�����������Q  
	setIdentity(KF.measurementNoiseCov, Scalar::all(1e-60));                                            //���������������R  
	setIdentity(KF.errorCovPost, Scalar::all(10000));                                                   //����������Э�������P    
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);

	measurement.at<float>(0) = center.x;
	measurement.at<float>(1) = center.y;

	KF.correct(measurement);                                                                 //update  
	Mat prediction = KF.predict();
	Point2f predict_pt = Point(prediction.at<float>(0), prediction.at<float>(1));   //Ԥ��ֵ(x',y')
	return predict_pt;
}