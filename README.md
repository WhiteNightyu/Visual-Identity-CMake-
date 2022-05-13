# SCUT华南理工大学2022无限机甲杯视觉识别程序开源

本程序可实现对于固定颜色的装甲板的识别与预测，使用c++语言编写，加入了OpenCV库，任何c++编译器均可编译
限制条件函数都封装在limit.cpp中，计算与kalman预测函数都在count.cpp中

# 程序原理：
         先使用降低摄像头曝光的方式，排除不发光物体
          然后使用通道分离与相减的方式，只保留指定颜色，排除白光与其他非指定颜色，修改通道相减的参数，可以改变筛选的颜色
          然后对于上述分离出来的结果进行轮廓识别，然后用红色填充轮廓（因曝光原因，灯条中心可能为白色，在之前的步骤中被排除）
          再使用通道分离与相减的方式，分离出红色，识别轮廓
          然后使用
          1、相对面积限制
          2、轮廓的最小外接四边形的长宽比限制
          3、轮廓的逼近多边形的边数限制
          4、可能的装甲板中心点与上一次识别结果的距离的限制
          5、其中一个可能的装甲板灯条的最小外接四边形的长，与“根据两个可能的装甲板灯条计算出来的可能的装甲板中心坐标，与其中一个可能的装甲板灯条的中心点，两点之间的距离”，二者比值的限制
          6、两个可能的装甲板灯条的最小外接四边形的宽的斜率之差的限制
          筛选出两个真正的装甲板灯条与装甲板中心坐标，并根据kalman滤波预测装甲板中心坐标的移动


# 程序效果：(均为使用组委会借出的linux开发板下的效果，因ssh连接传输的帧率过低，因此摄像头识别的演示视频在windows下运行来拍摄)
	成功率：对于装甲板平面与摄像头镜头平面的夹角不大于55°，且非快速移动的情况下（对于之前发的装甲板演示视频的移动速度绰绰有余），识别成功率可近似认为100%，只有极少数极端情况才会出现识别不出

		对于快速移动的情况下，识别成功率＞98%

		对于装甲板平面与摄像头镜头平面的夹角＞55°的情况，本程序因为限制的参数原因，会无法识别装甲板，但可以考虑到对于装甲板平面与摄像头镜头平面的夹角的理论最大角度，并根据此设置各个限制的参数

	运行帧率：对于组委会借出的linux开发板，本程序使用352×288分辨率，运行帧率位于25-35fps内，大多数情况下为28fps以上


# 过程中遇到的问题以及解决方法：
         对于之前发的装甲板演示视频，因曝光不够低，装甲板灯条中间部分被当做白色，在通道相减时被排除了。而解决方法即为程序原理中的“对于上述分离出来的结果进行轮廓识别，然后用红色填充轮廓，再使用通道分离与相减的方式，分离出红色，识别轮廓”
					
	在算法研发“轮廓的逼近多边形的边数限制”过程中，因曝光问题装甲板灯条周围存在光晕，所得到的逼近多边形的边数的方差较大，根据此做出的限制参数的效果较差。解决方法：对一开始通道相减后的图像进行膨胀操作，膨胀后的灯条部分更加接近四边形，逼近多边形的边数的方差大大减小


# 程序不足：
        在测试过程中，用手机显示装甲板，当有暖色调的强光（如台灯等）照射在镜面的手机屏幕上时，会对识别效果作出影响，不过在非快速移动的情况下，成功率大于98%
        当装甲板灯条与强光源重叠时，成功率会降低，但仍在85%以上，多数情况下在90%以上
