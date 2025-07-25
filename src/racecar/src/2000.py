import cv2
import numpy as np
import rospy
import queue
import time
import sys, os


from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

cap = cv2.VideoCapture(0)

lower_yellow =np.array([0, 60 , 205])
upper_yellow = np.array([179, 255 , 255])

lower_blue = np.array([100, 210 , 0])
upper_blue = np.array([179, 255, 255])

lower_red = np.array([0, 125, 0])
upper_red = np.array([10, 255, 255])

#speed
spe = 2000
angle = 75
#stt = 0
turn = 0
totle = 0
#t1 = queue.Queue()

rospy.init_node('racecar_teleop')#引用函数
pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)
#pub1 = rospy.Publisher('~/red_blue', Int64, queue_size=5)


while(1):
	ret, frame = cap.read()
	#seg
	height, width, channels = frame.shape
	#print(frame.shape)
	crop_img = frame[0:150, 130:1150]
	#yellow_img = frame[100:160, 400:1000]
	
	hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
	#hsv_yellow = cv2.cvtColor(yellow_img, cv2.COLOR_BGR2HSV)
	
	mask_red = cv2.inRange(hsv, lower_red, upper_red)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
	
	#mask_yellow = cv2.inRange(hsv_yellow, lower_yellow, upper_yellow)
	#cv2.imshow("mask_yellow",mask_yellow)
	
	m_b = cv2.moments(mask_blue, False)
	m_red = cv2.moments(mask_red, False)

	blue = cv2.countNonZero(mask_blue)#灰度值不为零的像素数
	red = cv2.countNonZero(mask_red)
	#yellow = cv2.countNonZero(mask_yellow)
	#print('yellow= ',yellow)
	try:
		cx_b, cy_b = m_b['m10']/m_b['m00'], m_b['m01']/m_b['m00']
		cx_r, cy_r = m_red['m10']/m_red['m00'], m_red['m01']/m_red['m00']
	except ZeroDivisionError:
		cx_b, cy_b = height/2, width/2
		cx_r, cy_r = height/2, width/2
		cx = 0
	
	cx_b = int(cx_b)
	cy_b = int(cy_b)
	cx_r = int(cx_r)
	cy_r = int(cy_r)
	# 绘制中心点
	cv2.circle(crop_img, (cx_b, cy_b), 7, 128, -1)
	cv2.circle(crop_img, (cx_r, cy_r), 7, 128, -1)
	#创将一个msg对象
	#msg = Int64()
	
	#cv2.imshow('capture', crop_img)
	cv2.imshow('RED', mask_red)
	cv2.imshow('Blue', mask_blue)
	
	#找到质心或者轮廓中心
	#height, width, channels = frame.shape
	error_b = cx_b - 510
	error_r = cx_r - 510
	error_x = error_b+error_r

	#if t1.empty() and stt == 0:
		#stt = 1
	#control
	#直行
	#totle = yellow + totle
	if  red>4500 or blue<2500:
		angle = 70
		turn = 1	#直行右转
		if red >6500 and blue<2700:#1500
			angle = 35
			turn = 3#弯道右转

	if  blue>5000 or red<2500:# 6500  2500
		angle = 80
		turn = 2	#直行左转
		
		if blue>13000 and red <6500:#13000     6500
			angle = 115
			turn = 4#弯道左转
		if blue>25000:
			angle = 145
		if blue>6500 and red<100:
			angle = 100
	else:
		angle = error_x * 0.30 + 75
		if turn == 1:
			angle = 68
		elif turn == 2:
			angle = 80
		elif turn == 3:
			angle = 35
		elif turn == 4:
			angle = 125
			
	if red < 50 and blue> 4700:
		angle = 130
		
	print('angle =',angle)
	print('turn =', turn)
	#print(' totle =',totle)
	print('blue =',blue)
	print('red  =',red)

	twist = Twist()
	twist.linear.x = spe; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angle
	pub.publish(twist)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		spe = 1600
		twist = Twist()
		twist.linear.x = spe;
		pub.publish(twist)
		print(spe)
		time.sleep(0.5)
		break

cap.release()
cv2.destroyAllWindows()
sys.exit()

