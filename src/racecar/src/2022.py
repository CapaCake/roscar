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
spe = 2008
angle = 75
#stt = 0
turn = 0
totle = 0
br = 0
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
	#cv2.imshow('RED', mask_red)
	cv2.imshow('Blue', mask_blue)
	cv2.imshow('red', mask_red)
	
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
	blu = blue-2000
	
	try:
		rb =round (red/blu,2)
		br = round(blu/red,2)	
	except ZeroDivisionError:
		if red ==0:
			angle = 65
		if blu ==0:
			angle = 85
	print("RB = ", rb)
	print("BR = ", br)
	#90度出弯调整，准备入s弯
	if rb>=0.2 and br >=1.1 or rb>= 0.01:
		angle = 60
	else:
		pass
	#右转的一些些判断条件
	if  red>3500 or blue<4000:
		angle = 69
		turn = 1		#直道上右转
		#s弯中段靠近蓝色行驶
		if rb<= 0 and br <= 0:
			angle = 55
			turn = 5
		if red > 18000 and br>= 0:
			angle= 30
			turn=8
			print("888888")
		#s弯后段出弯
		if red >3500 and blue<1000:
			angle = 30
			turn = 3	#弯道中右转
	#左转的一些些判断条件
	if  blue>6000 or red<2500:
		angle = 78
		turn = 2		#直道上左转
		#s弯出弯进90度弯调整
		if rb == 0.0 or br >=200:
			angle = 75
			turn = 6
			#print("6666666")
		#第3个s弯出弯调整,避免撞红色
		if rb<= 0 and br<= -15.0:
			angle = 70
			turn = 6
		#s弯左转出弯（入弯）
		if blue>10000 and red <3500:
			angle = 125
			turn = 4	#弯道上左转
		#if blue > 10000:
			#angle = 90
			#turn = 7
		#S弯避免撞红色，提前向右调整
		if red>11000 and turn==2:
			angle = 85
		#S弯避免撞蓝色，向左提前出弯
		if blue>6000 and red<100:
			angle = 100
			turn = 7
	else:
		angle = error_x * 0.30 + 75
		if turn == 1:	#直道上右转
			angle = 72
		elif turn == 2:	#直道上左转
			angle = 78
		elif turn == 3:	#S弯道中右转
			angle = 30
		elif turn == 4:	#S弯道中左转
			angle = 125
		elif turn == 5:	#S弯中段靠蓝行驶
			angle = 55
		elif turn == 6:	#出s弯调整
			angle = 72
		elif turn == 7:	#出s弯左转
			angle = 90
		elif turn == 8:
			angle = 55
	#90度左转
	if	red < 100 and blue > 2600:#2600
		if turn == 6 or turn == 5 or turn == 3:
			pass
		elif turn == 1 or turn == 2 or turn == 4:
			turn = 90
			spe = 2020
			angle = 140
		else:
			turn = 90
			spe = 2020
			angle = 150
	#停车
	#if  totle >50000:
		#spe = 1500
		#spe = 1600

	print('angle =',angle)
	print('turn =', turn)
	#print(' totle =',totle)
	print('blue =',blue)
	print('red  =',red)
	#print('error_r  =',error_r)
	print("spe = ", spe)


	if cv2.waitKey(1) & 0xFF == ord('q'):
		spe = 1500
		#spe = 1600
		angle = 75
		print(spe)
		print(angle)
		time.sleep(0.5)

		
	twist = Twist()
	twist.linear.x = spe; twist.linear.y = 0; twist.linear.z = 0
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angle
	pub.publish(twist)
	twist = Twist()
	twist.linear.x = spe;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = angle
	pub.publish(twist)
	
	if spe == 1500:
		break

cap.release()
cv2.destroyAllWindows()
sys.exit()

