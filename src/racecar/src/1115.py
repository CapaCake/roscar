import cv2
import numpy as np
import rospy
import queue
import time
import sys, os


from std_msgs.msg import Int64
from geometry_msgs.msg import Twist

cap = cv2.VideoCapture(0)

#lower_yellow =np.array([0, 60 , 205])
#upper_yellow = np.array([179, 255 , 255])

lower_blue = np.array([100, 210 , 0])
upper_blue = np.array([179, 255, 255])

lower_red = np.array([0, 125, 0])
upper_red = np.array([10, 255, 255])

#speed
spe = 2022
angle = 75
turn = 0

rospy.init_node('racecar_teleop')#引用函数
pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)


while(1):
	ret, frame = cap.read()
	#seg
	height, width, channels = frame.shape
	#print(frame.shape)
	crop_img = frame[0:150, 130:1150]
	cropred = frame[0:150, 130:640]
	cropblue = frame[0:150, 641:1150]

	hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
	hsvblue = cv2.cvtColor(cropblue, cv2.COLOR_BGR2HSV)
	hsvred = cv2.cvtColor(cropred, cv2.COLOR_BGR2HSV)

	mask_red = cv2.inRange(hsv, lower_red, upper_red)
	mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

	mask_red1 = cv2.inRange(hsvred, lower_red, upper_red)
	mask_blue1 = cv2.inRange(hsvblue, lower_blue, upper_blue)

	m_b = cv2.moments(mask_blue, False)
	m_red = cv2.moments(mask_red, False)

	blue = cv2.countNonZero(mask_blue1)#灰度值不为零的像素数
	red = cv2.countNonZero(mask_red1)

	print("blue = ", blue)
	print("red =  ", red)
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
	cv2.imshow('RED', mask_red1)
	cv2.imshow('BLUE', mask_blue1)
	#cv2.imshow('Blue', mask_blue)
	

	error_b = cx_b - 510
	error_r = cx_r - 510
	error_x = error_b+error_r

	blu = blue-2000
	
	try:
		rb =round (red/blu, 2)
		br = round(blu/red, 2)
	except ZeroDivisionError:
		if red == 0:
			angle = 65
		if blu == 0:
			angle = 85
	print("RB = " ,rb)
	print("BR = " ,br)
	if rb>=0.2 and br >=1.1 or rb>= 0.01:
		angle = 60
	
	if  red>5000 or blue<2500:
		angle = 72
		turn = 1		#直行右转
		
		if rb<= 0 and br <= 0:
			angle = 67
			turn = 5
		
		if red >3800 and blue<1000:
			angle = 35
			turn = 3	#弯道右转
			
	if  blue>6000 or red<2500:
		angle = 80
		turn = 2		#直行左转    30-150
		if rb == 0.0 or br >200:
			angle = 75
			turn = 6

		if rb<= 0 and br<= -15.0:
			angle = 75
			turn = 6

		if blue>10000 and red <6000:
			angle = 125
			turn = 4	#弯道左转
		if red>13000 and turn==2:
			angle = 70
		if blue>6000 and red<100:
			angle = 100
	else:
		angle = error_x * 0.30 + 75
		if turn == 1:
			angle = 72
		elif turn == 2:
			angle = 80
		elif turn == 3:
			angle = 35
		elif turn == 4:
			angle = 125
		elif turn == 5:
			angle = 67
		elif turn == 6:
			angle = 75
	if    red <90 and blue>4800: #左转
		if turn == 6 or turn == 5 or turn == 3:
			pass
		elif turn == 4 or turn == 1 or turn ==2:
			turn = 90
			angle = 135
		else:
			angle = 135
			turn = 90

	print('angle =',angle)
	print('turn =', turn)
	#print(' totle =',totle)
	print('blue =',blue)
	print('red  =',red)
	#print('error_r  =',error_r)

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
