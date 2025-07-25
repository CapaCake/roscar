import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
cap=cv2.VideoCapture(0)

lower_blue=np.array([20,173,48])
upper_blue=np.array([145,255,255])
lower_red=np.array([0,185,0])
upper_red=np.array([71,255,255]) 
#rospy.init_node('racecar_teleop')

spe = 1685
rospy.init_node('racecar_teleop')
pub = rospy.Publisher('~/car/cmd_vel', Twist, queue_size=5)
while(1):

	ret,frame=cap.read()
	
#seg
	height, width, channels = frame.shape
	descentre = 50
	rows_to_watch = 100
	crop_img = frame[(height)//4 + descentre:(height)//4 + (descentre+rows_to_watch)][1:width]
	#cv2.imshow('capture',crop_img)	
	
	hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)
	#caicai=cv2.inRange(hsv,lower,upper)
	#cv2.imshow('capture',caicai)
	mask_red=cv2.inRange(hsv,lower_red,upper_red)
	mask = cv2.inRange(hsv, lower_blue, upper_blue)
	res = cv2.bitwise_and(crop_img, crop_img, mask=mask)
	height, width, channels = frame.shape
	m_b= cv2.moments(mask,False)
	m_red=cv2.moments(mask_red,False)
	#caigou=cv2.moments(caicai,False)
	blue=cv2.countNonZero(mask)
	red=cv2.countNonZero(mask_red)
	#cv2.imshow('mask_red',mask_red)
	#cv2.imshow('res',res)
	#cv2.imshow('mask_blue',mask)
	try:
		cx_b, cy_b = m_b['m10']/m_b['m00'], m_b['m01']/m_b['m00']
		cx_r,cy_r =m_red['m10']/m_red['m00'], m_red['m01']/m_red['m00']
		#cx_c,cy_c =caigou['m10']/caigou['m00'], caigou['m01']/caigou['m00']
	except ZeroDivisionError:
		cx_b, cy_b = height/2, width/2
		cx_r, cy_r = height/2,width/2
		cx=0
	#print('x_b=',cx_b)
	#print('y=',cy)
	height, width, channels = frame.shape
	error_b=cx_b - width / 2  
	error_r=cx_r- width / 2
	error_x = error_b+error_r
	#error_r=cx_r - width / 2
	#error_x=error_r
	print('blue=',blue)
	print('red=',red)
	if red<100 and blue<100:
		angle=128
	elif blue<1000: #and red > 100:
		angle=50
	elif red<1000 :#and blue >100:
		angle=132

	
	else :
		angle = error_x*0.20+90#0.24
	if angle>75 and angle<105:
		angle=90
	#elif angle>100 and angle<=120:
	#	angle = 110
	#elif angle>120 and angle<=132:
	#	angle = 127 
	#elif angle >60 and angle<=80:
	#	angle = 70
	#elif angle>46 and angle<=60:
	#	angle = 53 
	twist = Twist()
	twist.linear.x = spe; twist.linear.y = 0; twist.linear.z = 0 
	twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = angle
	#print('speed= ',spe)
	#print('angle=',angle)

	pub.publish(twist)
	

	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

cap.release()
cv2.destroyAllWindows()

