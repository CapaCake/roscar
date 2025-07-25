from time import thread_time
import cv2
import numpy as np
from std_msgs.msg import Int64
import rospy
import threading
from std_msgs.msg import String
import os
import time


class Cam():
    def __init__(self) -> None:
        self.lower_yellow = np.array([0, 60, 205])
        self.upper_yellow = np.array([179, 255, 255])
        self.lower_blue = np.array([0, 0, 174])
        self.upper_blue = np.array([182, 59, 255])
        self.lower_green = np.array([45, 180, 40])
        self.upper_green = np.array([82, 255, 255])
        self.lower_white = np.array([78, 43, 200])
        self.upper_white = np.array([104, 255, 255])
        rospy.init_node('racecar_teleop')  # 引用函数
        self.runNum = 0
        self.smgPub = rospy.Publisher("/mode", String, queue_size=5)
        self.modeNum = "Mode0"

    def run_cam(self):
        capture = cv2.VideoCapture(0)
        while True:
            print(self.runNum)
            print(self.modeNum)
            ret, frame = capture.read()
            # mask_green = self.process_img(
            #     frame, self.lower_green, self.upper_green)
            mask_blue = self.process_img(
                frame, self.lower_blue, self.upper_blue)
            area = cv2.countNonZero(mask_blue)
            #print(area)
            if self.modeNum == "Mode0" and self.runNum == 0 and area >= 600:
                self.setMode("Mode1")
                self.runNum = 1
                self.saveMap()
                

            if self.modeNum == "Mode0" and self.runNum >= 1 and area >= 75:
                self.setMode("Mode2")
            #cv2.imshow("123", frame)
            #cv2.imshow("789", mask_blue)
            if cv2.waitKey(1) == ord('q'):
                break

    def process_img(self, img, lower_maskcolor: np.array, upper_maskcolor: np.array):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_maskcolor, upper_maskcolor)
        #k = np.ones((5, 5), dtype=np.uint8)  # type: ignore
        #eroded = cv2.erode(mask, k, iterations=1)
        return mask

    def setMode(self, modestr: str):
        modeSmg = String()
        if modestr == "Mode1":
            modeSmg.data = "Mode1"
            self.smgPub.publish(modeSmg)
            self.setModeTimer(6, "Mode0")
        elif modestr == "Mode2":
            modeSmg.data = "Mode2"
            self.smgPub.publish(modeSmg)
            self.setModeTimer(1.5, "Mode1")

        elif modestr == "Mode0":
            modeSmg.data = "Mode0"
            self.smgPub.publish(modeSmg)
        self.modeNum = modestr

    def setModeTimer(self, time: float, modeStr: str):
        self.runNum = self.runNum+1
        self.setModeTimerTr = threading.Timer(time, self.setMode, (modeStr,))
        self.setModeTimerTr.start()

    def saveMap(self):
        print(os.system("bash /home/racecar/smartcar/src/racecar/save_map.sh"))


if __name__ == "__main__":
    app = Cam()
    app.run_cam()
