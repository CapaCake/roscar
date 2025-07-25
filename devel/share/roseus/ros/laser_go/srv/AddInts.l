;; Auto-generated. Do not edit!


(when (boundp 'laser_go::AddInts)
  (if (not (find-package "LASER_GO"))
    (make-package "LASER_GO"))
  (shadow 'AddInts (find-package "LASER_GO")))
(unless (find-package "LASER_GO::ADDINTS")
  (make-package "LASER_GO::ADDINTS"))
(unless (find-package "LASER_GO::ADDINTSREQUEST")
  (make-package "LASER_GO::ADDINTSREQUEST"))
(unless (find-package "LASER_GO::ADDINTSRESPONSE")
  (make-package "LASER_GO::ADDINTSRESPONSE"))

(in-package "ROS")





(defclass laser_go::AddIntsRequest
  :super ros::object
  :slots (_num1 _num2 ))

(defmethod laser_go::AddIntsRequest
  (:init
   (&key
    ((:num1 __num1) 0)
    ((:num2 __num2) 0)
    )
   (send-super :init)
   (setq _num1 (round __num1))
   (setq _num2 (round __num2))
   self)
  (:num1
   (&optional __num1)
   (if __num1 (setq _num1 __num1)) _num1)
  (:num2
   (&optional __num2)
   (if __num2 (setq _num2 __num2)) _num2)
  (:serialization-length
   ()
   (+
    ;; int32 _num1
    4
    ;; int32 _num2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _num1
       (write-long _num1 s)
     ;; int32 _num2
       (write-long _num2 s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _num1
     (setq _num1 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _num2
     (setq _num2 (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass laser_go::AddIntsResponse
  :super ros::object
  :slots (_sum ))

(defmethod laser_go::AddIntsResponse
  (:init
   (&key
    ((:sum __sum) 0)
    )
   (send-super :init)
   (setq _sum (round __sum))
   self)
  (:sum
   (&optional __sum)
   (if __sum (setq _sum __sum)) _sum)
  (:serialization-length
   ()
   (+
    ;; int32 _sum
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _sum
       (write-long _sum s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _sum
     (setq _sum (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass laser_go::AddInts
  :super ros::object
  :slots ())

(setf (get laser_go::AddInts :md5sum-) "4781436a0c2affec8025955a6041e481")
(setf (get laser_go::AddInts :datatype-) "laser_go/AddInts")
(setf (get laser_go::AddInts :request) laser_go::AddIntsRequest)
(setf (get laser_go::AddInts :response) laser_go::AddIntsResponse)

(defmethod laser_go::AddIntsRequest
  (:response () (instance laser_go::AddIntsResponse :init)))

(setf (get laser_go::AddIntsRequest :md5sum-) "4781436a0c2affec8025955a6041e481")
(setf (get laser_go::AddIntsRequest :datatype-) "laser_go/AddIntsRequest")
(setf (get laser_go::AddIntsRequest :definition-)
      "int32 num1
int32 num2
---
int32 sum

")

(setf (get laser_go::AddIntsResponse :md5sum-) "4781436a0c2affec8025955a6041e481")
(setf (get laser_go::AddIntsResponse :datatype-) "laser_go/AddIntsResponse")
(setf (get laser_go::AddIntsResponse :definition-)
      "int32 num1
int32 num2
---
int32 sum

")



(provide :laser_go/AddInts "4781436a0c2affec8025955a6041e481")


