;; Auto-generated. Do not edit!


(when (boundp 'fs_msgs::Cone)
  (if (not (find-package "FS_MSGS"))
    (make-package "FS_MSGS"))
  (shadow 'Cone (find-package "FS_MSGS")))
(unless (find-package "FS_MSGS::CONE")
  (make-package "FS_MSGS::CONE"))

(in-package "ROS")
;;//! \htmlinclude Cone.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*BLUE*" (find-package "FS_MSGS::CONE"))
(shadow '*BLUE* (find-package "FS_MSGS::CONE"))
(defconstant fs_msgs::Cone::*BLUE* 0)
(intern "*YELLOW*" (find-package "FS_MSGS::CONE"))
(shadow '*YELLOW* (find-package "FS_MSGS::CONE"))
(defconstant fs_msgs::Cone::*YELLOW* 1)
(intern "*ORANGE_BIG*" (find-package "FS_MSGS::CONE"))
(shadow '*ORANGE_BIG* (find-package "FS_MSGS::CONE"))
(defconstant fs_msgs::Cone::*ORANGE_BIG* 2)
(intern "*ORANGE_SMALL*" (find-package "FS_MSGS::CONE"))
(shadow '*ORANGE_SMALL* (find-package "FS_MSGS::CONE"))
(defconstant fs_msgs::Cone::*ORANGE_SMALL* 3)
(intern "*UNKNOWN*" (find-package "FS_MSGS::CONE"))
(shadow '*UNKNOWN* (find-package "FS_MSGS::CONE"))
(defconstant fs_msgs::Cone::*UNKNOWN* 4)

(defun fs_msgs::Cone-to-symbol (const)
  (cond
        ((= const 0) 'fs_msgs::Cone::*BLUE*)
        ((= const 1) 'fs_msgs::Cone::*YELLOW*)
        ((= const 2) 'fs_msgs::Cone::*ORANGE_BIG*)
        ((= const 3) 'fs_msgs::Cone::*ORANGE_SMALL*)
        ((= const 4) 'fs_msgs::Cone::*UNKNOWN*)
        (t nil)))

(defclass fs_msgs::Cone
  :super ros::object
  :slots (_location _color ))

(defmethod fs_msgs::Cone
  (:init
   (&key
    ((:location __location) (instance geometry_msgs::Point :init))
    ((:color __color) 0)
    )
   (send-super :init)
   (setq _location __location)
   (setq _color (round __color))
   self)
  (:location
   (&rest __location)
   (if (keywordp (car __location))
       (send* _location __location)
     (progn
       (if __location (setq _location (car __location)))
       _location)))
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _location
    (send _location :serialization-length)
    ;; uint8 _color
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _location
       (send _location :serialize s)
     ;; uint8 _color
       (write-byte _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _location
     (send _location :deserialize buf ptr-) (incf ptr- (send _location :serialization-length))
   ;; uint8 _color
     (setq _color (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fs_msgs::Cone :md5sum-) "ce1c29c0358a1a503b00a09580fddb69")
(setf (get fs_msgs::Cone :datatype-) "fs_msgs/Cone")
(setf (get fs_msgs::Cone :definition-)
      "geometry_msgs/Point location #x,y,z [m] wrt to the car start location (origin)
uint8 color # use the enum below
# Constants
uint8 BLUE=0
uint8 YELLOW=1
uint8 ORANGE_BIG=2
uint8 ORANGE_SMALL=3
uint8 UNKNOWN=4
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :fs_msgs/Cone "ce1c29c0358a1a503b00a09580fddb69")


