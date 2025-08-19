;; Auto-generated. Do not edit!


(when (boundp 'fs_msgs::Track)
  (if (not (find-package "FS_MSGS"))
    (make-package "FS_MSGS"))
  (shadow 'Track (find-package "FS_MSGS")))
(unless (find-package "FS_MSGS::TRACK")
  (make-package "FS_MSGS::TRACK"))

(in-package "ROS")
;;//! \htmlinclude Track.msg.html


(defclass fs_msgs::Track
  :super ros::object
  :slots (_track ))

(defmethod fs_msgs::Track
  (:init
   (&key
    ((:track __track) ())
    )
   (send-super :init)
   (setq _track __track)
   self)
  (:track
   (&rest __track)
   (if (keywordp (car __track))
       (send* _track __track)
     (progn
       (if __track (setq _track (car __track)))
       _track)))
  (:serialization-length
   ()
   (+
    ;; fs_msgs/Cone[] _track
    (apply #'+ (send-all _track :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; fs_msgs/Cone[] _track
     (write-long (length _track) s)
     (dolist (elem _track)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; fs_msgs/Cone[] _track
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _track (let (r) (dotimes (i n) (push (instance fs_msgs::Cone :init) r)) r))
     (dolist (elem- _track)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get fs_msgs::Track :md5sum-) "03d754d3091336842daac1a974511950")
(setf (get fs_msgs::Track :datatype-) "fs_msgs/Track")
(setf (get fs_msgs::Track :definition-)
      "fs_msgs/Cone[] track
================================================================================
MSG: fs_msgs/Cone
geometry_msgs/Point location #x,y,z [m] wrt to the car start location (origin)
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



(provide :fs_msgs/Track "03d754d3091336842daac1a974511950")


