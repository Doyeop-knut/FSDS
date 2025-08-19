;; Auto-generated. Do not edit!


(when (boundp 'fs_msgs::GoSignal)
  (if (not (find-package "FS_MSGS"))
    (make-package "FS_MSGS"))
  (shadow 'GoSignal (find-package "FS_MSGS")))
(unless (find-package "FS_MSGS::GOSIGNAL")
  (make-package "FS_MSGS::GOSIGNAL"))

(in-package "ROS")
;;//! \htmlinclude GoSignal.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fs_msgs::GoSignal
  :super ros::object
  :slots (_header _mission _track ))

(defmethod fs_msgs::GoSignal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mission __mission) "")
    ((:track __track) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _mission (string __mission))
   (setq _track (string __track))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mission
   (&optional __mission)
   (if __mission (setq _mission __mission)) _mission)
  (:track
   (&optional __track)
   (if __track (setq _track __track)) _track)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _mission
    4 (length _mission)
    ;; string _track
    4 (length _track)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _mission
       (write-long (length _mission) s) (princ _mission s)
     ;; string _track
       (write-long (length _track) s) (princ _track s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _mission
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mission (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _track
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _track (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get fs_msgs::GoSignal :md5sum-) "1901c640b21e17abe7d6ab0e2bf50a8a")
(setf (get fs_msgs::GoSignal :datatype-) "fs_msgs/GoSignal")
(setf (get fs_msgs::GoSignal :definition-)
      "Header header
string mission
string track
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :fs_msgs/GoSignal "1901c640b21e17abe7d6ab0e2bf50a8a")


