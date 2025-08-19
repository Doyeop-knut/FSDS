;; Auto-generated. Do not edit!


(when (boundp 'fs_msgs::FinishedSignal)
  (if (not (find-package "FS_MSGS"))
    (make-package "FS_MSGS"))
  (shadow 'FinishedSignal (find-package "FS_MSGS")))
(unless (find-package "FS_MSGS::FINISHEDSIGNAL")
  (make-package "FS_MSGS::FINISHEDSIGNAL"))

(in-package "ROS")
;;//! \htmlinclude FinishedSignal.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fs_msgs::FinishedSignal
  :super ros::object
  :slots (_header _placeholder ))

(defmethod fs_msgs::FinishedSignal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:placeholder __placeholder) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _placeholder __placeholder)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:placeholder
   (&optional (__placeholder :null))
   (if (not (eq __placeholder :null)) (setq _placeholder __placeholder)) _placeholder)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _placeholder
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _placeholder
       (if _placeholder (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _placeholder
     (setq _placeholder (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get fs_msgs::FinishedSignal :md5sum-) "0fadad42372ab12322143ef83bb4835a")
(setf (get fs_msgs::FinishedSignal :datatype-) "fs_msgs/FinishedSignal")
(setf (get fs_msgs::FinishedSignal :definition-)
      "Header header
bool placeholder

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



(provide :fs_msgs/FinishedSignal "0fadad42372ab12322143ef83bb4835a")


