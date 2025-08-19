;; Auto-generated. Do not edit!


(when (boundp 'fs_msgs::ExtraInfo)
  (if (not (find-package "FS_MSGS"))
    (make-package "FS_MSGS"))
  (shadow 'ExtraInfo (find-package "FS_MSGS")))
(unless (find-package "FS_MSGS::EXTRAINFO")
  (make-package "FS_MSGS::EXTRAINFO"))

(in-package "ROS")
;;//! \htmlinclude ExtraInfo.msg.html


(defclass fs_msgs::ExtraInfo
  :super ros::object
  :slots (_doo_counter _laps ))

(defmethod fs_msgs::ExtraInfo
  (:init
   (&key
    ((:doo_counter __doo_counter) 0)
    ((:laps __laps) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _doo_counter (round __doo_counter))
   (setq _laps __laps)
   self)
  (:doo_counter
   (&optional __doo_counter)
   (if __doo_counter (setq _doo_counter __doo_counter)) _doo_counter)
  (:laps
   (&optional __laps)
   (if __laps (setq _laps __laps)) _laps)
  (:serialization-length
   ()
   (+
    ;; uint32 _doo_counter
    4
    ;; float32[] _laps
    (* 4    (length _laps)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _doo_counter
       (write-long _doo_counter s)
     ;; float32[] _laps
     (write-long (length _laps) s)
     (dotimes (i (length _laps))
       (sys::poke (elt _laps i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _doo_counter
     (setq _doo_counter (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32[] _laps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _laps (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _laps i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get fs_msgs::ExtraInfo :md5sum-) "722e1c6d60f33a21de73c32b7583c9b8")
(setf (get fs_msgs::ExtraInfo :datatype-) "fs_msgs/ExtraInfo")
(setf (get fs_msgs::ExtraInfo :definition-)
      "# The number of Down Or Out cones during the run
uint32 doo_counter

# The number of finished laps driven by the vehicle
float32[] laps 
")



(provide :fs_msgs/ExtraInfo "722e1c6d60f33a21de73c32b7583c9b8")


