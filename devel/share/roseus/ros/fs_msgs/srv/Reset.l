;; Auto-generated. Do not edit!


(when (boundp 'fs_msgs::Reset)
  (if (not (find-package "FS_MSGS"))
    (make-package "FS_MSGS"))
  (shadow 'Reset (find-package "FS_MSGS")))
(unless (find-package "FS_MSGS::RESET")
  (make-package "FS_MSGS::RESET"))
(unless (find-package "FS_MSGS::RESETREQUEST")
  (make-package "FS_MSGS::RESETREQUEST"))
(unless (find-package "FS_MSGS::RESETRESPONSE")
  (make-package "FS_MSGS::RESETRESPONSE"))

(in-package "ROS")





(defclass fs_msgs::ResetRequest
  :super ros::object
  :slots (_waitOnLastTask ))

(defmethod fs_msgs::ResetRequest
  (:init
   (&key
    ((:waitOnLastTask __waitOnLastTask) nil)
    )
   (send-super :init)
   (setq _waitOnLastTask __waitOnLastTask)
   self)
  (:waitOnLastTask
   (&optional (__waitOnLastTask :null))
   (if (not (eq __waitOnLastTask :null)) (setq _waitOnLastTask __waitOnLastTask)) _waitOnLastTask)
  (:serialization-length
   ()
   (+
    ;; bool _waitOnLastTask
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _waitOnLastTask
       (if _waitOnLastTask (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _waitOnLastTask
     (setq _waitOnLastTask (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass fs_msgs::ResetResponse
  :super ros::object
  :slots ())

(defmethod fs_msgs::ResetResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass fs_msgs::Reset
  :super ros::object
  :slots ())

(setf (get fs_msgs::Reset :md5sum-) "79b3eea4e90fd67af16afc9085f08e12")
(setf (get fs_msgs::Reset :datatype-) "fs_msgs/Reset")
(setf (get fs_msgs::Reset :request) fs_msgs::ResetRequest)
(setf (get fs_msgs::Reset :response) fs_msgs::ResetResponse)

(defmethod fs_msgs::ResetRequest
  (:response () (instance fs_msgs::ResetResponse :init)))

(setf (get fs_msgs::ResetRequest :md5sum-) "79b3eea4e90fd67af16afc9085f08e12")
(setf (get fs_msgs::ResetRequest :datatype-) "fs_msgs/ResetRequest")
(setf (get fs_msgs::ResetRequest :definition-)
      "bool waitOnLastTask 
---

")

(setf (get fs_msgs::ResetResponse :md5sum-) "79b3eea4e90fd67af16afc9085f08e12")
(setf (get fs_msgs::ResetResponse :datatype-) "fs_msgs/ResetResponse")
(setf (get fs_msgs::ResetResponse :definition-)
      "bool waitOnLastTask 
---

")



(provide :fs_msgs/Reset "79b3eea4e90fd67af16afc9085f08e12")


