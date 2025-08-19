; Auto-generated. Do not edit!


(cl:in-package fs_msgs-msg)


;//! \htmlinclude ExtraInfo.msg.html

(cl:defclass <ExtraInfo> (roslisp-msg-protocol:ros-message)
  ((doo_counter
    :reader doo_counter
    :initarg :doo_counter
    :type cl:integer
    :initform 0)
   (laps
    :reader laps
    :initarg :laps
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass ExtraInfo (<ExtraInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExtraInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExtraInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fs_msgs-msg:<ExtraInfo> is deprecated: use fs_msgs-msg:ExtraInfo instead.")))

(cl:ensure-generic-function 'doo_counter-val :lambda-list '(m))
(cl:defmethod doo_counter-val ((m <ExtraInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:doo_counter-val is deprecated.  Use fs_msgs-msg:doo_counter instead.")
  (doo_counter m))

(cl:ensure-generic-function 'laps-val :lambda-list '(m))
(cl:defmethod laps-val ((m <ExtraInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:laps-val is deprecated.  Use fs_msgs-msg:laps instead.")
  (laps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExtraInfo>) ostream)
  "Serializes a message object of type '<ExtraInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'doo_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'doo_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'doo_counter)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'doo_counter)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'laps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'laps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExtraInfo>) istream)
  "Deserializes a message object of type '<ExtraInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'doo_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'doo_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'doo_counter)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'doo_counter)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'laps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'laps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExtraInfo>)))
  "Returns string type for a message object of type '<ExtraInfo>"
  "fs_msgs/ExtraInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtraInfo)))
  "Returns string type for a message object of type 'ExtraInfo"
  "fs_msgs/ExtraInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExtraInfo>)))
  "Returns md5sum for a message object of type '<ExtraInfo>"
  "722e1c6d60f33a21de73c32b7583c9b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExtraInfo)))
  "Returns md5sum for a message object of type 'ExtraInfo"
  "722e1c6d60f33a21de73c32b7583c9b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExtraInfo>)))
  "Returns full string definition for message of type '<ExtraInfo>"
  (cl:format cl:nil "# The number of Down Or Out cones during the run~%uint32 doo_counter~%~%# The number of finished laps driven by the vehicle~%float32[] laps ~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExtraInfo)))
  "Returns full string definition for message of type 'ExtraInfo"
  (cl:format cl:nil "# The number of Down Or Out cones during the run~%uint32 doo_counter~%~%# The number of finished laps driven by the vehicle~%float32[] laps ~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExtraInfo>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'laps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExtraInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'ExtraInfo
    (cl:cons ':doo_counter (doo_counter msg))
    (cl:cons ':laps (laps msg))
))
