; Auto-generated. Do not edit!


(cl:in-package fs_msgs-msg)


;//! \htmlinclude GoSignal.msg.html

(cl:defclass <GoSignal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (mission
    :reader mission
    :initarg :mission
    :type cl:string
    :initform "")
   (track
    :reader track
    :initarg :track
    :type cl:string
    :initform ""))
)

(cl:defclass GoSignal (<GoSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GoSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GoSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fs_msgs-msg:<GoSignal> is deprecated: use fs_msgs-msg:GoSignal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GoSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:header-val is deprecated.  Use fs_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'mission-val :lambda-list '(m))
(cl:defmethod mission-val ((m <GoSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:mission-val is deprecated.  Use fs_msgs-msg:mission instead.")
  (mission m))

(cl:ensure-generic-function 'track-val :lambda-list '(m))
(cl:defmethod track-val ((m <GoSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:track-val is deprecated.  Use fs_msgs-msg:track instead.")
  (track m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GoSignal>) ostream)
  "Serializes a message object of type '<GoSignal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mission))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mission))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'track))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'track))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GoSignal>) istream)
  "Deserializes a message object of type '<GoSignal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mission) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mission) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'track) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'track) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GoSignal>)))
  "Returns string type for a message object of type '<GoSignal>"
  "fs_msgs/GoSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GoSignal)))
  "Returns string type for a message object of type 'GoSignal"
  "fs_msgs/GoSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GoSignal>)))
  "Returns md5sum for a message object of type '<GoSignal>"
  "1901c640b21e17abe7d6ab0e2bf50a8a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GoSignal)))
  "Returns md5sum for a message object of type 'GoSignal"
  "1901c640b21e17abe7d6ab0e2bf50a8a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GoSignal>)))
  "Returns full string definition for message of type '<GoSignal>"
  (cl:format cl:nil "Header header~%string mission~%string track~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GoSignal)))
  "Returns full string definition for message of type 'GoSignal"
  (cl:format cl:nil "Header header~%string mission~%string track~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GoSignal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'mission))
     4 (cl:length (cl:slot-value msg 'track))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GoSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'GoSignal
    (cl:cons ':header (header msg))
    (cl:cons ':mission (mission msg))
    (cl:cons ':track (track msg))
))
