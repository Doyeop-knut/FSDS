; Auto-generated. Do not edit!


(cl:in-package fs_msgs-msg)


;//! \htmlinclude FinishedSignal.msg.html

(cl:defclass <FinishedSignal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (placeholder
    :reader placeholder
    :initarg :placeholder
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FinishedSignal (<FinishedSignal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FinishedSignal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FinishedSignal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fs_msgs-msg:<FinishedSignal> is deprecated: use fs_msgs-msg:FinishedSignal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FinishedSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:header-val is deprecated.  Use fs_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'placeholder-val :lambda-list '(m))
(cl:defmethod placeholder-val ((m <FinishedSignal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fs_msgs-msg:placeholder-val is deprecated.  Use fs_msgs-msg:placeholder instead.")
  (placeholder m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FinishedSignal>) ostream)
  "Serializes a message object of type '<FinishedSignal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'placeholder) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FinishedSignal>) istream)
  "Deserializes a message object of type '<FinishedSignal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'placeholder) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FinishedSignal>)))
  "Returns string type for a message object of type '<FinishedSignal>"
  "fs_msgs/FinishedSignal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FinishedSignal)))
  "Returns string type for a message object of type 'FinishedSignal"
  "fs_msgs/FinishedSignal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FinishedSignal>)))
  "Returns md5sum for a message object of type '<FinishedSignal>"
  "0fadad42372ab12322143ef83bb4835a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FinishedSignal)))
  "Returns md5sum for a message object of type 'FinishedSignal"
  "0fadad42372ab12322143ef83bb4835a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FinishedSignal>)))
  "Returns full string definition for message of type '<FinishedSignal>"
  (cl:format cl:nil "Header header~%bool placeholder~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FinishedSignal)))
  "Returns full string definition for message of type 'FinishedSignal"
  (cl:format cl:nil "Header header~%bool placeholder~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FinishedSignal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FinishedSignal>))
  "Converts a ROS message object to a list"
  (cl:list 'FinishedSignal
    (cl:cons ':header (header msg))
    (cl:cons ':placeholder (placeholder msg))
))
