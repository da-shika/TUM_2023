; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude JointStateStamped.msg.html

(cl:defclass <JointStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type control_core_msgs-msg:JointState
    :initform (cl:make-instance 'control_core_msgs-msg:JointState)))
)

(cl:defclass JointStateStamped (<JointStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<JointStateStamped> is deprecated: use control_core_msgs-msg:JointStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JointStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:header-val is deprecated.  Use control_core_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <JointStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:state-val is deprecated.  Use control_core_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointStateStamped>) ostream)
  "Serializes a message object of type '<JointStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointStateStamped>) istream)
  "Deserializes a message object of type '<JointStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointStateStamped>)))
  "Returns string type for a message object of type '<JointStateStamped>"
  "control_core_msgs/JointStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointStateStamped)))
  "Returns string type for a message object of type 'JointStateStamped"
  "control_core_msgs/JointStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointStateStamped>)))
  "Returns md5sum for a message object of type '<JointStateStamped>"
  "6bec66d356b4dc145ca7abf84a28bd9b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointStateStamped)))
  "Returns md5sum for a message object of type 'JointStateStamped"
  "6bec66d356b4dc145ca7abf84a28bd9b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointStateStamped>)))
  "Returns full string definition for message of type '<JointStateStamped>"
  (cl:format cl:nil "std_msgs/Header header~%JointState state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointStateStamped)))
  "Returns full string definition for message of type 'JointStateStamped"
  (cl:format cl:nil "std_msgs/Header header~%JointState state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'JointStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
