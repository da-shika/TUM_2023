; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude LinearStateStamped.msg.html

(cl:defclass <LinearStateStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type control_core_msgs-msg:LinearState
    :initform (cl:make-instance 'control_core_msgs-msg:LinearState)))
)

(cl:defclass LinearStateStamped (<LinearStateStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinearStateStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinearStateStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<LinearStateStamped> is deprecated: use control_core_msgs-msg:LinearStateStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LinearStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:header-val is deprecated.  Use control_core_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <LinearStateStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:state-val is deprecated.  Use control_core_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinearStateStamped>) ostream)
  "Serializes a message object of type '<LinearStateStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinearStateStamped>) istream)
  "Deserializes a message object of type '<LinearStateStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinearStateStamped>)))
  "Returns string type for a message object of type '<LinearStateStamped>"
  "control_core_msgs/LinearStateStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearStateStamped)))
  "Returns string type for a message object of type 'LinearStateStamped"
  "control_core_msgs/LinearStateStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinearStateStamped>)))
  "Returns md5sum for a message object of type '<LinearStateStamped>"
  "b1b7305065501d4951b8d88b820348e4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinearStateStamped)))
  "Returns md5sum for a message object of type 'LinearStateStamped"
  "b1b7305065501d4951b8d88b820348e4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinearStateStamped>)))
  "Returns full string definition for message of type '<LinearStateStamped>"
  (cl:format cl:nil "std_msgs/Header header~%LinearState state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: control_core_msgs/LinearState~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinearStateStamped)))
  "Returns full string definition for message of type 'LinearStateStamped"
  (cl:format cl:nil "std_msgs/Header header~%LinearState state~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: control_core_msgs/LinearState~%geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinearStateStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinearStateStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'LinearStateStamped
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
))
