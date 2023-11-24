; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-msg)


;//! \htmlinclude MoveToCartesianMobileManipulatorResult.msg.html

(cl:defclass <MoveToCartesianMobileManipulatorResult> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (real
    :reader real
    :initarg :real
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray)))
)

(cl:defclass MoveToCartesianMobileManipulatorResult (<MoveToCartesianMobileManipulatorResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToCartesianMobileManipulatorResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToCartesianMobileManipulatorResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-msg:<MoveToCartesianMobileManipulatorResult> is deprecated: use behavior_msgs-msg:MoveToCartesianMobileManipulatorResult instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <MoveToCartesianMobileManipulatorResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:cmd-val is deprecated.  Use behavior_msgs-msg:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'real-val :lambda-list '(m))
(cl:defmethod real-val ((m <MoveToCartesianMobileManipulatorResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:real-val is deprecated.  Use behavior_msgs-msg:real instead.")
  (real m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToCartesianMobileManipulatorResult>) ostream)
  "Serializes a message object of type '<MoveToCartesianMobileManipulatorResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cmd) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'real) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToCartesianMobileManipulatorResult>) istream)
  "Deserializes a message object of type '<MoveToCartesianMobileManipulatorResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cmd) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'real) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToCartesianMobileManipulatorResult>)))
  "Returns string type for a message object of type '<MoveToCartesianMobileManipulatorResult>"
  "behavior_msgs/MoveToCartesianMobileManipulatorResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToCartesianMobileManipulatorResult)))
  "Returns string type for a message object of type 'MoveToCartesianMobileManipulatorResult"
  "behavior_msgs/MoveToCartesianMobileManipulatorResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToCartesianMobileManipulatorResult>)))
  "Returns md5sum for a message object of type '<MoveToCartesianMobileManipulatorResult>"
  "eb2a6bc542e35660b66d4996e310cce8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToCartesianMobileManipulatorResult)))
  "Returns md5sum for a message object of type 'MoveToCartesianMobileManipulatorResult"
  "eb2a6bc542e35660b66d4996e310cce8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToCartesianMobileManipulatorResult>)))
  "Returns full string definition for message of type '<MoveToCartesianMobileManipulatorResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%geometry_msgs/PoseArray cmd~%geometry_msgs/PoseArray real~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToCartesianMobileManipulatorResult)))
  "Returns full string definition for message of type 'MoveToCartesianMobileManipulatorResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%geometry_msgs/PoseArray cmd~%geometry_msgs/PoseArray real~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToCartesianMobileManipulatorResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cmd))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'real))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToCartesianMobileManipulatorResult>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToCartesianMobileManipulatorResult
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':real (real msg))
))
