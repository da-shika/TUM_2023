; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-msg)


;//! \htmlinclude MoveToCartesianMobileManipulatorActionGoal.msg.html

(cl:defclass <MoveToCartesianMobileManipulatorActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
    :type behavior_msgs-msg:MoveToCartesianMobileManipulatorGoal
    :initform (cl:make-instance 'behavior_msgs-msg:MoveToCartesianMobileManipulatorGoal)))
)

(cl:defclass MoveToCartesianMobileManipulatorActionGoal (<MoveToCartesianMobileManipulatorActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToCartesianMobileManipulatorActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToCartesianMobileManipulatorActionGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-msg:<MoveToCartesianMobileManipulatorActionGoal> is deprecated: use behavior_msgs-msg:MoveToCartesianMobileManipulatorActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MoveToCartesianMobileManipulatorActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:header-val is deprecated.  Use behavior_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <MoveToCartesianMobileManipulatorActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:goal_id-val is deprecated.  Use behavior_msgs-msg:goal_id instead.")
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <MoveToCartesianMobileManipulatorActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:goal-val is deprecated.  Use behavior_msgs-msg:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToCartesianMobileManipulatorActionGoal>) ostream)
  "Serializes a message object of type '<MoveToCartesianMobileManipulatorActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToCartesianMobileManipulatorActionGoal>) istream)
  "Deserializes a message object of type '<MoveToCartesianMobileManipulatorActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToCartesianMobileManipulatorActionGoal>)))
  "Returns string type for a message object of type '<MoveToCartesianMobileManipulatorActionGoal>"
  "behavior_msgs/MoveToCartesianMobileManipulatorActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToCartesianMobileManipulatorActionGoal)))
  "Returns string type for a message object of type 'MoveToCartesianMobileManipulatorActionGoal"
  "behavior_msgs/MoveToCartesianMobileManipulatorActionGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToCartesianMobileManipulatorActionGoal>)))
  "Returns md5sum for a message object of type '<MoveToCartesianMobileManipulatorActionGoal>"
  "662ee4fb355ee3052f1f9ff67c58f319")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToCartesianMobileManipulatorActionGoal)))
  "Returns md5sum for a message object of type 'MoveToCartesianMobileManipulatorActionGoal"
  "662ee4fb355ee3052f1f9ff67c58f319")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToCartesianMobileManipulatorActionGoal>)))
  "Returns full string definition for message of type '<MoveToCartesianMobileManipulatorActionGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%MoveToCartesianMobileManipulatorGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: behavior_msgs/MoveToCartesianMobileManipulatorGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%geometry_msgs/PoseArray targets  # target poses wrt frame_id in header field~%std_msgs/String[] motion_tasks       # motion task names for the movement~%std_msgs/Float64 period           # time period of the motion~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToCartesianMobileManipulatorActionGoal)))
  "Returns full string definition for message of type 'MoveToCartesianMobileManipulatorActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%MoveToCartesianMobileManipulatorGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: behavior_msgs/MoveToCartesianMobileManipulatorGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%geometry_msgs/PoseArray targets  # target poses wrt frame_id in header field~%std_msgs/String[] motion_tasks       # motion task names for the movement~%std_msgs/Float64 period           # time period of the motion~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToCartesianMobileManipulatorActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToCartesianMobileManipulatorActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToCartesianMobileManipulatorActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))