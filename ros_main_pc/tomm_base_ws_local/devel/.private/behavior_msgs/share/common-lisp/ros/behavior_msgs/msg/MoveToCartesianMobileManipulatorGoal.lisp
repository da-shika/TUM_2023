; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-msg)


;//! \htmlinclude MoveToCartesianMobileManipulatorGoal.msg.html

(cl:defclass <MoveToCartesianMobileManipulatorGoal> (roslisp-msg-protocol:ros-message)
  ((targets
    :reader targets
    :initarg :targets
    :type geometry_msgs-msg:PoseArray
    :initform (cl:make-instance 'geometry_msgs-msg:PoseArray))
   (motion_tasks
    :reader motion_tasks
    :initarg :motion_tasks
    :type (cl:vector std_msgs-msg:String)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:String :initial-element (cl:make-instance 'std_msgs-msg:String)))
   (period
    :reader period
    :initarg :period
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass MoveToCartesianMobileManipulatorGoal (<MoveToCartesianMobileManipulatorGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MoveToCartesianMobileManipulatorGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MoveToCartesianMobileManipulatorGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-msg:<MoveToCartesianMobileManipulatorGoal> is deprecated: use behavior_msgs-msg:MoveToCartesianMobileManipulatorGoal instead.")))

(cl:ensure-generic-function 'targets-val :lambda-list '(m))
(cl:defmethod targets-val ((m <MoveToCartesianMobileManipulatorGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:targets-val is deprecated.  Use behavior_msgs-msg:targets instead.")
  (targets m))

(cl:ensure-generic-function 'motion_tasks-val :lambda-list '(m))
(cl:defmethod motion_tasks-val ((m <MoveToCartesianMobileManipulatorGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:motion_tasks-val is deprecated.  Use behavior_msgs-msg:motion_tasks instead.")
  (motion_tasks m))

(cl:ensure-generic-function 'period-val :lambda-list '(m))
(cl:defmethod period-val ((m <MoveToCartesianMobileManipulatorGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:period-val is deprecated.  Use behavior_msgs-msg:period instead.")
  (period m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MoveToCartesianMobileManipulatorGoal>) ostream)
  "Serializes a message object of type '<MoveToCartesianMobileManipulatorGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'targets) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motion_tasks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motion_tasks))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'period) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MoveToCartesianMobileManipulatorGoal>) istream)
  "Deserializes a message object of type '<MoveToCartesianMobileManipulatorGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'targets) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motion_tasks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motion_tasks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:String))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'period) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MoveToCartesianMobileManipulatorGoal>)))
  "Returns string type for a message object of type '<MoveToCartesianMobileManipulatorGoal>"
  "behavior_msgs/MoveToCartesianMobileManipulatorGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MoveToCartesianMobileManipulatorGoal)))
  "Returns string type for a message object of type 'MoveToCartesianMobileManipulatorGoal"
  "behavior_msgs/MoveToCartesianMobileManipulatorGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MoveToCartesianMobileManipulatorGoal>)))
  "Returns md5sum for a message object of type '<MoveToCartesianMobileManipulatorGoal>"
  "68dd2db2f792d0ee848d89b6adce522f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MoveToCartesianMobileManipulatorGoal)))
  "Returns md5sum for a message object of type 'MoveToCartesianMobileManipulatorGoal"
  "68dd2db2f792d0ee848d89b6adce522f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MoveToCartesianMobileManipulatorGoal>)))
  "Returns full string definition for message of type '<MoveToCartesianMobileManipulatorGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%geometry_msgs/PoseArray targets  # target poses wrt frame_id in header field~%std_msgs/String[] motion_tasks       # motion task names for the movement~%std_msgs/Float64 period           # time period of the motion~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MoveToCartesianMobileManipulatorGoal)))
  "Returns full string definition for message of type 'MoveToCartesianMobileManipulatorGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%geometry_msgs/PoseArray targets  # target poses wrt frame_id in header field~%std_msgs/String[] motion_tasks       # motion task names for the movement~%std_msgs/Float64 period           # time period of the motion~%~%================================================================================~%MSG: geometry_msgs/PoseArray~%# An array of poses with a header for global reference.~%~%Header header~%~%Pose[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MoveToCartesianMobileManipulatorGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'targets))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motion_tasks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'period))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MoveToCartesianMobileManipulatorGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'MoveToCartesianMobileManipulatorGoal
    (cl:cons ':targets (targets msg))
    (cl:cons ':motion_tasks (motion_tasks msg))
    (cl:cons ':period (period msg))
))
