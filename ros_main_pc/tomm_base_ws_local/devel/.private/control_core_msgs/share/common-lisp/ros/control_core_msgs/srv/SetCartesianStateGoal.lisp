; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-srv)


;//! \htmlinclude SetCartesianStateGoal-request.msg.html

(cl:defclass <SetCartesianStateGoal-request> (roslisp-msg-protocol:ros-message)
  ((goal
    :reader goal
    :initarg :goal
    :type control_core_msgs-msg:CartesianState
    :initform (cl:make-instance 'control_core_msgs-msg:CartesianState)))
)

(cl:defclass SetCartesianStateGoal-request (<SetCartesianStateGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCartesianStateGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCartesianStateGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<SetCartesianStateGoal-request> is deprecated: use control_core_msgs-srv:SetCartesianStateGoal-request instead.")))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <SetCartesianStateGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:goal-val is deprecated.  Use control_core_msgs-srv:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCartesianStateGoal-request>) ostream)
  "Serializes a message object of type '<SetCartesianStateGoal-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCartesianStateGoal-request>) istream)
  "Deserializes a message object of type '<SetCartesianStateGoal-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCartesianStateGoal-request>)))
  "Returns string type for a service object of type '<SetCartesianStateGoal-request>"
  "control_core_msgs/SetCartesianStateGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianStateGoal-request)))
  "Returns string type for a service object of type 'SetCartesianStateGoal-request"
  "control_core_msgs/SetCartesianStateGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCartesianStateGoal-request>)))
  "Returns md5sum for a message object of type '<SetCartesianStateGoal-request>"
  "2f3406a57b13431c34e22c35d33bdee4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCartesianStateGoal-request)))
  "Returns md5sum for a message object of type 'SetCartesianStateGoal-request"
  "2f3406a57b13431c34e22c35d33bdee4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCartesianStateGoal-request>)))
  "Returns full string definition for message of type '<SetCartesianStateGoal-request>"
  (cl:format cl:nil "CartesianState goal~%~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCartesianStateGoal-request)))
  "Returns full string definition for message of type 'SetCartesianStateGoal-request"
  (cl:format cl:nil "CartesianState goal~%~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCartesianStateGoal-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCartesianStateGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCartesianStateGoal-request
    (cl:cons ':goal (goal msg))
))
;//! \htmlinclude SetCartesianStateGoal-response.msg.html

(cl:defclass <SetCartesianStateGoal-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetCartesianStateGoal-response (<SetCartesianStateGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCartesianStateGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCartesianStateGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<SetCartesianStateGoal-response> is deprecated: use control_core_msgs-srv:SetCartesianStateGoal-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCartesianStateGoal-response>) ostream)
  "Serializes a message object of type '<SetCartesianStateGoal-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCartesianStateGoal-response>) istream)
  "Deserializes a message object of type '<SetCartesianStateGoal-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCartesianStateGoal-response>)))
  "Returns string type for a service object of type '<SetCartesianStateGoal-response>"
  "control_core_msgs/SetCartesianStateGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianStateGoal-response)))
  "Returns string type for a service object of type 'SetCartesianStateGoal-response"
  "control_core_msgs/SetCartesianStateGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCartesianStateGoal-response>)))
  "Returns md5sum for a message object of type '<SetCartesianStateGoal-response>"
  "2f3406a57b13431c34e22c35d33bdee4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCartesianStateGoal-response)))
  "Returns md5sum for a message object of type 'SetCartesianStateGoal-response"
  "2f3406a57b13431c34e22c35d33bdee4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCartesianStateGoal-response>)))
  "Returns full string definition for message of type '<SetCartesianStateGoal-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCartesianStateGoal-response)))
  "Returns full string definition for message of type 'SetCartesianStateGoal-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCartesianStateGoal-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCartesianStateGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCartesianStateGoal-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCartesianStateGoal)))
  'SetCartesianStateGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCartesianStateGoal)))
  'SetCartesianStateGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianStateGoal)))
  "Returns string type for a service object of type '<SetCartesianStateGoal>"
  "control_core_msgs/SetCartesianStateGoal")