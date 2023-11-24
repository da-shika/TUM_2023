; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-srv)


;//! \htmlinclude SetCartesianPositionGoal-request.msg.html

(cl:defclass <SetCartesianPositionGoal-request> (roslisp-msg-protocol:ros-message)
  ((goal
    :reader goal
    :initarg :goal
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass SetCartesianPositionGoal-request (<SetCartesianPositionGoal-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCartesianPositionGoal-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCartesianPositionGoal-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<SetCartesianPositionGoal-request> is deprecated: use control_core_msgs-srv:SetCartesianPositionGoal-request instead.")))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <SetCartesianPositionGoal-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:goal-val is deprecated.  Use control_core_msgs-srv:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCartesianPositionGoal-request>) ostream)
  "Serializes a message object of type '<SetCartesianPositionGoal-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCartesianPositionGoal-request>) istream)
  "Deserializes a message object of type '<SetCartesianPositionGoal-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCartesianPositionGoal-request>)))
  "Returns string type for a service object of type '<SetCartesianPositionGoal-request>"
  "control_core_msgs/SetCartesianPositionGoalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianPositionGoal-request)))
  "Returns string type for a service object of type 'SetCartesianPositionGoal-request"
  "control_core_msgs/SetCartesianPositionGoalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCartesianPositionGoal-request>)))
  "Returns md5sum for a message object of type '<SetCartesianPositionGoal-request>"
  "313b76aa4f010582b3257488c62ac366")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCartesianPositionGoal-request)))
  "Returns md5sum for a message object of type 'SetCartesianPositionGoal-request"
  "313b76aa4f010582b3257488c62ac366")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCartesianPositionGoal-request>)))
  "Returns full string definition for message of type '<SetCartesianPositionGoal-request>"
  (cl:format cl:nil "geometry_msgs/Pose goal~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCartesianPositionGoal-request)))
  "Returns full string definition for message of type 'SetCartesianPositionGoal-request"
  (cl:format cl:nil "geometry_msgs/Pose goal~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCartesianPositionGoal-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCartesianPositionGoal-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCartesianPositionGoal-request
    (cl:cons ':goal (goal msg))
))
;//! \htmlinclude SetCartesianPositionGoal-response.msg.html

(cl:defclass <SetCartesianPositionGoal-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass SetCartesianPositionGoal-response (<SetCartesianPositionGoal-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetCartesianPositionGoal-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetCartesianPositionGoal-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<SetCartesianPositionGoal-response> is deprecated: use control_core_msgs-srv:SetCartesianPositionGoal-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetCartesianPositionGoal-response>) ostream)
  "Serializes a message object of type '<SetCartesianPositionGoal-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetCartesianPositionGoal-response>) istream)
  "Deserializes a message object of type '<SetCartesianPositionGoal-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetCartesianPositionGoal-response>)))
  "Returns string type for a service object of type '<SetCartesianPositionGoal-response>"
  "control_core_msgs/SetCartesianPositionGoalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianPositionGoal-response)))
  "Returns string type for a service object of type 'SetCartesianPositionGoal-response"
  "control_core_msgs/SetCartesianPositionGoalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetCartesianPositionGoal-response>)))
  "Returns md5sum for a message object of type '<SetCartesianPositionGoal-response>"
  "313b76aa4f010582b3257488c62ac366")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetCartesianPositionGoal-response)))
  "Returns md5sum for a message object of type 'SetCartesianPositionGoal-response"
  "313b76aa4f010582b3257488c62ac366")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetCartesianPositionGoal-response>)))
  "Returns full string definition for message of type '<SetCartesianPositionGoal-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetCartesianPositionGoal-response)))
  "Returns full string definition for message of type 'SetCartesianPositionGoal-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetCartesianPositionGoal-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetCartesianPositionGoal-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetCartesianPositionGoal-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetCartesianPositionGoal)))
  'SetCartesianPositionGoal-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetCartesianPositionGoal)))
  'SetCartesianPositionGoal-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetCartesianPositionGoal)))
  "Returns string type for a service object of type '<SetCartesianPositionGoal>"
  "control_core_msgs/SetCartesianPositionGoal")