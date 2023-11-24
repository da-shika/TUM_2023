; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude LinearState.msg.html

(cl:defclass <LinearState> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass LinearState (<LinearState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LinearState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LinearState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<LinearState> is deprecated: use control_core_msgs-msg:LinearState instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <LinearState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:position-val is deprecated.  Use control_core_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <LinearState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:velocity-val is deprecated.  Use control_core_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <LinearState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:acceleration-val is deprecated.  Use control_core_msgs-msg:acceleration instead.")
  (acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LinearState>) ostream)
  "Serializes a message object of type '<LinearState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LinearState>) istream)
  "Deserializes a message object of type '<LinearState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LinearState>)))
  "Returns string type for a message object of type '<LinearState>"
  "control_core_msgs/LinearState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LinearState)))
  "Returns string type for a message object of type 'LinearState"
  "control_core_msgs/LinearState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LinearState>)))
  "Returns md5sum for a message object of type '<LinearState>"
  "b6bed245f19156eb5ee582c6cf27c090")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LinearState)))
  "Returns md5sum for a message object of type 'LinearState"
  "b6bed245f19156eb5ee582c6cf27c090")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LinearState>)))
  "Returns full string definition for message of type '<LinearState>"
  (cl:format cl:nil "geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LinearState)))
  "Returns full string definition for message of type 'LinearState"
  (cl:format cl:nil "geometry_msgs/Point position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LinearState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LinearState>))
  "Converts a ROS message object to a list"
  (cl:list 'LinearState
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
))
