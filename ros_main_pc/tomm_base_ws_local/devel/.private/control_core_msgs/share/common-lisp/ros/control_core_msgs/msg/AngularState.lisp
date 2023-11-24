; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude AngularState.msg.html

(cl:defclass <AngularState> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
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

(cl:defclass AngularState (<AngularState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AngularState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AngularState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<AngularState> is deprecated: use control_core_msgs-msg:AngularState instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <AngularState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:position-val is deprecated.  Use control_core_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <AngularState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:velocity-val is deprecated.  Use control_core_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <AngularState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:acceleration-val is deprecated.  Use control_core_msgs-msg:acceleration instead.")
  (acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AngularState>) ostream)
  "Serializes a message object of type '<AngularState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AngularState>) istream)
  "Deserializes a message object of type '<AngularState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AngularState>)))
  "Returns string type for a message object of type '<AngularState>"
  "control_core_msgs/AngularState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AngularState)))
  "Returns string type for a message object of type 'AngularState"
  "control_core_msgs/AngularState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AngularState>)))
  "Returns md5sum for a message object of type '<AngularState>"
  "7e1d5f76ab83eb7ee7b25ee90215e8d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AngularState)))
  "Returns md5sum for a message object of type 'AngularState"
  "7e1d5f76ab83eb7ee7b25ee90215e8d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AngularState>)))
  "Returns full string definition for message of type '<AngularState>"
  (cl:format cl:nil "geometry_msgs/Quaternion position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AngularState)))
  "Returns full string definition for message of type 'AngularState"
  (cl:format cl:nil "geometry_msgs/Quaternion position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AngularState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AngularState>))
  "Converts a ROS message object to a list"
  (cl:list 'AngularState
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
))
