; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude JointState.msg.html

(cl:defclass <JointState> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector))
   (velocity
    :reader velocity
    :initarg :velocity
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector)))
)

(cl:defclass JointState (<JointState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<JointState> is deprecated: use control_core_msgs-msg:JointState instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <JointState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:position-val is deprecated.  Use control_core_msgs-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <JointState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:velocity-val is deprecated.  Use control_core_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <JointState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:acceleration-val is deprecated.  Use control_core_msgs-msg:acceleration instead.")
  (acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointState>) ostream)
  "Serializes a message object of type '<JointState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointState>) istream)
  "Deserializes a message object of type '<JointState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointState>)))
  "Returns string type for a message object of type '<JointState>"
  "control_core_msgs/JointState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointState)))
  "Returns string type for a message object of type 'JointState"
  "control_core_msgs/JointState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointState>)))
  "Returns md5sum for a message object of type '<JointState>"
  "6682f84f321f6df0d77b7afd20d85702")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointState)))
  "Returns md5sum for a message object of type 'JointState"
  "6682f84f321f6df0d77b7afd20d85702")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointState>)))
  "Returns full string definition for message of type '<JointState>"
  (cl:format cl:nil "Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointState)))
  "Returns full string definition for message of type 'JointState"
  (cl:format cl:nil "Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointState>))
  "Converts a ROS message object to a list"
  (cl:list 'JointState
    (cl:cons ':position (position msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
))
