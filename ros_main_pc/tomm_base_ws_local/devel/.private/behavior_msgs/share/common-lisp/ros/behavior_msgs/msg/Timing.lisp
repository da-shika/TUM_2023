; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-msg)


;//! \htmlinclude Timing.msg.html

(cl:defclass <Timing> (roslisp-msg-protocol:ros-message)
  ((control_loop_dur
    :reader control_loop_dur
    :initarg :control_loop_dur
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (solver_dur
    :reader solver_dur
    :initarg :solver_dur
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (geometry_dur
    :reader geometry_dur
    :initarg :geometry_dur
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64)))
)

(cl:defclass Timing (<Timing>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Timing>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Timing)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-msg:<Timing> is deprecated: use behavior_msgs-msg:Timing instead.")))

(cl:ensure-generic-function 'control_loop_dur-val :lambda-list '(m))
(cl:defmethod control_loop_dur-val ((m <Timing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:control_loop_dur-val is deprecated.  Use behavior_msgs-msg:control_loop_dur instead.")
  (control_loop_dur m))

(cl:ensure-generic-function 'solver_dur-val :lambda-list '(m))
(cl:defmethod solver_dur-val ((m <Timing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:solver_dur-val is deprecated.  Use behavior_msgs-msg:solver_dur instead.")
  (solver_dur m))

(cl:ensure-generic-function 'geometry_dur-val :lambda-list '(m))
(cl:defmethod geometry_dur-val ((m <Timing>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:geometry_dur-val is deprecated.  Use behavior_msgs-msg:geometry_dur instead.")
  (geometry_dur m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Timing>) ostream)
  "Serializes a message object of type '<Timing>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'control_loop_dur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'solver_dur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'geometry_dur) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Timing>) istream)
  "Deserializes a message object of type '<Timing>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'control_loop_dur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'solver_dur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'geometry_dur) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Timing>)))
  "Returns string type for a message object of type '<Timing>"
  "behavior_msgs/Timing")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Timing)))
  "Returns string type for a message object of type 'Timing"
  "behavior_msgs/Timing")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Timing>)))
  "Returns md5sum for a message object of type '<Timing>"
  "860a4c96fe2b2a5b2000e2a96ca33c2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Timing)))
  "Returns md5sum for a message object of type 'Timing"
  "860a4c96fe2b2a5b2000e2a96ca33c2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Timing>)))
  "Returns full string definition for message of type '<Timing>"
  (cl:format cl:nil "std_msgs/Int64 control_loop_dur~%std_msgs/Int64 solver_dur~%std_msgs/Int64 geometry_dur~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Timing)))
  "Returns full string definition for message of type 'Timing"
  (cl:format cl:nil "std_msgs/Int64 control_loop_dur~%std_msgs/Int64 solver_dur~%std_msgs/Int64 geometry_dur~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Timing>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'control_loop_dur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'solver_dur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'geometry_dur))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Timing>))
  "Converts a ROS message object to a list"
  (cl:list 'Timing
    (cl:cons ':control_loop_dur (control_loop_dur msg))
    (cl:cons ':solver_dur (solver_dur msg))
    (cl:cons ':geometry_dur (geometry_dur msg))
))
