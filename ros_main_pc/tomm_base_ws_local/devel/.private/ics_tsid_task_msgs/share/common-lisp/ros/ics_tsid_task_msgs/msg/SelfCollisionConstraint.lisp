; Auto-generated. Do not edit!


(cl:in-package ics_tsid_task_msgs-msg)


;//! \htmlinclude SelfCollisionConstraint.msg.html

(cl:defclass <SelfCollisionConstraint> (roslisp-msg-protocol:ros-message)
  ((dur
    :reader dur
    :initarg :dur
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (num_active_ieq
    :reader num_active_ieq
    :initarg :num_active_ieq
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (num_violated_ieq
    :reader num_violated_ieq
    :initarg :num_violated_ieq
    :type std_msgs-msg:Int32
    :initform (cl:make-instance 'std_msgs-msg:Int32))
   (min_distance
    :reader min_distance
    :initarg :min_distance
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (distances
    :reader distances
    :initarg :distances
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector)))
)

(cl:defclass SelfCollisionConstraint (<SelfCollisionConstraint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SelfCollisionConstraint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SelfCollisionConstraint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ics_tsid_task_msgs-msg:<SelfCollisionConstraint> is deprecated: use ics_tsid_task_msgs-msg:SelfCollisionConstraint instead.")))

(cl:ensure-generic-function 'dur-val :lambda-list '(m))
(cl:defmethod dur-val ((m <SelfCollisionConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:dur-val is deprecated.  Use ics_tsid_task_msgs-msg:dur instead.")
  (dur m))

(cl:ensure-generic-function 'num_active_ieq-val :lambda-list '(m))
(cl:defmethod num_active_ieq-val ((m <SelfCollisionConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:num_active_ieq-val is deprecated.  Use ics_tsid_task_msgs-msg:num_active_ieq instead.")
  (num_active_ieq m))

(cl:ensure-generic-function 'num_violated_ieq-val :lambda-list '(m))
(cl:defmethod num_violated_ieq-val ((m <SelfCollisionConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:num_violated_ieq-val is deprecated.  Use ics_tsid_task_msgs-msg:num_violated_ieq instead.")
  (num_violated_ieq m))

(cl:ensure-generic-function 'min_distance-val :lambda-list '(m))
(cl:defmethod min_distance-val ((m <SelfCollisionConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:min_distance-val is deprecated.  Use ics_tsid_task_msgs-msg:min_distance instead.")
  (min_distance m))

(cl:ensure-generic-function 'distances-val :lambda-list '(m))
(cl:defmethod distances-val ((m <SelfCollisionConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:distances-val is deprecated.  Use ics_tsid_task_msgs-msg:distances instead.")
  (distances m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SelfCollisionConstraint>) ostream)
  "Serializes a message object of type '<SelfCollisionConstraint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'num_active_ieq) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'num_violated_ieq) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_distance) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'distances) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SelfCollisionConstraint>) istream)
  "Deserializes a message object of type '<SelfCollisionConstraint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'num_active_ieq) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'num_violated_ieq) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_distance) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'distances) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SelfCollisionConstraint>)))
  "Returns string type for a message object of type '<SelfCollisionConstraint>"
  "ics_tsid_task_msgs/SelfCollisionConstraint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SelfCollisionConstraint)))
  "Returns string type for a message object of type 'SelfCollisionConstraint"
  "ics_tsid_task_msgs/SelfCollisionConstraint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SelfCollisionConstraint>)))
  "Returns md5sum for a message object of type '<SelfCollisionConstraint>"
  "f09e82fd510858f1c3f5993df0a68ab2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SelfCollisionConstraint)))
  "Returns md5sum for a message object of type 'SelfCollisionConstraint"
  "f09e82fd510858f1c3f5993df0a68ab2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SelfCollisionConstraint>)))
  "Returns full string definition for message of type '<SelfCollisionConstraint>"
  (cl:format cl:nil "std_msgs/Int64 dur~%std_msgs/Int32 num_active_ieq~%std_msgs/Int32 num_violated_ieq~%~%std_msgs/Float64 min_distance~%control_core_msgs/Vector distances~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SelfCollisionConstraint)))
  "Returns full string definition for message of type 'SelfCollisionConstraint"
  (cl:format cl:nil "std_msgs/Int64 dur~%std_msgs/Int32 num_active_ieq~%std_msgs/Int32 num_violated_ieq~%~%std_msgs/Float64 min_distance~%control_core_msgs/Vector distances~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SelfCollisionConstraint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'num_active_ieq))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'num_violated_ieq))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_distance))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'distances))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SelfCollisionConstraint>))
  "Converts a ROS message object to a list"
  (cl:list 'SelfCollisionConstraint
    (cl:cons ':dur (dur msg))
    (cl:cons ':num_active_ieq (num_active_ieq msg))
    (cl:cons ':num_violated_ieq (num_violated_ieq msg))
    (cl:cons ':min_distance (min_distance msg))
    (cl:cons ':distances (distances msg))
))
