; Auto-generated. Do not edit!


(cl:in-package ics_tsid_task_msgs-msg)


;//! \htmlinclude SkinForceConstraint.msg.html

(cl:defclass <SkinForceConstraint> (roslisp-msg-protocol:ros-message)
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
   (max_force
    :reader max_force
    :initarg :max_force
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (min_distance
    :reader min_distance
    :initarg :min_distance
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (max_proximity
    :reader max_proximity
    :initarg :max_proximity
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (cell_nums
    :reader cell_nums
    :initarg :cell_nums
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector))
   (forces
    :reader forces
    :initarg :forces
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector))
   (vel_cmds
    :reader vel_cmds
    :initarg :vel_cmds
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector))
   (acc_limits
    :reader acc_limits
    :initarg :acc_limits
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector))
   (weight
    :reader weight
    :initarg :weight
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (a_relaxed
    :reader a_relaxed
    :initarg :a_relaxed
    :type control_core_msgs-msg:Vector
    :initform (cl:make-instance 'control_core_msgs-msg:Vector)))
)

(cl:defclass SkinForceConstraint (<SkinForceConstraint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkinForceConstraint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkinForceConstraint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ics_tsid_task_msgs-msg:<SkinForceConstraint> is deprecated: use ics_tsid_task_msgs-msg:SkinForceConstraint instead.")))

(cl:ensure-generic-function 'dur-val :lambda-list '(m))
(cl:defmethod dur-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:dur-val is deprecated.  Use ics_tsid_task_msgs-msg:dur instead.")
  (dur m))

(cl:ensure-generic-function 'num_active_ieq-val :lambda-list '(m))
(cl:defmethod num_active_ieq-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:num_active_ieq-val is deprecated.  Use ics_tsid_task_msgs-msg:num_active_ieq instead.")
  (num_active_ieq m))

(cl:ensure-generic-function 'num_violated_ieq-val :lambda-list '(m))
(cl:defmethod num_violated_ieq-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:num_violated_ieq-val is deprecated.  Use ics_tsid_task_msgs-msg:num_violated_ieq instead.")
  (num_violated_ieq m))

(cl:ensure-generic-function 'max_force-val :lambda-list '(m))
(cl:defmethod max_force-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:max_force-val is deprecated.  Use ics_tsid_task_msgs-msg:max_force instead.")
  (max_force m))

(cl:ensure-generic-function 'min_distance-val :lambda-list '(m))
(cl:defmethod min_distance-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:min_distance-val is deprecated.  Use ics_tsid_task_msgs-msg:min_distance instead.")
  (min_distance m))

(cl:ensure-generic-function 'max_proximity-val :lambda-list '(m))
(cl:defmethod max_proximity-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:max_proximity-val is deprecated.  Use ics_tsid_task_msgs-msg:max_proximity instead.")
  (max_proximity m))

(cl:ensure-generic-function 'cell_nums-val :lambda-list '(m))
(cl:defmethod cell_nums-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:cell_nums-val is deprecated.  Use ics_tsid_task_msgs-msg:cell_nums instead.")
  (cell_nums m))

(cl:ensure-generic-function 'forces-val :lambda-list '(m))
(cl:defmethod forces-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:forces-val is deprecated.  Use ics_tsid_task_msgs-msg:forces instead.")
  (forces m))

(cl:ensure-generic-function 'vel_cmds-val :lambda-list '(m))
(cl:defmethod vel_cmds-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:vel_cmds-val is deprecated.  Use ics_tsid_task_msgs-msg:vel_cmds instead.")
  (vel_cmds m))

(cl:ensure-generic-function 'acc_limits-val :lambda-list '(m))
(cl:defmethod acc_limits-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:acc_limits-val is deprecated.  Use ics_tsid_task_msgs-msg:acc_limits instead.")
  (acc_limits m))

(cl:ensure-generic-function 'weight-val :lambda-list '(m))
(cl:defmethod weight-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:weight-val is deprecated.  Use ics_tsid_task_msgs-msg:weight instead.")
  (weight m))

(cl:ensure-generic-function 'a_relaxed-val :lambda-list '(m))
(cl:defmethod a_relaxed-val ((m <SkinForceConstraint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ics_tsid_task_msgs-msg:a_relaxed-val is deprecated.  Use ics_tsid_task_msgs-msg:a_relaxed instead.")
  (a_relaxed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkinForceConstraint>) ostream)
  "Serializes a message object of type '<SkinForceConstraint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dur) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'num_active_ieq) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'num_violated_ieq) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_distance) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_proximity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cell_nums) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'forces) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel_cmds) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc_limits) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'weight) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'a_relaxed) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkinForceConstraint>) istream)
  "Deserializes a message object of type '<SkinForceConstraint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dur) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'num_active_ieq) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'num_violated_ieq) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_distance) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_proximity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cell_nums) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'forces) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel_cmds) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc_limits) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'weight) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'a_relaxed) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkinForceConstraint>)))
  "Returns string type for a message object of type '<SkinForceConstraint>"
  "ics_tsid_task_msgs/SkinForceConstraint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkinForceConstraint)))
  "Returns string type for a message object of type 'SkinForceConstraint"
  "ics_tsid_task_msgs/SkinForceConstraint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkinForceConstraint>)))
  "Returns md5sum for a message object of type '<SkinForceConstraint>"
  "997ea5adb62ebb597e70c90a3e090602")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkinForceConstraint)))
  "Returns md5sum for a message object of type 'SkinForceConstraint"
  "997ea5adb62ebb597e70c90a3e090602")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkinForceConstraint>)))
  "Returns full string definition for message of type '<SkinForceConstraint>"
  (cl:format cl:nil "std_msgs/Int64 dur~%std_msgs/Int32 num_active_ieq~%std_msgs/Int32 num_violated_ieq~%~%std_msgs/Float64 max_force~%std_msgs/Float64 min_distance~%std_msgs/Float64 max_proximity~%~%control_core_msgs/Vector cell_nums~%control_core_msgs/Vector forces~%control_core_msgs/Vector vel_cmds~%control_core_msgs/Vector acc_limits~%~%std_msgs/Float64 weight~%control_core_msgs/Vector a_relaxed~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkinForceConstraint)))
  "Returns full string definition for message of type 'SkinForceConstraint"
  (cl:format cl:nil "std_msgs/Int64 dur~%std_msgs/Int32 num_active_ieq~%std_msgs/Int32 num_violated_ieq~%~%std_msgs/Float64 max_force~%std_msgs/Float64 min_distance~%std_msgs/Float64 max_proximity~%~%control_core_msgs/Vector cell_nums~%control_core_msgs/Vector forces~%control_core_msgs/Vector vel_cmds~%control_core_msgs/Vector acc_limits~%~%std_msgs/Float64 weight~%control_core_msgs/Vector a_relaxed~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Int32~%int32 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkinForceConstraint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dur))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'num_active_ieq))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'num_violated_ieq))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_distance))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_proximity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cell_nums))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'forces))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel_cmds))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc_limits))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'weight))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'a_relaxed))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkinForceConstraint>))
  "Converts a ROS message object to a list"
  (cl:list 'SkinForceConstraint
    (cl:cons ':dur (dur msg))
    (cl:cons ':num_active_ieq (num_active_ieq msg))
    (cl:cons ':num_violated_ieq (num_violated_ieq msg))
    (cl:cons ':max_force (max_force msg))
    (cl:cons ':min_distance (min_distance msg))
    (cl:cons ':max_proximity (max_proximity msg))
    (cl:cons ':cell_nums (cell_nums msg))
    (cl:cons ':forces (forces msg))
    (cl:cons ':vel_cmds (vel_cmds msg))
    (cl:cons ':acc_limits (acc_limits msg))
    (cl:cons ':weight (weight msg))
    (cl:cons ':a_relaxed (a_relaxed msg))
))
