; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-msg)


;//! \htmlinclude BoxManipulation.msg.html

(cl:defclass <BoxManipulation> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (W_r_ref
    :reader W_r_ref
    :initarg :W_r_ref
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_l_ref
    :reader W_l_ref
    :initarg :W_l_ref
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_r_off
    :reader W_r_off
    :initarg :W_r_off
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_l_off
    :reader W_l_off
    :initarg :W_l_off
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_r_des
    :reader W_r_des
    :initarg :W_r_des
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_l_des
    :reader W_l_des
    :initarg :W_l_des
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_r_real_skin
    :reader W_r_real_skin
    :initarg :W_r_real_skin
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_l_real_skin
    :reader W_l_real_skin
    :initarg :W_l_real_skin
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_r_real_ft
    :reader W_r_real_ft
    :initarg :W_r_real_ft
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (W_l_real_ft
    :reader W_l_real_ft
    :initarg :W_l_real_ft
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench)))
)

(cl:defclass BoxManipulation (<BoxManipulation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoxManipulation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoxManipulation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-msg:<BoxManipulation> is deprecated: use behavior_msgs-msg:BoxManipulation instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:header-val is deprecated.  Use behavior_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:state-val is deprecated.  Use behavior_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'W_r_ref-val :lambda-list '(m))
(cl:defmethod W_r_ref-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_r_ref-val is deprecated.  Use behavior_msgs-msg:W_r_ref instead.")
  (W_r_ref m))

(cl:ensure-generic-function 'W_l_ref-val :lambda-list '(m))
(cl:defmethod W_l_ref-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_l_ref-val is deprecated.  Use behavior_msgs-msg:W_l_ref instead.")
  (W_l_ref m))

(cl:ensure-generic-function 'W_r_off-val :lambda-list '(m))
(cl:defmethod W_r_off-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_r_off-val is deprecated.  Use behavior_msgs-msg:W_r_off instead.")
  (W_r_off m))

(cl:ensure-generic-function 'W_l_off-val :lambda-list '(m))
(cl:defmethod W_l_off-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_l_off-val is deprecated.  Use behavior_msgs-msg:W_l_off instead.")
  (W_l_off m))

(cl:ensure-generic-function 'W_r_des-val :lambda-list '(m))
(cl:defmethod W_r_des-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_r_des-val is deprecated.  Use behavior_msgs-msg:W_r_des instead.")
  (W_r_des m))

(cl:ensure-generic-function 'W_l_des-val :lambda-list '(m))
(cl:defmethod W_l_des-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_l_des-val is deprecated.  Use behavior_msgs-msg:W_l_des instead.")
  (W_l_des m))

(cl:ensure-generic-function 'W_r_real_skin-val :lambda-list '(m))
(cl:defmethod W_r_real_skin-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_r_real_skin-val is deprecated.  Use behavior_msgs-msg:W_r_real_skin instead.")
  (W_r_real_skin m))

(cl:ensure-generic-function 'W_l_real_skin-val :lambda-list '(m))
(cl:defmethod W_l_real_skin-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_l_real_skin-val is deprecated.  Use behavior_msgs-msg:W_l_real_skin instead.")
  (W_l_real_skin m))

(cl:ensure-generic-function 'W_r_real_ft-val :lambda-list '(m))
(cl:defmethod W_r_real_ft-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_r_real_ft-val is deprecated.  Use behavior_msgs-msg:W_r_real_ft instead.")
  (W_r_real_ft m))

(cl:ensure-generic-function 'W_l_real_ft-val :lambda-list '(m))
(cl:defmethod W_l_real_ft-val ((m <BoxManipulation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-msg:W_l_real_ft-val is deprecated.  Use behavior_msgs-msg:W_l_real_ft instead.")
  (W_l_real_ft m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoxManipulation>) ostream)
  "Serializes a message object of type '<BoxManipulation>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_r_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_l_ref) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_r_off) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_l_off) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_r_des) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_l_des) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_r_real_skin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_l_real_skin) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_r_real_ft) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'W_l_real_ft) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoxManipulation>) istream)
  "Deserializes a message object of type '<BoxManipulation>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_r_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_l_ref) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_r_off) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_l_off) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_r_des) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_l_des) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_r_real_skin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_l_real_skin) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_r_real_ft) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'W_l_real_ft) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoxManipulation>)))
  "Returns string type for a message object of type '<BoxManipulation>"
  "behavior_msgs/BoxManipulation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoxManipulation)))
  "Returns string type for a message object of type 'BoxManipulation"
  "behavior_msgs/BoxManipulation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoxManipulation>)))
  "Returns md5sum for a message object of type '<BoxManipulation>"
  "1764b3b821c4db67f223b0ac4257c2d9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoxManipulation)))
  "Returns md5sum for a message object of type 'BoxManipulation"
  "1764b3b821c4db67f223b0ac4257c2d9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoxManipulation>)))
  "Returns full string definition for message of type '<BoxManipulation>"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 state~%~%geometry_msgs/Wrench W_r_ref~%geometry_msgs/Wrench W_l_ref~%~%geometry_msgs/Wrench W_r_off~%geometry_msgs/Wrench W_l_off~%~%geometry_msgs/Wrench W_r_des~%geometry_msgs/Wrench W_l_des~%~%geometry_msgs/Wrench W_r_real_skin~%geometry_msgs/Wrench W_l_real_skin~%~%geometry_msgs/Wrench W_r_real_ft~%geometry_msgs/Wrench W_l_real_ft~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoxManipulation)))
  "Returns full string definition for message of type 'BoxManipulation"
  (cl:format cl:nil "std_msgs/Header header~%~%uint8 state~%~%geometry_msgs/Wrench W_r_ref~%geometry_msgs/Wrench W_l_ref~%~%geometry_msgs/Wrench W_r_off~%geometry_msgs/Wrench W_l_off~%~%geometry_msgs/Wrench W_r_des~%geometry_msgs/Wrench W_l_des~%~%geometry_msgs/Wrench W_r_real_skin~%geometry_msgs/Wrench W_l_real_skin~%~%geometry_msgs/Wrench W_r_real_ft~%geometry_msgs/Wrench W_l_real_ft~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoxManipulation>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_r_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_l_ref))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_r_off))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_l_off))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_r_des))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_l_des))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_r_real_skin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_l_real_skin))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_r_real_ft))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'W_l_real_ft))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoxManipulation>))
  "Converts a ROS message object to a list"
  (cl:list 'BoxManipulation
    (cl:cons ':header (header msg))
    (cl:cons ':state (state msg))
    (cl:cons ':W_r_ref (W_r_ref msg))
    (cl:cons ':W_l_ref (W_l_ref msg))
    (cl:cons ':W_r_off (W_r_off msg))
    (cl:cons ':W_l_off (W_l_off msg))
    (cl:cons ':W_r_des (W_r_des msg))
    (cl:cons ':W_l_des (W_l_des msg))
    (cl:cons ':W_r_real_skin (W_r_real_skin msg))
    (cl:cons ':W_l_real_skin (W_l_real_skin msg))
    (cl:cons ':W_r_real_ft (W_r_real_ft msg))
    (cl:cons ':W_l_real_ft (W_l_real_ft msg))
))
