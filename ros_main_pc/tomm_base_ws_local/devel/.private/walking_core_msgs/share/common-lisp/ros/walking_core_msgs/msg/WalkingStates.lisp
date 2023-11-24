; Auto-generated. Do not edit!


(cl:in-package walking_core_msgs-msg)


;//! \htmlinclude WalkingStates.msg.html

(cl:defclass <WalkingStates> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (phase
    :reader phase
    :initarg :phase
    :type walking_core_msgs-msg:WalkingPhase
    :initform (cl:make-instance 'walking_core_msgs-msg:WalkingPhase))
   (stance_foot
    :reader stance_foot
    :initarg :stance_foot
    :type control_core_msgs-msg:BodyId
    :initform (cl:make-instance 'control_core_msgs-msg:BodyId))
   (swing_foot
    :reader swing_foot
    :initarg :swing_foot
    :type control_core_msgs-msg:BodyId
    :initform (cl:make-instance 'control_core_msgs-msg:BodyId))
   (elapsed
    :reader elapsed
    :initarg :elapsed
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (left_foot_ratio
    :reader left_foot_ratio
    :initarg :left_foot_ratio
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (cur_step
    :reader cur_step
    :initarg :cur_step
    :type walking_core_msgs-msg:FootStep
    :initform (cl:make-instance 'walking_core_msgs-msg:FootStep))
   (target_step
    :reader target_step
    :initarg :target_step
    :type walking_core_msgs-msg:FootStep
    :initform (cl:make-instance 'walking_core_msgs-msg:FootStep))
   (next_step
    :reader next_step
    :initarg :next_step
    :type walking_core_msgs-msg:FootStep
    :initform (cl:make-instance 'walking_core_msgs-msg:FootStep)))
)

(cl:defclass WalkingStates (<WalkingStates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WalkingStates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WalkingStates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walking_core_msgs-msg:<WalkingStates> is deprecated: use walking_core_msgs-msg:WalkingStates instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:header-val is deprecated.  Use walking_core_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'phase-val :lambda-list '(m))
(cl:defmethod phase-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:phase-val is deprecated.  Use walking_core_msgs-msg:phase instead.")
  (phase m))

(cl:ensure-generic-function 'stance_foot-val :lambda-list '(m))
(cl:defmethod stance_foot-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:stance_foot-val is deprecated.  Use walking_core_msgs-msg:stance_foot instead.")
  (stance_foot m))

(cl:ensure-generic-function 'swing_foot-val :lambda-list '(m))
(cl:defmethod swing_foot-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:swing_foot-val is deprecated.  Use walking_core_msgs-msg:swing_foot instead.")
  (swing_foot m))

(cl:ensure-generic-function 'elapsed-val :lambda-list '(m))
(cl:defmethod elapsed-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:elapsed-val is deprecated.  Use walking_core_msgs-msg:elapsed instead.")
  (elapsed m))

(cl:ensure-generic-function 'left_foot_ratio-val :lambda-list '(m))
(cl:defmethod left_foot_ratio-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:left_foot_ratio-val is deprecated.  Use walking_core_msgs-msg:left_foot_ratio instead.")
  (left_foot_ratio m))

(cl:ensure-generic-function 'cur_step-val :lambda-list '(m))
(cl:defmethod cur_step-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:cur_step-val is deprecated.  Use walking_core_msgs-msg:cur_step instead.")
  (cur_step m))

(cl:ensure-generic-function 'target_step-val :lambda-list '(m))
(cl:defmethod target_step-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:target_step-val is deprecated.  Use walking_core_msgs-msg:target_step instead.")
  (target_step m))

(cl:ensure-generic-function 'next_step-val :lambda-list '(m))
(cl:defmethod next_step-val ((m <WalkingStates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:next_step-val is deprecated.  Use walking_core_msgs-msg:next_step instead.")
  (next_step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WalkingStates>) ostream)
  "Serializes a message object of type '<WalkingStates>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'phase) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'stance_foot) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'swing_foot) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'elapsed) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'left_foot_ratio) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cur_step) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'target_step) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'next_step) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WalkingStates>) istream)
  "Deserializes a message object of type '<WalkingStates>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'phase) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'stance_foot) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'swing_foot) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'elapsed) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'left_foot_ratio) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cur_step) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'target_step) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'next_step) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WalkingStates>)))
  "Returns string type for a message object of type '<WalkingStates>"
  "walking_core_msgs/WalkingStates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WalkingStates)))
  "Returns string type for a message object of type 'WalkingStates"
  "walking_core_msgs/WalkingStates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WalkingStates>)))
  "Returns md5sum for a message object of type '<WalkingStates>"
  "e12756d011ab43b51ff2b08d69aadcb4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WalkingStates)))
  "Returns md5sum for a message object of type 'WalkingStates"
  "e12756d011ab43b51ff2b08d69aadcb4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WalkingStates>)))
  "Returns full string definition for message of type '<WalkingStates>"
  (cl:format cl:nil "std_msgs/Header header~%WalkingPhase phase~%control_core_msgs/BodyId stance_foot~%control_core_msgs/BodyId swing_foot~%std_msgs/Float64 elapsed~%std_msgs/Float64 left_foot_ratio~%FootStep cur_step~%FootStep target_step~%FootStep next_step~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: walking_core_msgs/WalkingPhase~%uint8 PHASE_STANCE=0~%uint8 PHASE_DOUBLESUPPORT=1~%uint8 PHASE_SINGLESUPPORT=2~%uint8 phase~%================================================================================~%MSG: control_core_msgs/BodyId~%uint8 ID_LEFT_FOOT=0~%uint8 ID_RIGHT_FOOT=1~%uint8 ID_LEFT_HAND=2~%uint8 ID_RIGHT_HAND=3~%uint8 id~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: walking_core_msgs/FootStep~%control_core_msgs/Contact contact~%std_msgs/Int64 body_id~%std_msgs/Bool final_step~%std_msgs/Int64 n_step~%================================================================================~%MSG: control_core_msgs/Contact~%geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WalkingStates)))
  "Returns full string definition for message of type 'WalkingStates"
  (cl:format cl:nil "std_msgs/Header header~%WalkingPhase phase~%control_core_msgs/BodyId stance_foot~%control_core_msgs/BodyId swing_foot~%std_msgs/Float64 elapsed~%std_msgs/Float64 left_foot_ratio~%FootStep cur_step~%FootStep target_step~%FootStep next_step~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: walking_core_msgs/WalkingPhase~%uint8 PHASE_STANCE=0~%uint8 PHASE_DOUBLESUPPORT=1~%uint8 PHASE_SINGLESUPPORT=2~%uint8 phase~%================================================================================~%MSG: control_core_msgs/BodyId~%uint8 ID_LEFT_FOOT=0~%uint8 ID_RIGHT_FOOT=1~%uint8 ID_LEFT_HAND=2~%uint8 ID_RIGHT_HAND=3~%uint8 id~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: walking_core_msgs/FootStep~%control_core_msgs/Contact contact~%std_msgs/Int64 body_id~%std_msgs/Bool final_step~%std_msgs/Int64 n_step~%================================================================================~%MSG: control_core_msgs/Contact~%geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WalkingStates>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'phase))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'stance_foot))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'swing_foot))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'elapsed))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'left_foot_ratio))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cur_step))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'target_step))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'next_step))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WalkingStates>))
  "Converts a ROS message object to a list"
  (cl:list 'WalkingStates
    (cl:cons ':header (header msg))
    (cl:cons ':phase (phase msg))
    (cl:cons ':stance_foot (stance_foot msg))
    (cl:cons ':swing_foot (swing_foot msg))
    (cl:cons ':elapsed (elapsed msg))
    (cl:cons ':left_foot_ratio (left_foot_ratio msg))
    (cl:cons ':cur_step (cur_step msg))
    (cl:cons ':target_step (target_step msg))
    (cl:cons ':next_step (next_step msg))
))
