; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude SkinPatch.msg.html

(cl:defclass <SkinPatch> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (force
    :reader force
    :initarg :force
    :type control_core_msgs-msg:SkinModality
    :initform (cl:make-instance 'control_core_msgs-msg:SkinModality))
   (proximity
    :reader proximity
    :initarg :proximity
    :type control_core_msgs-msg:SkinModality
    :initform (cl:make-instance 'control_core_msgs-msg:SkinModality))
   (min_dist
    :reader min_dist
    :initarg :min_dist
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (max_dist
    :reader max_dist
    :initarg :max_dist
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass SkinPatch (<SkinPatch>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkinPatch>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkinPatch)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<SkinPatch> is deprecated: use control_core_msgs-msg:SkinPatch instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <SkinPatch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:header-val is deprecated.  Use control_core_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <SkinPatch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:pose-val is deprecated.  Use control_core_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <SkinPatch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:force-val is deprecated.  Use control_core_msgs-msg:force instead.")
  (force m))

(cl:ensure-generic-function 'proximity-val :lambda-list '(m))
(cl:defmethod proximity-val ((m <SkinPatch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:proximity-val is deprecated.  Use control_core_msgs-msg:proximity instead.")
  (proximity m))

(cl:ensure-generic-function 'min_dist-val :lambda-list '(m))
(cl:defmethod min_dist-val ((m <SkinPatch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:min_dist-val is deprecated.  Use control_core_msgs-msg:min_dist instead.")
  (min_dist m))

(cl:ensure-generic-function 'max_dist-val :lambda-list '(m))
(cl:defmethod max_dist-val ((m <SkinPatch>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:max_dist-val is deprecated.  Use control_core_msgs-msg:max_dist instead.")
  (max_dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkinPatch>) ostream)
  "Serializes a message object of type '<SkinPatch>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'proximity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_dist) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_dist) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkinPatch>) istream)
  "Deserializes a message object of type '<SkinPatch>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'proximity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_dist) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_dist) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkinPatch>)))
  "Returns string type for a message object of type '<SkinPatch>"
  "control_core_msgs/SkinPatch")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkinPatch)))
  "Returns string type for a message object of type 'SkinPatch"
  "control_core_msgs/SkinPatch")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkinPatch>)))
  "Returns md5sum for a message object of type '<SkinPatch>"
  "a5c4ba5fbedd1c63abbdd1a1450dda87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkinPatch)))
  "Returns md5sum for a message object of type 'SkinPatch"
  "a5c4ba5fbedd1c63abbdd1a1450dda87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkinPatch>)))
  "Returns full string definition for message of type '<SkinPatch>"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkinPatch)))
  "Returns full string definition for message of type 'SkinPatch"
  (cl:format cl:nil "std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkinPatch>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'proximity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_dist))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_dist))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkinPatch>))
  "Converts a ROS message object to a list"
  (cl:list 'SkinPatch
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':force (force msg))
    (cl:cons ':proximity (proximity msg))
    (cl:cons ':min_dist (min_dist msg))
    (cl:cons ':max_dist (max_dist msg))
))
