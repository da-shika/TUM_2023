; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude SkinPatches.msg.html

(cl:defclass <SkinPatches> (roslisp-msg-protocol:ros-message)
  ((patches
    :reader patches
    :initarg :patches
    :type (cl:vector control_core_msgs-msg:SkinPatch)
   :initform (cl:make-array 0 :element-type 'control_core_msgs-msg:SkinPatch :initial-element (cl:make-instance 'control_core_msgs-msg:SkinPatch))))
)

(cl:defclass SkinPatches (<SkinPatches>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkinPatches>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkinPatches)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<SkinPatches> is deprecated: use control_core_msgs-msg:SkinPatches instead.")))

(cl:ensure-generic-function 'patches-val :lambda-list '(m))
(cl:defmethod patches-val ((m <SkinPatches>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:patches-val is deprecated.  Use control_core_msgs-msg:patches instead.")
  (patches m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkinPatches>) ostream)
  "Serializes a message object of type '<SkinPatches>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'patches))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'patches))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkinPatches>) istream)
  "Deserializes a message object of type '<SkinPatches>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'patches) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'patches)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'control_core_msgs-msg:SkinPatch))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkinPatches>)))
  "Returns string type for a message object of type '<SkinPatches>"
  "control_core_msgs/SkinPatches")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkinPatches)))
  "Returns string type for a message object of type 'SkinPatches"
  "control_core_msgs/SkinPatches")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkinPatches>)))
  "Returns md5sum for a message object of type '<SkinPatches>"
  "54325af118430bc2db92b5f684606b52")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkinPatches)))
  "Returns md5sum for a message object of type 'SkinPatches"
  "54325af118430bc2db92b5f684606b52")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkinPatches>)))
  "Returns full string definition for message of type '<SkinPatches>"
  (cl:format cl:nil "SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkinPatches)))
  "Returns full string definition for message of type 'SkinPatches"
  (cl:format cl:nil "SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkinPatches>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'patches) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkinPatches>))
  "Converts a ROS message object to a list"
  (cl:list 'SkinPatches
    (cl:cons ':patches (patches msg))
))
