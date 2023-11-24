; Auto-generated. Do not edit!


(cl:in-package walking_core_msgs-msg)


;//! \htmlinclude FootSteps.msg.html

(cl:defclass <FootSteps> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (footsteps
    :reader footsteps
    :initarg :footsteps
    :type (cl:vector walking_core_msgs-msg:FootStep)
   :initform (cl:make-array 0 :element-type 'walking_core_msgs-msg:FootStep :initial-element (cl:make-instance 'walking_core_msgs-msg:FootStep))))
)

(cl:defclass FootSteps (<FootSteps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootSteps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootSteps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walking_core_msgs-msg:<FootSteps> is deprecated: use walking_core_msgs-msg:FootSteps instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FootSteps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:header-val is deprecated.  Use walking_core_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'footsteps-val :lambda-list '(m))
(cl:defmethod footsteps-val ((m <FootSteps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:footsteps-val is deprecated.  Use walking_core_msgs-msg:footsteps instead.")
  (footsteps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootSteps>) ostream)
  "Serializes a message object of type '<FootSteps>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'footsteps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'footsteps))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootSteps>) istream)
  "Deserializes a message object of type '<FootSteps>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'footsteps) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'footsteps)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'walking_core_msgs-msg:FootStep))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootSteps>)))
  "Returns string type for a message object of type '<FootSteps>"
  "walking_core_msgs/FootSteps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootSteps)))
  "Returns string type for a message object of type 'FootSteps"
  "walking_core_msgs/FootSteps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootSteps>)))
  "Returns md5sum for a message object of type '<FootSteps>"
  "afa335a6af8741fb5411d66b054528e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootSteps)))
  "Returns md5sum for a message object of type 'FootSteps"
  "afa335a6af8741fb5411d66b054528e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootSteps>)))
  "Returns full string definition for message of type '<FootSteps>"
  (cl:format cl:nil "std_msgs/Header header~%FootStep[] footsteps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: walking_core_msgs/FootStep~%control_core_msgs/Contact contact~%std_msgs/Int64 body_id~%std_msgs/Bool final_step~%std_msgs/Int64 n_step~%================================================================================~%MSG: control_core_msgs/Contact~%geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootSteps)))
  "Returns full string definition for message of type 'FootSteps"
  (cl:format cl:nil "std_msgs/Header header~%FootStep[] footsteps~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: walking_core_msgs/FootStep~%control_core_msgs/Contact contact~%std_msgs/Int64 body_id~%std_msgs/Bool final_step~%std_msgs/Int64 n_step~%================================================================================~%MSG: control_core_msgs/Contact~%geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootSteps>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'footsteps) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootSteps>))
  "Converts a ROS message object to a list"
  (cl:list 'FootSteps
    (cl:cons ':header (header msg))
    (cl:cons ':footsteps (footsteps msg))
))
