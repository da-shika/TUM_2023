; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude RobotState.msg.html

(cl:defclass <RobotState> (roslisp-msg-protocol:ros-message)
  ((joints
    :reader joints
    :initarg :joints
    :type control_core_msgs-msg:JointState
    :initform (cl:make-instance 'control_core_msgs-msg:JointState))
   (floating_base
    :reader floating_base
    :initarg :floating_base
    :type control_core_msgs-msg:CartesianState
    :initform (cl:make-instance 'control_core_msgs-msg:CartesianState))
   (imu
    :reader imu
    :initarg :imu
    :type sensor_msgs-msg:Imu
    :initform (cl:make-instance 'sensor_msgs-msg:Imu))
   (ft_sensors
    :reader ft_sensors
    :initarg :ft_sensors
    :type (cl:vector geometry_msgs-msg:WrenchStamped)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:WrenchStamped :initial-element (cl:make-instance 'geometry_msgs-msg:WrenchStamped)))
   (patches
    :reader patches
    :initarg :patches
    :type (cl:vector control_core_msgs-msg:SkinPatch)
   :initform (cl:make-array 0 :element-type 'control_core_msgs-msg:SkinPatch :initial-element (cl:make-instance 'control_core_msgs-msg:SkinPatch))))
)

(cl:defclass RobotState (<RobotState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<RobotState> is deprecated: use control_core_msgs-msg:RobotState instead.")))

(cl:ensure-generic-function 'joints-val :lambda-list '(m))
(cl:defmethod joints-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:joints-val is deprecated.  Use control_core_msgs-msg:joints instead.")
  (joints m))

(cl:ensure-generic-function 'floating_base-val :lambda-list '(m))
(cl:defmethod floating_base-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:floating_base-val is deprecated.  Use control_core_msgs-msg:floating_base instead.")
  (floating_base m))

(cl:ensure-generic-function 'imu-val :lambda-list '(m))
(cl:defmethod imu-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:imu-val is deprecated.  Use control_core_msgs-msg:imu instead.")
  (imu m))

(cl:ensure-generic-function 'ft_sensors-val :lambda-list '(m))
(cl:defmethod ft_sensors-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:ft_sensors-val is deprecated.  Use control_core_msgs-msg:ft_sensors instead.")
  (ft_sensors m))

(cl:ensure-generic-function 'patches-val :lambda-list '(m))
(cl:defmethod patches-val ((m <RobotState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:patches-val is deprecated.  Use control_core_msgs-msg:patches instead.")
  (patches m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotState>) ostream)
  "Serializes a message object of type '<RobotState>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'joints) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'floating_base) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'imu) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ft_sensors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'ft_sensors))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'patches))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'patches))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotState>) istream)
  "Deserializes a message object of type '<RobotState>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'joints) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'floating_base) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'imu) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ft_sensors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ft_sensors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:WrenchStamped))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotState>)))
  "Returns string type for a message object of type '<RobotState>"
  "control_core_msgs/RobotState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotState)))
  "Returns string type for a message object of type 'RobotState"
  "control_core_msgs/RobotState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotState>)))
  "Returns md5sum for a message object of type '<RobotState>"
  "c93ef624511e3a3594ee3528444b88cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotState)))
  "Returns md5sum for a message object of type 'RobotState"
  "c93ef624511e3a3594ee3528444b88cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotState>)))
  "Returns full string definition for message of type '<RobotState>"
  (cl:format cl:nil "JointState joints~%CartesianState floating_base~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped[] ft_sensors~%SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotState)))
  "Returns full string definition for message of type 'RobotState"
  (cl:format cl:nil "JointState joints~%CartesianState floating_base~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped[] ft_sensors~%SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotState>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'joints))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'floating_base))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'imu))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ft_sensors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'patches) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotState>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotState
    (cl:cons ':joints (joints msg))
    (cl:cons ':floating_base (floating_base msg))
    (cl:cons ':imu (imu msg))
    (cl:cons ':ft_sensors (ft_sensors msg))
    (cl:cons ':patches (patches msg))
))
