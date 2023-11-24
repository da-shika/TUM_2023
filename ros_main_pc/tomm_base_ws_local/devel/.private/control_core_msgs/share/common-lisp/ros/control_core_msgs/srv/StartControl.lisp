; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-srv)


;//! \htmlinclude StartControl-request.msg.html

(cl:defclass <StartControl-request> (roslisp-msg-protocol:ros-message)
  ((CONNECTION_STATE_INIT
    :reader CONNECTION_STATE_INIT
    :initarg :CONNECTION_STATE_INIT
    :type cl:fixnum
    :initform 0)
   (CONNECTION_STATE_START
    :reader CONNECTION_STATE_START
    :initarg :CONNECTION_STATE_START
    :type cl:fixnum
    :initform 0)
   (connection_state
    :reader connection_state
    :initarg :connection_state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StartControl-request (<StartControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<StartControl-request> is deprecated: use control_core_msgs-srv:StartControl-request instead.")))

(cl:ensure-generic-function 'CONNECTION_STATE_INIT-val :lambda-list '(m))
(cl:defmethod CONNECTION_STATE_INIT-val ((m <StartControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:CONNECTION_STATE_INIT-val is deprecated.  Use control_core_msgs-srv:CONNECTION_STATE_INIT instead.")
  (CONNECTION_STATE_INIT m))

(cl:ensure-generic-function 'CONNECTION_STATE_START-val :lambda-list '(m))
(cl:defmethod CONNECTION_STATE_START-val ((m <StartControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:CONNECTION_STATE_START-val is deprecated.  Use control_core_msgs-srv:CONNECTION_STATE_START instead.")
  (CONNECTION_STATE_START m))

(cl:ensure-generic-function 'connection_state-val :lambda-list '(m))
(cl:defmethod connection_state-val ((m <StartControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:connection_state-val is deprecated.  Use control_core_msgs-srv:connection_state instead.")
  (connection_state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartControl-request>) ostream)
  "Serializes a message object of type '<StartControl-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CONNECTION_STATE_INIT)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CONNECTION_STATE_START)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'connection_state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartControl-request>) istream)
  "Deserializes a message object of type '<StartControl-request>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CONNECTION_STATE_INIT)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'CONNECTION_STATE_START)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'connection_state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartControl-request>)))
  "Returns string type for a service object of type '<StartControl-request>"
  "control_core_msgs/StartControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartControl-request)))
  "Returns string type for a service object of type 'StartControl-request"
  "control_core_msgs/StartControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartControl-request>)))
  "Returns md5sum for a message object of type '<StartControl-request>"
  "adb9278d966383f41d079c269f203a93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartControl-request)))
  "Returns md5sum for a message object of type 'StartControl-request"
  "adb9278d966383f41d079c269f203a93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartControl-request>)))
  "Returns full string definition for message of type '<StartControl-request>"
  (cl:format cl:nil "uint8 CONNECTION_STATE_INIT~%uint8 CONNECTION_STATE_START~%uint8 connection_state          # connection state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartControl-request)))
  "Returns full string definition for message of type 'StartControl-request"
  (cl:format cl:nil "uint8 CONNECTION_STATE_INIT~%uint8 CONNECTION_STATE_START~%uint8 connection_state          # connection state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartControl-request>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'StartControl-request
    (cl:cons ':CONNECTION_STATE_INIT (CONNECTION_STATE_INIT msg))
    (cl:cons ':CONNECTION_STATE_START (CONNECTION_STATE_START msg))
    (cl:cons ':connection_state (connection_state msg))
))
;//! \htmlinclude StartControl-response.msg.html

(cl:defclass <StartControl-response> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type control_core_msgs-msg:RobotState
    :initform (cl:make-instance 'control_core_msgs-msg:RobotState)))
)

(cl:defclass StartControl-response (<StartControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StartControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StartControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<StartControl-response> is deprecated: use control_core_msgs-srv:StartControl-response instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <StartControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:state-val is deprecated.  Use control_core_msgs-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StartControl-response>) ostream)
  "Serializes a message object of type '<StartControl-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StartControl-response>) istream)
  "Deserializes a message object of type '<StartControl-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StartControl-response>)))
  "Returns string type for a service object of type '<StartControl-response>"
  "control_core_msgs/StartControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartControl-response)))
  "Returns string type for a service object of type 'StartControl-response"
  "control_core_msgs/StartControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StartControl-response>)))
  "Returns md5sum for a message object of type '<StartControl-response>"
  "adb9278d966383f41d079c269f203a93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StartControl-response)))
  "Returns md5sum for a message object of type 'StartControl-response"
  "adb9278d966383f41d079c269f203a93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StartControl-response>)))
  "Returns full string definition for message of type '<StartControl-response>"
  (cl:format cl:nil "RobotState state                # robot state~%~%================================================================================~%MSG: control_core_msgs/RobotState~%JointState joints~%CartesianState floating_base~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped[] ft_sensors~%SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StartControl-response)))
  "Returns full string definition for message of type 'StartControl-response"
  (cl:format cl:nil "RobotState state                # robot state~%~%================================================================================~%MSG: control_core_msgs/RobotState~%JointState joints~%CartesianState floating_base~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped[] ft_sensors~%SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StartControl-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StartControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'StartControl-response
    (cl:cons ':state (state msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'StartControl)))
  'StartControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'StartControl)))
  'StartControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StartControl)))
  "Returns string type for a service object of type '<StartControl>"
  "control_core_msgs/StartControl")