; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-srv)


;//! \htmlinclude ComputeControl-request.msg.html

(cl:defclass <ComputeControl-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type control_core_msgs-msg:RobotState
    :initform (cl:make-instance 'control_core_msgs-msg:RobotState))
   (do_stop
    :reader do_stop
    :initarg :do_stop
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ComputeControl-request (<ComputeControl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeControl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeControl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<ComputeControl-request> is deprecated: use control_core_msgs-srv:ComputeControl-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ComputeControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:state-val is deprecated.  Use control_core_msgs-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'do_stop-val :lambda-list '(m))
(cl:defmethod do_stop-val ((m <ComputeControl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:do_stop-val is deprecated.  Use control_core_msgs-srv:do_stop instead.")
  (do_stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeControl-request>) ostream)
  "Serializes a message object of type '<ComputeControl-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'state) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_stop)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeControl-request>) istream)
  "Deserializes a message object of type '<ComputeControl-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'state) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_stop)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeControl-request>)))
  "Returns string type for a service object of type '<ComputeControl-request>"
  "control_core_msgs/ComputeControlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeControl-request)))
  "Returns string type for a service object of type 'ComputeControl-request"
  "control_core_msgs/ComputeControlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeControl-request>)))
  "Returns md5sum for a message object of type '<ComputeControl-request>"
  "3f913e6ebf5507fa61207688a03bb51a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeControl-request)))
  "Returns md5sum for a message object of type 'ComputeControl-request"
  "3f913e6ebf5507fa61207688a03bb51a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeControl-request>)))
  "Returns full string definition for message of type '<ComputeControl-request>"
  (cl:format cl:nil "RobotState state    # current robot state~%uint8 do_stop       # plugin request stop~%~%================================================================================~%MSG: control_core_msgs/RobotState~%JointState joints~%CartesianState floating_base~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped[] ft_sensors~%SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeControl-request)))
  "Returns full string definition for message of type 'ComputeControl-request"
  (cl:format cl:nil "RobotState state    # current robot state~%uint8 do_stop       # plugin request stop~%~%================================================================================~%MSG: control_core_msgs/RobotState~%JointState joints~%CartesianState floating_base~%sensor_msgs/Imu imu~%geometry_msgs/WrenchStamped[] ft_sensors~%SkinPatch[] patches~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%================================================================================~%MSG: control_core_msgs/CartesianState~%geometry_msgs/Pose position~%geometry_msgs/Twist velocity~%geometry_msgs/Accel acceleration~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the ~%# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the~%# data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation ~%# estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each ~%# covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/WrenchStamped~%# A wrench with reference coordinate frame and timestamp~%Header header~%Wrench wrench~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: control_core_msgs/SkinPatch~%std_msgs/Header header~%geometry_msgs/Pose pose~%SkinModality force~%SkinModality proximity~%std_msgs/Float64 min_dist~%std_msgs/Float64 max_dist~%================================================================================~%MSG: control_core_msgs/SkinModality~%std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeControl-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'state))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeControl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeControl-request
    (cl:cons ':state (state msg))
    (cl:cons ':do_stop (do_stop msg))
))
;//! \htmlinclude ComputeControl-response.msg.html

(cl:defclass <ComputeControl-response> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type control_core_msgs-msg:JointState
    :initform (cl:make-instance 'control_core_msgs-msg:JointState))
   (do_stop
    :reader do_stop
    :initarg :do_stop
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ComputeControl-response (<ComputeControl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ComputeControl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ComputeControl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-srv:<ComputeControl-response> is deprecated: use control_core_msgs-srv:ComputeControl-response instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <ComputeControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:command-val is deprecated.  Use control_core_msgs-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'do_stop-val :lambda-list '(m))
(cl:defmethod do_stop-val ((m <ComputeControl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-srv:do_stop-val is deprecated.  Use control_core_msgs-srv:do_stop instead.")
  (do_stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ComputeControl-response>) ostream)
  "Serializes a message object of type '<ComputeControl-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'command) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_stop)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ComputeControl-response>) istream)
  "Deserializes a message object of type '<ComputeControl-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'command) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'do_stop)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ComputeControl-response>)))
  "Returns string type for a service object of type '<ComputeControl-response>"
  "control_core_msgs/ComputeControlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeControl-response)))
  "Returns string type for a service object of type 'ComputeControl-response"
  "control_core_msgs/ComputeControlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ComputeControl-response>)))
  "Returns md5sum for a message object of type '<ComputeControl-response>"
  "3f913e6ebf5507fa61207688a03bb51a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ComputeControl-response)))
  "Returns md5sum for a message object of type 'ComputeControl-response"
  "3f913e6ebf5507fa61207688a03bb51a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ComputeControl-response>)))
  "Returns full string definition for message of type '<ComputeControl-response>"
  (cl:format cl:nil "JointState command  # current robot command~%uint8 do_stop       # client request stop~%~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ComputeControl-response)))
  "Returns full string definition for message of type 'ComputeControl-response"
  (cl:format cl:nil "JointState command  # current robot command~%uint8 do_stop       # client request stop~%~%================================================================================~%MSG: control_core_msgs/JointState~%Vector position~%Vector velocity~%Vector acceleration~%================================================================================~%MSG: control_core_msgs/Vector~%float64[] data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ComputeControl-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'command))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ComputeControl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ComputeControl-response
    (cl:cons ':command (command msg))
    (cl:cons ':do_stop (do_stop msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ComputeControl)))
  'ComputeControl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ComputeControl)))
  'ComputeControl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ComputeControl)))
  "Returns string type for a service object of type '<ComputeControl>"
  "control_core_msgs/ComputeControl")