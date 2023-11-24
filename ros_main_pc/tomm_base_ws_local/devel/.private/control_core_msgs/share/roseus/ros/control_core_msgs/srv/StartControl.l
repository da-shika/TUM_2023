;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::StartControl)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'StartControl (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::STARTCONTROL")
  (make-package "CONTROL_CORE_MSGS::STARTCONTROL"))
(unless (find-package "CONTROL_CORE_MSGS::STARTCONTROLREQUEST")
  (make-package "CONTROL_CORE_MSGS::STARTCONTROLREQUEST"))
(unless (find-package "CONTROL_CORE_MSGS::STARTCONTROLRESPONSE")
  (make-package "CONTROL_CORE_MSGS::STARTCONTROLRESPONSE"))

(in-package "ROS")





(defclass control_core_msgs::StartControlRequest
  :super ros::object
  :slots (_CONNECTION_STATE_INIT _CONNECTION_STATE_START _connection_state ))

(defmethod control_core_msgs::StartControlRequest
  (:init
   (&key
    ((:CONNECTION_STATE_INIT __CONNECTION_STATE_INIT) 0)
    ((:CONNECTION_STATE_START __CONNECTION_STATE_START) 0)
    ((:connection_state __connection_state) 0)
    )
   (send-super :init)
   (setq _CONNECTION_STATE_INIT (round __CONNECTION_STATE_INIT))
   (setq _CONNECTION_STATE_START (round __CONNECTION_STATE_START))
   (setq _connection_state (round __connection_state))
   self)
  (:CONNECTION_STATE_INIT
   (&optional __CONNECTION_STATE_INIT)
   (if __CONNECTION_STATE_INIT (setq _CONNECTION_STATE_INIT __CONNECTION_STATE_INIT)) _CONNECTION_STATE_INIT)
  (:CONNECTION_STATE_START
   (&optional __CONNECTION_STATE_START)
   (if __CONNECTION_STATE_START (setq _CONNECTION_STATE_START __CONNECTION_STATE_START)) _CONNECTION_STATE_START)
  (:connection_state
   (&optional __connection_state)
   (if __connection_state (setq _connection_state __connection_state)) _connection_state)
  (:serialization-length
   ()
   (+
    ;; uint8 _CONNECTION_STATE_INIT
    1
    ;; uint8 _CONNECTION_STATE_START
    1
    ;; uint8 _connection_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _CONNECTION_STATE_INIT
       (write-byte _CONNECTION_STATE_INIT s)
     ;; uint8 _CONNECTION_STATE_START
       (write-byte _CONNECTION_STATE_START s)
     ;; uint8 _connection_state
       (write-byte _connection_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _CONNECTION_STATE_INIT
     (setq _CONNECTION_STATE_INIT (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _CONNECTION_STATE_START
     (setq _CONNECTION_STATE_START (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _connection_state
     (setq _connection_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(defclass control_core_msgs::StartControlResponse
  :super ros::object
  :slots (_state ))

(defmethod control_core_msgs::StartControlResponse
  (:init
   (&key
    ((:state __state) (instance control_core_msgs::RobotState :init))
    )
   (send-super :init)
   (setq _state __state)
   self)
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:serialization-length
   ()
   (+
    ;; control_core_msgs/RobotState _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; control_core_msgs/RobotState _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; control_core_msgs/RobotState _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(defclass control_core_msgs::StartControl
  :super ros::object
  :slots ())

(setf (get control_core_msgs::StartControl :md5sum-) "adb9278d966383f41d079c269f203a93")
(setf (get control_core_msgs::StartControl :datatype-) "control_core_msgs/StartControl")
(setf (get control_core_msgs::StartControl :request) control_core_msgs::StartControlRequest)
(setf (get control_core_msgs::StartControl :response) control_core_msgs::StartControlResponse)

(defmethod control_core_msgs::StartControlRequest
  (:response () (instance control_core_msgs::StartControlResponse :init)))

(setf (get control_core_msgs::StartControlRequest :md5sum-) "adb9278d966383f41d079c269f203a93")
(setf (get control_core_msgs::StartControlRequest :datatype-) "control_core_msgs/StartControlRequest")
(setf (get control_core_msgs::StartControlRequest :definition-)
      "uint8 CONNECTION_STATE_INIT
uint8 CONNECTION_STATE_START
uint8 connection_state          # connection state
---
RobotState state                # robot state

================================================================================
MSG: control_core_msgs/RobotState
JointState joints
CartesianState floating_base
sensor_msgs/Imu imu
geometry_msgs/WrenchStamped[] ft_sensors
SkinPatch[] patches
================================================================================
MSG: control_core_msgs/JointState
Vector position
Vector velocity
Vector acceleration
================================================================================
MSG: control_core_msgs/Vector
float64[] data
================================================================================
MSG: control_core_msgs/CartesianState
geometry_msgs/Pose position
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: control_core_msgs/SkinPatch
std_msgs/Header header
geometry_msgs/Pose pose
SkinModality force
SkinModality proximity
std_msgs/Float64 min_dist
std_msgs/Float64 max_dist
================================================================================
MSG: control_core_msgs/SkinModality
std_msgs/Float64 min
std_msgs/Float64 max
std_msgs/Float64 area
geometry_msgs/Point cop
geometry_msgs/Wrench wrench
geometry_msgs/Polygon hull
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
")

(setf (get control_core_msgs::StartControlResponse :md5sum-) "adb9278d966383f41d079c269f203a93")
(setf (get control_core_msgs::StartControlResponse :datatype-) "control_core_msgs/StartControlResponse")
(setf (get control_core_msgs::StartControlResponse :definition-)
      "uint8 CONNECTION_STATE_INIT
uint8 CONNECTION_STATE_START
uint8 connection_state          # connection state
---
RobotState state                # robot state

================================================================================
MSG: control_core_msgs/RobotState
JointState joints
CartesianState floating_base
sensor_msgs/Imu imu
geometry_msgs/WrenchStamped[] ft_sensors
SkinPatch[] patches
================================================================================
MSG: control_core_msgs/JointState
Vector position
Vector velocity
Vector acceleration
================================================================================
MSG: control_core_msgs/Vector
float64[] data
================================================================================
MSG: control_core_msgs/CartesianState
geometry_msgs/Pose position
geometry_msgs/Twist velocity
geometry_msgs/Accel acceleration
================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: sensor_msgs/Imu
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the 
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation 
# estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each 
# covariance matrix, and disregard the associated estimate.

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z 

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/WrenchStamped
# A wrench with reference coordinate frame and timestamp
Header header
Wrench wrench

================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: control_core_msgs/SkinPatch
std_msgs/Header header
geometry_msgs/Pose pose
SkinModality force
SkinModality proximity
std_msgs/Float64 min_dist
std_msgs/Float64 max_dist
================================================================================
MSG: control_core_msgs/SkinModality
std_msgs/Float64 min
std_msgs/Float64 max
std_msgs/Float64 area
geometry_msgs/Point cop
geometry_msgs/Wrench wrench
geometry_msgs/Polygon hull
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
")



(provide :control_core_msgs/StartControl "adb9278d966383f41d079c269f203a93")


