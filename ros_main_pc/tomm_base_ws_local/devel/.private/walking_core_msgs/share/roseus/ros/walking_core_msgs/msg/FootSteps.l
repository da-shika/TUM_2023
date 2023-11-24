;; Auto-generated. Do not edit!


(when (boundp 'walking_core_msgs::FootSteps)
  (if (not (find-package "WALKING_CORE_MSGS"))
    (make-package "WALKING_CORE_MSGS"))
  (shadow 'FootSteps (find-package "WALKING_CORE_MSGS")))
(unless (find-package "WALKING_CORE_MSGS::FOOTSTEPS")
  (make-package "WALKING_CORE_MSGS::FOOTSTEPS"))

(in-package "ROS")
;;//! \htmlinclude FootSteps.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass walking_core_msgs::FootSteps
  :super ros::object
  :slots (_header _footsteps ))

(defmethod walking_core_msgs::FootSteps
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:footsteps __footsteps) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _footsteps __footsteps)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:footsteps
   (&rest __footsteps)
   (if (keywordp (car __footsteps))
       (send* _footsteps __footsteps)
     (progn
       (if __footsteps (setq _footsteps (car __footsteps)))
       _footsteps)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; walking_core_msgs/FootStep[] _footsteps
    (apply #'+ (send-all _footsteps :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; walking_core_msgs/FootStep[] _footsteps
     (write-long (length _footsteps) s)
     (dolist (elem _footsteps)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; walking_core_msgs/FootStep[] _footsteps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footsteps (let (r) (dotimes (i n) (push (instance walking_core_msgs::FootStep :init) r)) r))
     (dolist (elem- _footsteps)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get walking_core_msgs::FootSteps :md5sum-) "afa335a6af8741fb5411d66b054528e3")
(setf (get walking_core_msgs::FootSteps :datatype-) "walking_core_msgs/FootSteps")
(setf (get walking_core_msgs::FootSteps :definition-)
      "std_msgs/Header header
FootStep[] footsteps

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
MSG: walking_core_msgs/FootStep
control_core_msgs/Contact contact
std_msgs/Int64 body_id
std_msgs/Bool final_step
std_msgs/Int64 n_step
================================================================================
MSG: control_core_msgs/Contact
geometry_msgs/Pose pose
geometry_msgs/Polygon hull
geometry_msgs/Point offset
std_msgs/Float64 friction
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
float32 z
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :walking_core_msgs/FootSteps "afa335a6af8741fb5411d66b054528e3")


