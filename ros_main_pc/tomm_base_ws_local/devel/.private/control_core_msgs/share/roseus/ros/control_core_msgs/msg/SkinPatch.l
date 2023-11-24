;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::SkinPatch)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'SkinPatch (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::SKINPATCH")
  (make-package "CONTROL_CORE_MSGS::SKINPATCH"))

(in-package "ROS")
;;//! \htmlinclude SkinPatch.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_core_msgs::SkinPatch
  :super ros::object
  :slots (_header _pose _force _proximity _min_dist _max_dist ))

(defmethod control_core_msgs::SkinPatch
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:force __force) (instance control_core_msgs::SkinModality :init))
    ((:proximity __proximity) (instance control_core_msgs::SkinModality :init))
    ((:min_dist __min_dist) (instance std_msgs::Float64 :init))
    ((:max_dist __max_dist) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _force __force)
   (setq _proximity __proximity)
   (setq _min_dist __min_dist)
   (setq _max_dist __max_dist)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:force
   (&rest __force)
   (if (keywordp (car __force))
       (send* _force __force)
     (progn
       (if __force (setq _force (car __force)))
       _force)))
  (:proximity
   (&rest __proximity)
   (if (keywordp (car __proximity))
       (send* _proximity __proximity)
     (progn
       (if __proximity (setq _proximity (car __proximity)))
       _proximity)))
  (:min_dist
   (&rest __min_dist)
   (if (keywordp (car __min_dist))
       (send* _min_dist __min_dist)
     (progn
       (if __min_dist (setq _min_dist (car __min_dist)))
       _min_dist)))
  (:max_dist
   (&rest __max_dist)
   (if (keywordp (car __max_dist))
       (send* _max_dist __max_dist)
     (progn
       (if __max_dist (setq _max_dist (car __max_dist)))
       _max_dist)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; control_core_msgs/SkinModality _force
    (send _force :serialization-length)
    ;; control_core_msgs/SkinModality _proximity
    (send _proximity :serialization-length)
    ;; std_msgs/Float64 _min_dist
    (send _min_dist :serialization-length)
    ;; std_msgs/Float64 _max_dist
    (send _max_dist :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; control_core_msgs/SkinModality _force
       (send _force :serialize s)
     ;; control_core_msgs/SkinModality _proximity
       (send _proximity :serialize s)
     ;; std_msgs/Float64 _min_dist
       (send _min_dist :serialize s)
     ;; std_msgs/Float64 _max_dist
       (send _max_dist :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; control_core_msgs/SkinModality _force
     (send _force :deserialize buf ptr-) (incf ptr- (send _force :serialization-length))
   ;; control_core_msgs/SkinModality _proximity
     (send _proximity :deserialize buf ptr-) (incf ptr- (send _proximity :serialization-length))
   ;; std_msgs/Float64 _min_dist
     (send _min_dist :deserialize buf ptr-) (incf ptr- (send _min_dist :serialization-length))
   ;; std_msgs/Float64 _max_dist
     (send _max_dist :deserialize buf ptr-) (incf ptr- (send _max_dist :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::SkinPatch :md5sum-) "a5c4ba5fbedd1c63abbdd1a1450dda87")
(setf (get control_core_msgs::SkinPatch :datatype-) "control_core_msgs/SkinPatch")
(setf (get control_core_msgs::SkinPatch :definition-)
      "std_msgs/Header header
geometry_msgs/Pose pose
SkinModality force
SkinModality proximity
std_msgs/Float64 min_dist
std_msgs/Float64 max_dist
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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

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
")



(provide :control_core_msgs/SkinPatch "a5c4ba5fbedd1c63abbdd1a1450dda87")


