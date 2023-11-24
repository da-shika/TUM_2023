;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::SkinPatches)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'SkinPatches (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::SKINPATCHES")
  (make-package "CONTROL_CORE_MSGS::SKINPATCHES"))

(in-package "ROS")
;;//! \htmlinclude SkinPatches.msg.html


(defclass control_core_msgs::SkinPatches
  :super ros::object
  :slots (_patches ))

(defmethod control_core_msgs::SkinPatches
  (:init
   (&key
    ((:patches __patches) ())
    )
   (send-super :init)
   (setq _patches __patches)
   self)
  (:patches
   (&rest __patches)
   (if (keywordp (car __patches))
       (send* _patches __patches)
     (progn
       (if __patches (setq _patches (car __patches)))
       _patches)))
  (:serialization-length
   ()
   (+
    ;; control_core_msgs/SkinPatch[] _patches
    (apply #'+ (send-all _patches :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; control_core_msgs/SkinPatch[] _patches
     (write-long (length _patches) s)
     (dolist (elem _patches)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; control_core_msgs/SkinPatch[] _patches
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _patches (let (r) (dotimes (i n) (push (instance control_core_msgs::SkinPatch :init) r)) r))
     (dolist (elem- _patches)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get control_core_msgs::SkinPatches :md5sum-) "54325af118430bc2db92b5f684606b52")
(setf (get control_core_msgs::SkinPatches :datatype-) "control_core_msgs/SkinPatches")
(setf (get control_core_msgs::SkinPatches :definition-)
      "SkinPatch[] patches
================================================================================
MSG: control_core_msgs/SkinPatch
std_msgs/Header header
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



(provide :control_core_msgs/SkinPatches "54325af118430bc2db92b5f684606b52")


