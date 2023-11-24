;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::SkinModality)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'SkinModality (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::SKINMODALITY")
  (make-package "CONTROL_CORE_MSGS::SKINMODALITY"))

(in-package "ROS")
;;//! \htmlinclude SkinModality.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_core_msgs::SkinModality
  :super ros::object
  :slots (_min _max _area _cop _wrench _hull ))

(defmethod control_core_msgs::SkinModality
  (:init
   (&key
    ((:min __min) (instance std_msgs::Float64 :init))
    ((:max __max) (instance std_msgs::Float64 :init))
    ((:area __area) (instance std_msgs::Float64 :init))
    ((:cop __cop) (instance geometry_msgs::Point :init))
    ((:wrench __wrench) (instance geometry_msgs::Wrench :init))
    ((:hull __hull) (instance geometry_msgs::Polygon :init))
    )
   (send-super :init)
   (setq _min __min)
   (setq _max __max)
   (setq _area __area)
   (setq _cop __cop)
   (setq _wrench __wrench)
   (setq _hull __hull)
   self)
  (:min
   (&rest __min)
   (if (keywordp (car __min))
       (send* _min __min)
     (progn
       (if __min (setq _min (car __min)))
       _min)))
  (:max
   (&rest __max)
   (if (keywordp (car __max))
       (send* _max __max)
     (progn
       (if __max (setq _max (car __max)))
       _max)))
  (:area
   (&rest __area)
   (if (keywordp (car __area))
       (send* _area __area)
     (progn
       (if __area (setq _area (car __area)))
       _area)))
  (:cop
   (&rest __cop)
   (if (keywordp (car __cop))
       (send* _cop __cop)
     (progn
       (if __cop (setq _cop (car __cop)))
       _cop)))
  (:wrench
   (&rest __wrench)
   (if (keywordp (car __wrench))
       (send* _wrench __wrench)
     (progn
       (if __wrench (setq _wrench (car __wrench)))
       _wrench)))
  (:hull
   (&rest __hull)
   (if (keywordp (car __hull))
       (send* _hull __hull)
     (progn
       (if __hull (setq _hull (car __hull)))
       _hull)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float64 _min
    (send _min :serialization-length)
    ;; std_msgs/Float64 _max
    (send _max :serialization-length)
    ;; std_msgs/Float64 _area
    (send _area :serialization-length)
    ;; geometry_msgs/Point _cop
    (send _cop :serialization-length)
    ;; geometry_msgs/Wrench _wrench
    (send _wrench :serialization-length)
    ;; geometry_msgs/Polygon _hull
    (send _hull :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float64 _min
       (send _min :serialize s)
     ;; std_msgs/Float64 _max
       (send _max :serialize s)
     ;; std_msgs/Float64 _area
       (send _area :serialize s)
     ;; geometry_msgs/Point _cop
       (send _cop :serialize s)
     ;; geometry_msgs/Wrench _wrench
       (send _wrench :serialize s)
     ;; geometry_msgs/Polygon _hull
       (send _hull :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float64 _min
     (send _min :deserialize buf ptr-) (incf ptr- (send _min :serialization-length))
   ;; std_msgs/Float64 _max
     (send _max :deserialize buf ptr-) (incf ptr- (send _max :serialization-length))
   ;; std_msgs/Float64 _area
     (send _area :deserialize buf ptr-) (incf ptr- (send _area :serialization-length))
   ;; geometry_msgs/Point _cop
     (send _cop :deserialize buf ptr-) (incf ptr- (send _cop :serialization-length))
   ;; geometry_msgs/Wrench _wrench
     (send _wrench :deserialize buf ptr-) (incf ptr- (send _wrench :serialization-length))
   ;; geometry_msgs/Polygon _hull
     (send _hull :deserialize buf ptr-) (incf ptr- (send _hull :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::SkinModality :md5sum-) "db74e377abd390ca1fca36f1294853e5")
(setf (get control_core_msgs::SkinModality :datatype-) "control_core_msgs/SkinModality")
(setf (get control_core_msgs::SkinModality :definition-)
      "std_msgs/Float64 min
std_msgs/Float64 max
std_msgs/Float64 area
geometry_msgs/Point cop
geometry_msgs/Wrench wrench
geometry_msgs/Polygon hull
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

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



(provide :control_core_msgs/SkinModality "db74e377abd390ca1fca36f1294853e5")


