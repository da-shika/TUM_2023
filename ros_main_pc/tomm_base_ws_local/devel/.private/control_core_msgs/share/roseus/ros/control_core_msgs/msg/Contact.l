;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::Contact)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'Contact (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::CONTACT")
  (make-package "CONTROL_CORE_MSGS::CONTACT"))

(in-package "ROS")
;;//! \htmlinclude Contact.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_core_msgs::Contact
  :super ros::object
  :slots (_pose _hull _offset _friction ))

(defmethod control_core_msgs::Contact
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:hull __hull) (instance geometry_msgs::Polygon :init))
    ((:offset __offset) (instance geometry_msgs::Point :init))
    ((:friction __friction) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _hull __hull)
   (setq _offset __offset)
   (setq _friction __friction)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:hull
   (&rest __hull)
   (if (keywordp (car __hull))
       (send* _hull __hull)
     (progn
       (if __hull (setq _hull (car __hull)))
       _hull)))
  (:offset
   (&rest __offset)
   (if (keywordp (car __offset))
       (send* _offset __offset)
     (progn
       (if __offset (setq _offset (car __offset)))
       _offset)))
  (:friction
   (&rest __friction)
   (if (keywordp (car __friction))
       (send* _friction __friction)
     (progn
       (if __friction (setq _friction (car __friction)))
       _friction)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Polygon _hull
    (send _hull :serialization-length)
    ;; geometry_msgs/Point _offset
    (send _offset :serialization-length)
    ;; std_msgs/Float64 _friction
    (send _friction :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Polygon _hull
       (send _hull :serialize s)
     ;; geometry_msgs/Point _offset
       (send _offset :serialize s)
     ;; std_msgs/Float64 _friction
       (send _friction :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Polygon _hull
     (send _hull :deserialize buf ptr-) (incf ptr- (send _hull :serialization-length))
   ;; geometry_msgs/Point _offset
     (send _offset :deserialize buf ptr-) (incf ptr- (send _offset :serialization-length))
   ;; std_msgs/Float64 _friction
     (send _friction :deserialize buf ptr-) (incf ptr- (send _friction :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::Contact :md5sum-) "092ef89bc484cc9b681ba742d6241a00")
(setf (get control_core_msgs::Contact :datatype-) "control_core_msgs/Contact")
(setf (get control_core_msgs::Contact :definition-)
      "geometry_msgs/Pose pose
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
")



(provide :control_core_msgs/Contact "092ef89bc484cc9b681ba742d6241a00")


