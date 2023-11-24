;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::LinearState)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'LinearState (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::LINEARSTATE")
  (make-package "CONTROL_CORE_MSGS::LINEARSTATE"))

(in-package "ROS")
;;//! \htmlinclude LinearState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass control_core_msgs::LinearState
  :super ros::object
  :slots (_position _velocity _acceleration ))

(defmethod control_core_msgs::LinearState
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::LinearState :md5sum-) "b6bed245f19156eb5ee582c6cf27c090")
(setf (get control_core_msgs::LinearState :datatype-) "control_core_msgs/LinearState")
(setf (get control_core_msgs::LinearState :definition-)
      "geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

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
")



(provide :control_core_msgs/LinearState "b6bed245f19156eb5ee582c6cf27c090")


