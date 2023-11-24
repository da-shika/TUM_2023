;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::JointState)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'JointState (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::JOINTSTATE")
  (make-package "CONTROL_CORE_MSGS::JOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude JointState.msg.html


(defclass control_core_msgs::JointState
  :super ros::object
  :slots (_position _velocity _acceleration ))

(defmethod control_core_msgs::JointState
  (:init
   (&key
    ((:position __position) (instance control_core_msgs::Vector :init))
    ((:velocity __velocity) (instance control_core_msgs::Vector :init))
    ((:acceleration __acceleration) (instance control_core_msgs::Vector :init))
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
    ;; control_core_msgs/Vector _position
    (send _position :serialization-length)
    ;; control_core_msgs/Vector _velocity
    (send _velocity :serialization-length)
    ;; control_core_msgs/Vector _acceleration
    (send _acceleration :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; control_core_msgs/Vector _position
       (send _position :serialize s)
     ;; control_core_msgs/Vector _velocity
       (send _velocity :serialize s)
     ;; control_core_msgs/Vector _acceleration
       (send _acceleration :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; control_core_msgs/Vector _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; control_core_msgs/Vector _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; control_core_msgs/Vector _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::JointState :md5sum-) "6682f84f321f6df0d77b7afd20d85702")
(setf (get control_core_msgs::JointState :datatype-) "control_core_msgs/JointState")
(setf (get control_core_msgs::JointState :definition-)
      "Vector position
Vector velocity
Vector acceleration
================================================================================
MSG: control_core_msgs/Vector
float64[] data
")



(provide :control_core_msgs/JointState "6682f84f321f6df0d77b7afd20d85702")


