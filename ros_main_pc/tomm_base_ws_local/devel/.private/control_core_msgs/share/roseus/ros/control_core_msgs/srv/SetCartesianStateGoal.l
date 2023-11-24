;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::SetCartesianStateGoal)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'SetCartesianStateGoal (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::SETCARTESIANSTATEGOAL")
  (make-package "CONTROL_CORE_MSGS::SETCARTESIANSTATEGOAL"))
(unless (find-package "CONTROL_CORE_MSGS::SETCARTESIANSTATEGOALREQUEST")
  (make-package "CONTROL_CORE_MSGS::SETCARTESIANSTATEGOALREQUEST"))
(unless (find-package "CONTROL_CORE_MSGS::SETCARTESIANSTATEGOALRESPONSE")
  (make-package "CONTROL_CORE_MSGS::SETCARTESIANSTATEGOALRESPONSE"))

(in-package "ROS")





(defclass control_core_msgs::SetCartesianStateGoalRequest
  :super ros::object
  :slots (_goal ))

(defmethod control_core_msgs::SetCartesianStateGoalRequest
  (:init
   (&key
    ((:goal __goal) (instance control_core_msgs::CartesianState :init))
    )
   (send-super :init)
   (setq _goal __goal)
   self)
  (:goal
   (&rest __goal)
   (if (keywordp (car __goal))
       (send* _goal __goal)
     (progn
       (if __goal (setq _goal (car __goal)))
       _goal)))
  (:serialization-length
   ()
   (+
    ;; control_core_msgs/CartesianState _goal
    (send _goal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; control_core_msgs/CartesianState _goal
       (send _goal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; control_core_msgs/CartesianState _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;;
   self)
  )

(defclass control_core_msgs::SetCartesianStateGoalResponse
  :super ros::object
  :slots ())

(defmethod control_core_msgs::SetCartesianStateGoalResponse
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass control_core_msgs::SetCartesianStateGoal
  :super ros::object
  :slots ())

(setf (get control_core_msgs::SetCartesianStateGoal :md5sum-) "2f3406a57b13431c34e22c35d33bdee4")
(setf (get control_core_msgs::SetCartesianStateGoal :datatype-) "control_core_msgs/SetCartesianStateGoal")
(setf (get control_core_msgs::SetCartesianStateGoal :request) control_core_msgs::SetCartesianStateGoalRequest)
(setf (get control_core_msgs::SetCartesianStateGoal :response) control_core_msgs::SetCartesianStateGoalResponse)

(defmethod control_core_msgs::SetCartesianStateGoalRequest
  (:response () (instance control_core_msgs::SetCartesianStateGoalResponse :init)))

(setf (get control_core_msgs::SetCartesianStateGoalRequest :md5sum-) "2f3406a57b13431c34e22c35d33bdee4")
(setf (get control_core_msgs::SetCartesianStateGoalRequest :datatype-) "control_core_msgs/SetCartesianStateGoalRequest")
(setf (get control_core_msgs::SetCartesianStateGoalRequest :definition-)
      "CartesianState goal

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
---
")

(setf (get control_core_msgs::SetCartesianStateGoalResponse :md5sum-) "2f3406a57b13431c34e22c35d33bdee4")
(setf (get control_core_msgs::SetCartesianStateGoalResponse :datatype-) "control_core_msgs/SetCartesianStateGoalResponse")
(setf (get control_core_msgs::SetCartesianStateGoalResponse :definition-)
      "CartesianState goal

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
---
")



(provide :control_core_msgs/SetCartesianStateGoal "2f3406a57b13431c34e22c35d33bdee4")


