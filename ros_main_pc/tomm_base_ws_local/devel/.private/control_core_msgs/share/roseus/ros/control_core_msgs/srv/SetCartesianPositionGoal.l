;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::SetCartesianPositionGoal)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'SetCartesianPositionGoal (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::SETCARTESIANPOSITIONGOAL")
  (make-package "CONTROL_CORE_MSGS::SETCARTESIANPOSITIONGOAL"))
(unless (find-package "CONTROL_CORE_MSGS::SETCARTESIANPOSITIONGOALREQUEST")
  (make-package "CONTROL_CORE_MSGS::SETCARTESIANPOSITIONGOALREQUEST"))
(unless (find-package "CONTROL_CORE_MSGS::SETCARTESIANPOSITIONGOALRESPONSE")
  (make-package "CONTROL_CORE_MSGS::SETCARTESIANPOSITIONGOALRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass control_core_msgs::SetCartesianPositionGoalRequest
  :super ros::object
  :slots (_goal ))

(defmethod control_core_msgs::SetCartesianPositionGoalRequest
  (:init
   (&key
    ((:goal __goal) (instance geometry_msgs::Pose :init))
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
    ;; geometry_msgs/Pose _goal
    (send _goal :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Pose _goal
       (send _goal :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Pose _goal
     (send _goal :deserialize buf ptr-) (incf ptr- (send _goal :serialization-length))
   ;;
   self)
  )

(defclass control_core_msgs::SetCartesianPositionGoalResponse
  :super ros::object
  :slots ())

(defmethod control_core_msgs::SetCartesianPositionGoalResponse
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

(defclass control_core_msgs::SetCartesianPositionGoal
  :super ros::object
  :slots ())

(setf (get control_core_msgs::SetCartesianPositionGoal :md5sum-) "313b76aa4f010582b3257488c62ac366")
(setf (get control_core_msgs::SetCartesianPositionGoal :datatype-) "control_core_msgs/SetCartesianPositionGoal")
(setf (get control_core_msgs::SetCartesianPositionGoal :request) control_core_msgs::SetCartesianPositionGoalRequest)
(setf (get control_core_msgs::SetCartesianPositionGoal :response) control_core_msgs::SetCartesianPositionGoalResponse)

(defmethod control_core_msgs::SetCartesianPositionGoalRequest
  (:response () (instance control_core_msgs::SetCartesianPositionGoalResponse :init)))

(setf (get control_core_msgs::SetCartesianPositionGoalRequest :md5sum-) "313b76aa4f010582b3257488c62ac366")
(setf (get control_core_msgs::SetCartesianPositionGoalRequest :datatype-) "control_core_msgs/SetCartesianPositionGoalRequest")
(setf (get control_core_msgs::SetCartesianPositionGoalRequest :definition-)
      "geometry_msgs/Pose goal

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
---
")

(setf (get control_core_msgs::SetCartesianPositionGoalResponse :md5sum-) "313b76aa4f010582b3257488c62ac366")
(setf (get control_core_msgs::SetCartesianPositionGoalResponse :datatype-) "control_core_msgs/SetCartesianPositionGoalResponse")
(setf (get control_core_msgs::SetCartesianPositionGoalResponse :definition-)
      "geometry_msgs/Pose goal

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
---
")



(provide :control_core_msgs/SetCartesianPositionGoal "313b76aa4f010582b3257488c62ac366")


