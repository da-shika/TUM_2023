;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::MoveToCartesianGoal)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'MoveToCartesianGoal (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::MOVETOCARTESIANGOAL")
  (make-package "BEHAVIOR_MSGS::MOVETOCARTESIANGOAL"))

(in-package "ROS")
;;//! \htmlinclude MoveToCartesianGoal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass behavior_msgs::MoveToCartesianGoal
  :super ros::object
  :slots (_target _motion_task _period ))

(defmethod behavior_msgs::MoveToCartesianGoal
  (:init
   (&key
    ((:target __target) (instance geometry_msgs::PoseStamped :init))
    ((:motion_task __motion_task) (instance std_msgs::String :init))
    ((:period __period) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _target __target)
   (setq _motion_task __motion_task)
   (setq _period __period)
   self)
  (:target
   (&rest __target)
   (if (keywordp (car __target))
       (send* _target __target)
     (progn
       (if __target (setq _target (car __target)))
       _target)))
  (:motion_task
   (&rest __motion_task)
   (if (keywordp (car __motion_task))
       (send* _motion_task __motion_task)
     (progn
       (if __motion_task (setq _motion_task (car __motion_task)))
       _motion_task)))
  (:period
   (&rest __period)
   (if (keywordp (car __period))
       (send* _period __period)
     (progn
       (if __period (setq _period (car __period)))
       _period)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/PoseStamped _target
    (send _target :serialization-length)
    ;; std_msgs/String _motion_task
    (send _motion_task :serialization-length)
    ;; std_msgs/Float64 _period
    (send _period :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/PoseStamped _target
       (send _target :serialize s)
     ;; std_msgs/String _motion_task
       (send _motion_task :serialize s)
     ;; std_msgs/Float64 _period
       (send _period :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/PoseStamped _target
     (send _target :deserialize buf ptr-) (incf ptr- (send _target :serialization-length))
   ;; std_msgs/String _motion_task
     (send _motion_task :deserialize buf ptr-) (incf ptr- (send _motion_task :serialization-length))
   ;; std_msgs/Float64 _period
     (send _period :deserialize buf ptr-) (incf ptr- (send _period :serialization-length))
   ;;
   self)
  )

(setf (get behavior_msgs::MoveToCartesianGoal :md5sum-) "6a64607e6531d426b71f2d434d7f8905")
(setf (get behavior_msgs::MoveToCartesianGoal :datatype-) "behavior_msgs/MoveToCartesianGoal")
(setf (get behavior_msgs::MoveToCartesianGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
geometry_msgs/PoseStamped target  # target pose wrt frame_id in header field
std_msgs/String motion_task       # motion task for the movement
std_msgs/Float64 period           # time period of the motion

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

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
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Float64
float64 data
")



(provide :behavior_msgs/MoveToCartesianGoal "6a64607e6531d426b71f2d434d7f8905")

