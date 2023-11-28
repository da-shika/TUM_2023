;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::ChangeContactGoal)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'ChangeContactGoal (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::CHANGECONTACTGOAL")
  (make-package "BEHAVIOR_MSGS::CHANGECONTACTGOAL"))

(in-package "ROS")
;;//! \htmlinclude ChangeContactGoal.msg.html
(if (not (find-package "CONTROL_CORE_MSGS"))
  (ros::roseus-add-msgs "control_core_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass behavior_msgs::ChangeContactGoal
  :super ros::object
  :slots (_contact _contact_task _period_shift _period_move ))

(defmethod behavior_msgs::ChangeContactGoal
  (:init
   (&key
    ((:contact __contact) (instance control_core_msgs::Contact :init))
    ((:contact_task __contact_task) (instance std_msgs::String :init))
    ((:period_shift __period_shift) (instance std_msgs::Float64 :init))
    ((:period_move __period_move) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _contact __contact)
   (setq _contact_task __contact_task)
   (setq _period_shift __period_shift)
   (setq _period_move __period_move)
   self)
  (:contact
   (&rest __contact)
   (if (keywordp (car __contact))
       (send* _contact __contact)
     (progn
       (if __contact (setq _contact (car __contact)))
       _contact)))
  (:contact_task
   (&rest __contact_task)
   (if (keywordp (car __contact_task))
       (send* _contact_task __contact_task)
     (progn
       (if __contact_task (setq _contact_task (car __contact_task)))
       _contact_task)))
  (:period_shift
   (&rest __period_shift)
   (if (keywordp (car __period_shift))
       (send* _period_shift __period_shift)
     (progn
       (if __period_shift (setq _period_shift (car __period_shift)))
       _period_shift)))
  (:period_move
   (&rest __period_move)
   (if (keywordp (car __period_move))
       (send* _period_move __period_move)
     (progn
       (if __period_move (setq _period_move (car __period_move)))
       _period_move)))
  (:serialization-length
   ()
   (+
    ;; control_core_msgs/Contact _contact
    (send _contact :serialization-length)
    ;; std_msgs/String _contact_task
    (send _contact_task :serialization-length)
    ;; std_msgs/Float64 _period_shift
    (send _period_shift :serialization-length)
    ;; std_msgs/Float64 _period_move
    (send _period_move :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; control_core_msgs/Contact _contact
       (send _contact :serialize s)
     ;; std_msgs/String _contact_task
       (send _contact_task :serialize s)
     ;; std_msgs/Float64 _period_shift
       (send _period_shift :serialize s)
     ;; std_msgs/Float64 _period_move
       (send _period_move :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; control_core_msgs/Contact _contact
     (send _contact :deserialize buf ptr-) (incf ptr- (send _contact :serialization-length))
   ;; std_msgs/String _contact_task
     (send _contact_task :deserialize buf ptr-) (incf ptr- (send _contact_task :serialization-length))
   ;; std_msgs/Float64 _period_shift
     (send _period_shift :deserialize buf ptr-) (incf ptr- (send _period_shift :serialization-length))
   ;; std_msgs/Float64 _period_move
     (send _period_move :deserialize buf ptr-) (incf ptr- (send _period_move :serialization-length))
   ;;
   self)
  )

(setf (get behavior_msgs::ChangeContactGoal :md5sum-) "c367d756141e2b8f1da4bca88b2ffb4b")
(setf (get behavior_msgs::ChangeContactGoal :datatype-) "behavior_msgs/ChangeContactGoal")
(setf (get behavior_msgs::ChangeContactGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
control_core_msgs/Contact contact
std_msgs/String contact_task
std_msgs/Float64 period_shift
std_msgs/Float64 period_move

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
MSG: std_msgs/String
string data

")



(provide :behavior_msgs/ChangeContactGoal "c367d756141e2b8f1da4bca88b2ffb4b")

