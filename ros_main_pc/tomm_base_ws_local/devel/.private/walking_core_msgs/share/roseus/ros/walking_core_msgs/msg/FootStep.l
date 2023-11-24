;; Auto-generated. Do not edit!


(when (boundp 'walking_core_msgs::FootStep)
  (if (not (find-package "WALKING_CORE_MSGS"))
    (make-package "WALKING_CORE_MSGS"))
  (shadow 'FootStep (find-package "WALKING_CORE_MSGS")))
(unless (find-package "WALKING_CORE_MSGS::FOOTSTEP")
  (make-package "WALKING_CORE_MSGS::FOOTSTEP"))

(in-package "ROS")
;;//! \htmlinclude FootStep.msg.html
(if (not (find-package "CONTROL_CORE_MSGS"))
  (ros::roseus-add-msgs "control_core_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass walking_core_msgs::FootStep
  :super ros::object
  :slots (_contact _body_id _final_step _n_step ))

(defmethod walking_core_msgs::FootStep
  (:init
   (&key
    ((:contact __contact) (instance control_core_msgs::Contact :init))
    ((:body_id __body_id) (instance std_msgs::Int64 :init))
    ((:final_step __final_step) (instance std_msgs::Bool :init))
    ((:n_step __n_step) (instance std_msgs::Int64 :init))
    )
   (send-super :init)
   (setq _contact __contact)
   (setq _body_id __body_id)
   (setq _final_step __final_step)
   (setq _n_step __n_step)
   self)
  (:contact
   (&rest __contact)
   (if (keywordp (car __contact))
       (send* _contact __contact)
     (progn
       (if __contact (setq _contact (car __contact)))
       _contact)))
  (:body_id
   (&rest __body_id)
   (if (keywordp (car __body_id))
       (send* _body_id __body_id)
     (progn
       (if __body_id (setq _body_id (car __body_id)))
       _body_id)))
  (:final_step
   (&rest __final_step)
   (if (keywordp (car __final_step))
       (send* _final_step __final_step)
     (progn
       (if __final_step (setq _final_step (car __final_step)))
       _final_step)))
  (:n_step
   (&rest __n_step)
   (if (keywordp (car __n_step))
       (send* _n_step __n_step)
     (progn
       (if __n_step (setq _n_step (car __n_step)))
       _n_step)))
  (:serialization-length
   ()
   (+
    ;; control_core_msgs/Contact _contact
    (send _contact :serialization-length)
    ;; std_msgs/Int64 _body_id
    (send _body_id :serialization-length)
    ;; std_msgs/Bool _final_step
    (send _final_step :serialization-length)
    ;; std_msgs/Int64 _n_step
    (send _n_step :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; control_core_msgs/Contact _contact
       (send _contact :serialize s)
     ;; std_msgs/Int64 _body_id
       (send _body_id :serialize s)
     ;; std_msgs/Bool _final_step
       (send _final_step :serialize s)
     ;; std_msgs/Int64 _n_step
       (send _n_step :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; control_core_msgs/Contact _contact
     (send _contact :deserialize buf ptr-) (incf ptr- (send _contact :serialization-length))
   ;; std_msgs/Int64 _body_id
     (send _body_id :deserialize buf ptr-) (incf ptr- (send _body_id :serialization-length))
   ;; std_msgs/Bool _final_step
     (send _final_step :deserialize buf ptr-) (incf ptr- (send _final_step :serialization-length))
   ;; std_msgs/Int64 _n_step
     (send _n_step :deserialize buf ptr-) (incf ptr- (send _n_step :serialization-length))
   ;;
   self)
  )

(setf (get walking_core_msgs::FootStep :md5sum-) "9d0f09bb8a0492982eab7b79dd33028a")
(setf (get walking_core_msgs::FootStep :datatype-) "walking_core_msgs/FootStep")
(setf (get walking_core_msgs::FootStep :definition-)
      "control_core_msgs/Contact contact
std_msgs/Int64 body_id
std_msgs/Bool final_step
std_msgs/Int64 n_step
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
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :walking_core_msgs/FootStep "9d0f09bb8a0492982eab7b79dd33028a")


