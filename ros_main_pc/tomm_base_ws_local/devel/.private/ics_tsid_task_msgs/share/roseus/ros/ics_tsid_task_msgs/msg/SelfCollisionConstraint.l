;; Auto-generated. Do not edit!


(when (boundp 'ics_tsid_task_msgs::SelfCollisionConstraint)
  (if (not (find-package "ICS_TSID_TASK_MSGS"))
    (make-package "ICS_TSID_TASK_MSGS"))
  (shadow 'SelfCollisionConstraint (find-package "ICS_TSID_TASK_MSGS")))
(unless (find-package "ICS_TSID_TASK_MSGS::SELFCOLLISIONCONSTRAINT")
  (make-package "ICS_TSID_TASK_MSGS::SELFCOLLISIONCONSTRAINT"))

(in-package "ROS")
;;//! \htmlinclude SelfCollisionConstraint.msg.html
(if (not (find-package "CONTROL_CORE_MSGS"))
  (ros::roseus-add-msgs "control_core_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ics_tsid_task_msgs::SelfCollisionConstraint
  :super ros::object
  :slots (_dur _num_active_ieq _num_violated_ieq _min_distance _distances ))

(defmethod ics_tsid_task_msgs::SelfCollisionConstraint
  (:init
   (&key
    ((:dur __dur) (instance std_msgs::Int64 :init))
    ((:num_active_ieq __num_active_ieq) (instance std_msgs::Int32 :init))
    ((:num_violated_ieq __num_violated_ieq) (instance std_msgs::Int32 :init))
    ((:min_distance __min_distance) (instance std_msgs::Float64 :init))
    ((:distances __distances) (instance control_core_msgs::Vector :init))
    )
   (send-super :init)
   (setq _dur __dur)
   (setq _num_active_ieq __num_active_ieq)
   (setq _num_violated_ieq __num_violated_ieq)
   (setq _min_distance __min_distance)
   (setq _distances __distances)
   self)
  (:dur
   (&rest __dur)
   (if (keywordp (car __dur))
       (send* _dur __dur)
     (progn
       (if __dur (setq _dur (car __dur)))
       _dur)))
  (:num_active_ieq
   (&rest __num_active_ieq)
   (if (keywordp (car __num_active_ieq))
       (send* _num_active_ieq __num_active_ieq)
     (progn
       (if __num_active_ieq (setq _num_active_ieq (car __num_active_ieq)))
       _num_active_ieq)))
  (:num_violated_ieq
   (&rest __num_violated_ieq)
   (if (keywordp (car __num_violated_ieq))
       (send* _num_violated_ieq __num_violated_ieq)
     (progn
       (if __num_violated_ieq (setq _num_violated_ieq (car __num_violated_ieq)))
       _num_violated_ieq)))
  (:min_distance
   (&rest __min_distance)
   (if (keywordp (car __min_distance))
       (send* _min_distance __min_distance)
     (progn
       (if __min_distance (setq _min_distance (car __min_distance)))
       _min_distance)))
  (:distances
   (&rest __distances)
   (if (keywordp (car __distances))
       (send* _distances __distances)
     (progn
       (if __distances (setq _distances (car __distances)))
       _distances)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int64 _dur
    (send _dur :serialization-length)
    ;; std_msgs/Int32 _num_active_ieq
    (send _num_active_ieq :serialization-length)
    ;; std_msgs/Int32 _num_violated_ieq
    (send _num_violated_ieq :serialization-length)
    ;; std_msgs/Float64 _min_distance
    (send _min_distance :serialization-length)
    ;; control_core_msgs/Vector _distances
    (send _distances :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int64 _dur
       (send _dur :serialize s)
     ;; std_msgs/Int32 _num_active_ieq
       (send _num_active_ieq :serialize s)
     ;; std_msgs/Int32 _num_violated_ieq
       (send _num_violated_ieq :serialize s)
     ;; std_msgs/Float64 _min_distance
       (send _min_distance :serialize s)
     ;; control_core_msgs/Vector _distances
       (send _distances :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int64 _dur
     (send _dur :deserialize buf ptr-) (incf ptr- (send _dur :serialization-length))
   ;; std_msgs/Int32 _num_active_ieq
     (send _num_active_ieq :deserialize buf ptr-) (incf ptr- (send _num_active_ieq :serialization-length))
   ;; std_msgs/Int32 _num_violated_ieq
     (send _num_violated_ieq :deserialize buf ptr-) (incf ptr- (send _num_violated_ieq :serialization-length))
   ;; std_msgs/Float64 _min_distance
     (send _min_distance :deserialize buf ptr-) (incf ptr- (send _min_distance :serialization-length))
   ;; control_core_msgs/Vector _distances
     (send _distances :deserialize buf ptr-) (incf ptr- (send _distances :serialization-length))
   ;;
   self)
  )

(setf (get ics_tsid_task_msgs::SelfCollisionConstraint :md5sum-) "f09e82fd510858f1c3f5993df0a68ab2")
(setf (get ics_tsid_task_msgs::SelfCollisionConstraint :datatype-) "ics_tsid_task_msgs/SelfCollisionConstraint")
(setf (get ics_tsid_task_msgs::SelfCollisionConstraint :definition-)
      "std_msgs/Int64 dur
std_msgs/Int32 num_active_ieq
std_msgs/Int32 num_violated_ieq

std_msgs/Float64 min_distance
control_core_msgs/Vector distances
================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Int32
int32 data
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: control_core_msgs/Vector
float64[] data
")



(provide :ics_tsid_task_msgs/SelfCollisionConstraint "f09e82fd510858f1c3f5993df0a68ab2")


