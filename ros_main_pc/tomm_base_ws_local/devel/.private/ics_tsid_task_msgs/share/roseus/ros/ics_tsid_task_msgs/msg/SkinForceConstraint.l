;; Auto-generated. Do not edit!


(when (boundp 'ics_tsid_task_msgs::SkinForceConstraint)
  (if (not (find-package "ICS_TSID_TASK_MSGS"))
    (make-package "ICS_TSID_TASK_MSGS"))
  (shadow 'SkinForceConstraint (find-package "ICS_TSID_TASK_MSGS")))
(unless (find-package "ICS_TSID_TASK_MSGS::SKINFORCECONSTRAINT")
  (make-package "ICS_TSID_TASK_MSGS::SKINFORCECONSTRAINT"))

(in-package "ROS")
;;//! \htmlinclude SkinForceConstraint.msg.html
(if (not (find-package "CONTROL_CORE_MSGS"))
  (ros::roseus-add-msgs "control_core_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ics_tsid_task_msgs::SkinForceConstraint
  :super ros::object
  :slots (_dur _num_active_ieq _num_violated_ieq _max_force _min_distance _max_proximity _cell_nums _forces _vel_cmds _acc_limits _weight _a_relaxed ))

(defmethod ics_tsid_task_msgs::SkinForceConstraint
  (:init
   (&key
    ((:dur __dur) (instance std_msgs::Int64 :init))
    ((:num_active_ieq __num_active_ieq) (instance std_msgs::Int32 :init))
    ((:num_violated_ieq __num_violated_ieq) (instance std_msgs::Int32 :init))
    ((:max_force __max_force) (instance std_msgs::Float64 :init))
    ((:min_distance __min_distance) (instance std_msgs::Float64 :init))
    ((:max_proximity __max_proximity) (instance std_msgs::Float64 :init))
    ((:cell_nums __cell_nums) (instance control_core_msgs::Vector :init))
    ((:forces __forces) (instance control_core_msgs::Vector :init))
    ((:vel_cmds __vel_cmds) (instance control_core_msgs::Vector :init))
    ((:acc_limits __acc_limits) (instance control_core_msgs::Vector :init))
    ((:weight __weight) (instance std_msgs::Float64 :init))
    ((:a_relaxed __a_relaxed) (instance control_core_msgs::Vector :init))
    )
   (send-super :init)
   (setq _dur __dur)
   (setq _num_active_ieq __num_active_ieq)
   (setq _num_violated_ieq __num_violated_ieq)
   (setq _max_force __max_force)
   (setq _min_distance __min_distance)
   (setq _max_proximity __max_proximity)
   (setq _cell_nums __cell_nums)
   (setq _forces __forces)
   (setq _vel_cmds __vel_cmds)
   (setq _acc_limits __acc_limits)
   (setq _weight __weight)
   (setq _a_relaxed __a_relaxed)
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
  (:max_force
   (&rest __max_force)
   (if (keywordp (car __max_force))
       (send* _max_force __max_force)
     (progn
       (if __max_force (setq _max_force (car __max_force)))
       _max_force)))
  (:min_distance
   (&rest __min_distance)
   (if (keywordp (car __min_distance))
       (send* _min_distance __min_distance)
     (progn
       (if __min_distance (setq _min_distance (car __min_distance)))
       _min_distance)))
  (:max_proximity
   (&rest __max_proximity)
   (if (keywordp (car __max_proximity))
       (send* _max_proximity __max_proximity)
     (progn
       (if __max_proximity (setq _max_proximity (car __max_proximity)))
       _max_proximity)))
  (:cell_nums
   (&rest __cell_nums)
   (if (keywordp (car __cell_nums))
       (send* _cell_nums __cell_nums)
     (progn
       (if __cell_nums (setq _cell_nums (car __cell_nums)))
       _cell_nums)))
  (:forces
   (&rest __forces)
   (if (keywordp (car __forces))
       (send* _forces __forces)
     (progn
       (if __forces (setq _forces (car __forces)))
       _forces)))
  (:vel_cmds
   (&rest __vel_cmds)
   (if (keywordp (car __vel_cmds))
       (send* _vel_cmds __vel_cmds)
     (progn
       (if __vel_cmds (setq _vel_cmds (car __vel_cmds)))
       _vel_cmds)))
  (:acc_limits
   (&rest __acc_limits)
   (if (keywordp (car __acc_limits))
       (send* _acc_limits __acc_limits)
     (progn
       (if __acc_limits (setq _acc_limits (car __acc_limits)))
       _acc_limits)))
  (:weight
   (&rest __weight)
   (if (keywordp (car __weight))
       (send* _weight __weight)
     (progn
       (if __weight (setq _weight (car __weight)))
       _weight)))
  (:a_relaxed
   (&rest __a_relaxed)
   (if (keywordp (car __a_relaxed))
       (send* _a_relaxed __a_relaxed)
     (progn
       (if __a_relaxed (setq _a_relaxed (car __a_relaxed)))
       _a_relaxed)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int64 _dur
    (send _dur :serialization-length)
    ;; std_msgs/Int32 _num_active_ieq
    (send _num_active_ieq :serialization-length)
    ;; std_msgs/Int32 _num_violated_ieq
    (send _num_violated_ieq :serialization-length)
    ;; std_msgs/Float64 _max_force
    (send _max_force :serialization-length)
    ;; std_msgs/Float64 _min_distance
    (send _min_distance :serialization-length)
    ;; std_msgs/Float64 _max_proximity
    (send _max_proximity :serialization-length)
    ;; control_core_msgs/Vector _cell_nums
    (send _cell_nums :serialization-length)
    ;; control_core_msgs/Vector _forces
    (send _forces :serialization-length)
    ;; control_core_msgs/Vector _vel_cmds
    (send _vel_cmds :serialization-length)
    ;; control_core_msgs/Vector _acc_limits
    (send _acc_limits :serialization-length)
    ;; std_msgs/Float64 _weight
    (send _weight :serialization-length)
    ;; control_core_msgs/Vector _a_relaxed
    (send _a_relaxed :serialization-length)
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
     ;; std_msgs/Float64 _max_force
       (send _max_force :serialize s)
     ;; std_msgs/Float64 _min_distance
       (send _min_distance :serialize s)
     ;; std_msgs/Float64 _max_proximity
       (send _max_proximity :serialize s)
     ;; control_core_msgs/Vector _cell_nums
       (send _cell_nums :serialize s)
     ;; control_core_msgs/Vector _forces
       (send _forces :serialize s)
     ;; control_core_msgs/Vector _vel_cmds
       (send _vel_cmds :serialize s)
     ;; control_core_msgs/Vector _acc_limits
       (send _acc_limits :serialize s)
     ;; std_msgs/Float64 _weight
       (send _weight :serialize s)
     ;; control_core_msgs/Vector _a_relaxed
       (send _a_relaxed :serialize s)
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
   ;; std_msgs/Float64 _max_force
     (send _max_force :deserialize buf ptr-) (incf ptr- (send _max_force :serialization-length))
   ;; std_msgs/Float64 _min_distance
     (send _min_distance :deserialize buf ptr-) (incf ptr- (send _min_distance :serialization-length))
   ;; std_msgs/Float64 _max_proximity
     (send _max_proximity :deserialize buf ptr-) (incf ptr- (send _max_proximity :serialization-length))
   ;; control_core_msgs/Vector _cell_nums
     (send _cell_nums :deserialize buf ptr-) (incf ptr- (send _cell_nums :serialization-length))
   ;; control_core_msgs/Vector _forces
     (send _forces :deserialize buf ptr-) (incf ptr- (send _forces :serialization-length))
   ;; control_core_msgs/Vector _vel_cmds
     (send _vel_cmds :deserialize buf ptr-) (incf ptr- (send _vel_cmds :serialization-length))
   ;; control_core_msgs/Vector _acc_limits
     (send _acc_limits :deserialize buf ptr-) (incf ptr- (send _acc_limits :serialization-length))
   ;; std_msgs/Float64 _weight
     (send _weight :deserialize buf ptr-) (incf ptr- (send _weight :serialization-length))
   ;; control_core_msgs/Vector _a_relaxed
     (send _a_relaxed :deserialize buf ptr-) (incf ptr- (send _a_relaxed :serialization-length))
   ;;
   self)
  )

(setf (get ics_tsid_task_msgs::SkinForceConstraint :md5sum-) "997ea5adb62ebb597e70c90a3e090602")
(setf (get ics_tsid_task_msgs::SkinForceConstraint :datatype-) "ics_tsid_task_msgs/SkinForceConstraint")
(setf (get ics_tsid_task_msgs::SkinForceConstraint :definition-)
      "std_msgs/Int64 dur
std_msgs/Int32 num_active_ieq
std_msgs/Int32 num_violated_ieq

std_msgs/Float64 max_force
std_msgs/Float64 min_distance
std_msgs/Float64 max_proximity

control_core_msgs/Vector cell_nums
control_core_msgs/Vector forces
control_core_msgs/Vector vel_cmds
control_core_msgs/Vector acc_limits

std_msgs/Float64 weight
control_core_msgs/Vector a_relaxed

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



(provide :ics_tsid_task_msgs/SkinForceConstraint "997ea5adb62ebb597e70c90a3e090602")


