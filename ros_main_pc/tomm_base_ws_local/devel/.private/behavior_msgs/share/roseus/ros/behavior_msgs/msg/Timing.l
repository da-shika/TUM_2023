;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::Timing)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'Timing (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::TIMING")
  (make-package "BEHAVIOR_MSGS::TIMING"))

(in-package "ROS")
;;//! \htmlinclude Timing.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass behavior_msgs::Timing
  :super ros::object
  :slots (_control_loop_dur _solver_dur _geometry_dur ))

(defmethod behavior_msgs::Timing
  (:init
   (&key
    ((:control_loop_dur __control_loop_dur) (instance std_msgs::Int64 :init))
    ((:solver_dur __solver_dur) (instance std_msgs::Int64 :init))
    ((:geometry_dur __geometry_dur) (instance std_msgs::Int64 :init))
    )
   (send-super :init)
   (setq _control_loop_dur __control_loop_dur)
   (setq _solver_dur __solver_dur)
   (setq _geometry_dur __geometry_dur)
   self)
  (:control_loop_dur
   (&rest __control_loop_dur)
   (if (keywordp (car __control_loop_dur))
       (send* _control_loop_dur __control_loop_dur)
     (progn
       (if __control_loop_dur (setq _control_loop_dur (car __control_loop_dur)))
       _control_loop_dur)))
  (:solver_dur
   (&rest __solver_dur)
   (if (keywordp (car __solver_dur))
       (send* _solver_dur __solver_dur)
     (progn
       (if __solver_dur (setq _solver_dur (car __solver_dur)))
       _solver_dur)))
  (:geometry_dur
   (&rest __geometry_dur)
   (if (keywordp (car __geometry_dur))
       (send* _geometry_dur __geometry_dur)
     (progn
       (if __geometry_dur (setq _geometry_dur (car __geometry_dur)))
       _geometry_dur)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int64 _control_loop_dur
    (send _control_loop_dur :serialization-length)
    ;; std_msgs/Int64 _solver_dur
    (send _solver_dur :serialization-length)
    ;; std_msgs/Int64 _geometry_dur
    (send _geometry_dur :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int64 _control_loop_dur
       (send _control_loop_dur :serialize s)
     ;; std_msgs/Int64 _solver_dur
       (send _solver_dur :serialize s)
     ;; std_msgs/Int64 _geometry_dur
       (send _geometry_dur :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int64 _control_loop_dur
     (send _control_loop_dur :deserialize buf ptr-) (incf ptr- (send _control_loop_dur :serialization-length))
   ;; std_msgs/Int64 _solver_dur
     (send _solver_dur :deserialize buf ptr-) (incf ptr- (send _solver_dur :serialization-length))
   ;; std_msgs/Int64 _geometry_dur
     (send _geometry_dur :deserialize buf ptr-) (incf ptr- (send _geometry_dur :serialization-length))
   ;;
   self)
  )

(setf (get behavior_msgs::Timing :md5sum-) "860a4c96fe2b2a5b2000e2a96ca33c2c")
(setf (get behavior_msgs::Timing :datatype-) "behavior_msgs/Timing")
(setf (get behavior_msgs::Timing :definition-)
      "std_msgs/Int64 control_loop_dur
std_msgs/Int64 solver_dur
std_msgs/Int64 geometry_dur
================================================================================
MSG: std_msgs/Int64
int64 data
")



(provide :behavior_msgs/Timing "860a4c96fe2b2a5b2000e2a96ca33c2c")


