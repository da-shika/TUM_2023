;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::JointStateTrajectory)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'JointStateTrajectory (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::JOINTSTATETRAJECTORY")
  (make-package "CONTROL_CORE_MSGS::JOINTSTATETRAJECTORY"))

(in-package "ROS")
;;//! \htmlinclude JointStateTrajectory.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_core_msgs::JointStateTrajectory
  :super ros::object
  :slots (_header _time _trajectory ))

(defmethod control_core_msgs::JointStateTrajectory
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:trajectory __trajectory) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _time __time)
   (setq _trajectory __trajectory)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:trajectory
   (&rest __trajectory)
   (if (keywordp (car __trajectory))
       (send* _trajectory __trajectory)
     (progn
       (if __trajectory (setq _trajectory (car __trajectory)))
       _trajectory)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _time
    (* 8    (length _time)) 4
    ;; control_core_msgs/JointState[] _trajectory
    (apply #'+ (send-all _trajectory :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _time
     (write-long (length _time) s)
     (dotimes (i (length _time))
       (sys::poke (elt _time i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; control_core_msgs/JointState[] _trajectory
     (write-long (length _trajectory) s)
     (dolist (elem _trajectory)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _time
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _time (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _time i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; control_core_msgs/JointState[] _trajectory
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _trajectory (let (r) (dotimes (i n) (push (instance control_core_msgs::JointState :init) r)) r))
     (dolist (elem- _trajectory)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get control_core_msgs::JointStateTrajectory :md5sum-) "ad80bd01b1a3c57c74c1c776dde6b612")
(setf (get control_core_msgs::JointStateTrajectory :datatype-) "control_core_msgs/JointStateTrajectory")
(setf (get control_core_msgs::JointStateTrajectory :definition-)
      "std_msgs/Header header
float64[] time
JointState[] trajectory
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
MSG: control_core_msgs/JointState
Vector position
Vector velocity
Vector acceleration
================================================================================
MSG: control_core_msgs/Vector
float64[] data
")



(provide :control_core_msgs/JointStateTrajectory "ad80bd01b1a3c57c74c1c776dde6b612")


