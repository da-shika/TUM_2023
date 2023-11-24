;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::JointStateStamped)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'JointStateStamped (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::JOINTSTATESTAMPED")
  (make-package "CONTROL_CORE_MSGS::JOINTSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude JointStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_core_msgs::JointStateStamped
  :super ros::object
  :slots (_header _state ))

(defmethod control_core_msgs::JointStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) (instance control_core_msgs::JointState :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state __state)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&rest __state)
   (if (keywordp (car __state))
       (send* _state __state)
     (progn
       (if __state (setq _state (car __state)))
       _state)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; control_core_msgs/JointState _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; control_core_msgs/JointState _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; control_core_msgs/JointState _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::JointStateStamped :md5sum-) "6bec66d356b4dc145ca7abf84a28bd9b")
(setf (get control_core_msgs::JointStateStamped :datatype-) "control_core_msgs/JointStateStamped")
(setf (get control_core_msgs::JointStateStamped :definition-)
      "std_msgs/Header header
JointState state
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



(provide :control_core_msgs/JointStateStamped "6bec66d356b4dc145ca7abf84a28bd9b")


