;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::LinearStateStamped)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'LinearStateStamped (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::LINEARSTATESTAMPED")
  (make-package "CONTROL_CORE_MSGS::LINEARSTATESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude LinearStateStamped.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass control_core_msgs::LinearStateStamped
  :super ros::object
  :slots (_header _state ))

(defmethod control_core_msgs::LinearStateStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) (instance control_core_msgs::LinearState :init))
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
    ;; control_core_msgs/LinearState _state
    (send _state :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; control_core_msgs/LinearState _state
       (send _state :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; control_core_msgs/LinearState _state
     (send _state :deserialize buf ptr-) (incf ptr- (send _state :serialization-length))
   ;;
   self)
  )

(setf (get control_core_msgs::LinearStateStamped :md5sum-) "b1b7305065501d4951b8d88b820348e4")
(setf (get control_core_msgs::LinearStateStamped :datatype-) "control_core_msgs/LinearStateStamped")
(setf (get control_core_msgs::LinearStateStamped :definition-)
      "std_msgs/Header header
LinearState state
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
MSG: control_core_msgs/LinearState
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

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
")



(provide :control_core_msgs/LinearStateStamped "b1b7305065501d4951b8d88b820348e4")


