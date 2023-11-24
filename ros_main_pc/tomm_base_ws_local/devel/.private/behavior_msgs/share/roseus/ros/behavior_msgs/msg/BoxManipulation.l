;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::BoxManipulation)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'BoxManipulation (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::BOXMANIPULATION")
  (make-package "BEHAVIOR_MSGS::BOXMANIPULATION"))

(in-package "ROS")
;;//! \htmlinclude BoxManipulation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass behavior_msgs::BoxManipulation
  :super ros::object
  :slots (_header _state _W_r_ref _W_l_ref _W_r_off _W_l_off _W_r_des _W_l_des _W_r_real_skin _W_l_real_skin _W_r_real_ft _W_l_real_ft ))

(defmethod behavior_msgs::BoxManipulation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state __state) 0)
    ((:W_r_ref __W_r_ref) (instance geometry_msgs::Wrench :init))
    ((:W_l_ref __W_l_ref) (instance geometry_msgs::Wrench :init))
    ((:W_r_off __W_r_off) (instance geometry_msgs::Wrench :init))
    ((:W_l_off __W_l_off) (instance geometry_msgs::Wrench :init))
    ((:W_r_des __W_r_des) (instance geometry_msgs::Wrench :init))
    ((:W_l_des __W_l_des) (instance geometry_msgs::Wrench :init))
    ((:W_r_real_skin __W_r_real_skin) (instance geometry_msgs::Wrench :init))
    ((:W_l_real_skin __W_l_real_skin) (instance geometry_msgs::Wrench :init))
    ((:W_r_real_ft __W_r_real_ft) (instance geometry_msgs::Wrench :init))
    ((:W_l_real_ft __W_l_real_ft) (instance geometry_msgs::Wrench :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state (round __state))
   (setq _W_r_ref __W_r_ref)
   (setq _W_l_ref __W_l_ref)
   (setq _W_r_off __W_r_off)
   (setq _W_l_off __W_l_off)
   (setq _W_r_des __W_r_des)
   (setq _W_l_des __W_l_des)
   (setq _W_r_real_skin __W_r_real_skin)
   (setq _W_l_real_skin __W_l_real_skin)
   (setq _W_r_real_ft __W_r_real_ft)
   (setq _W_l_real_ft __W_l_real_ft)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:W_r_ref
   (&rest __W_r_ref)
   (if (keywordp (car __W_r_ref))
       (send* _W_r_ref __W_r_ref)
     (progn
       (if __W_r_ref (setq _W_r_ref (car __W_r_ref)))
       _W_r_ref)))
  (:W_l_ref
   (&rest __W_l_ref)
   (if (keywordp (car __W_l_ref))
       (send* _W_l_ref __W_l_ref)
     (progn
       (if __W_l_ref (setq _W_l_ref (car __W_l_ref)))
       _W_l_ref)))
  (:W_r_off
   (&rest __W_r_off)
   (if (keywordp (car __W_r_off))
       (send* _W_r_off __W_r_off)
     (progn
       (if __W_r_off (setq _W_r_off (car __W_r_off)))
       _W_r_off)))
  (:W_l_off
   (&rest __W_l_off)
   (if (keywordp (car __W_l_off))
       (send* _W_l_off __W_l_off)
     (progn
       (if __W_l_off (setq _W_l_off (car __W_l_off)))
       _W_l_off)))
  (:W_r_des
   (&rest __W_r_des)
   (if (keywordp (car __W_r_des))
       (send* _W_r_des __W_r_des)
     (progn
       (if __W_r_des (setq _W_r_des (car __W_r_des)))
       _W_r_des)))
  (:W_l_des
   (&rest __W_l_des)
   (if (keywordp (car __W_l_des))
       (send* _W_l_des __W_l_des)
     (progn
       (if __W_l_des (setq _W_l_des (car __W_l_des)))
       _W_l_des)))
  (:W_r_real_skin
   (&rest __W_r_real_skin)
   (if (keywordp (car __W_r_real_skin))
       (send* _W_r_real_skin __W_r_real_skin)
     (progn
       (if __W_r_real_skin (setq _W_r_real_skin (car __W_r_real_skin)))
       _W_r_real_skin)))
  (:W_l_real_skin
   (&rest __W_l_real_skin)
   (if (keywordp (car __W_l_real_skin))
       (send* _W_l_real_skin __W_l_real_skin)
     (progn
       (if __W_l_real_skin (setq _W_l_real_skin (car __W_l_real_skin)))
       _W_l_real_skin)))
  (:W_r_real_ft
   (&rest __W_r_real_ft)
   (if (keywordp (car __W_r_real_ft))
       (send* _W_r_real_ft __W_r_real_ft)
     (progn
       (if __W_r_real_ft (setq _W_r_real_ft (car __W_r_real_ft)))
       _W_r_real_ft)))
  (:W_l_real_ft
   (&rest __W_l_real_ft)
   (if (keywordp (car __W_l_real_ft))
       (send* _W_l_real_ft __W_l_real_ft)
     (progn
       (if __W_l_real_ft (setq _W_l_real_ft (car __W_l_real_ft)))
       _W_l_real_ft)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _state
    1
    ;; geometry_msgs/Wrench _W_r_ref
    (send _W_r_ref :serialization-length)
    ;; geometry_msgs/Wrench _W_l_ref
    (send _W_l_ref :serialization-length)
    ;; geometry_msgs/Wrench _W_r_off
    (send _W_r_off :serialization-length)
    ;; geometry_msgs/Wrench _W_l_off
    (send _W_l_off :serialization-length)
    ;; geometry_msgs/Wrench _W_r_des
    (send _W_r_des :serialization-length)
    ;; geometry_msgs/Wrench _W_l_des
    (send _W_l_des :serialization-length)
    ;; geometry_msgs/Wrench _W_r_real_skin
    (send _W_r_real_skin :serialization-length)
    ;; geometry_msgs/Wrench _W_l_real_skin
    (send _W_l_real_skin :serialization-length)
    ;; geometry_msgs/Wrench _W_r_real_ft
    (send _W_r_real_ft :serialization-length)
    ;; geometry_msgs/Wrench _W_l_real_ft
    (send _W_l_real_ft :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _state
       (write-byte _state s)
     ;; geometry_msgs/Wrench _W_r_ref
       (send _W_r_ref :serialize s)
     ;; geometry_msgs/Wrench _W_l_ref
       (send _W_l_ref :serialize s)
     ;; geometry_msgs/Wrench _W_r_off
       (send _W_r_off :serialize s)
     ;; geometry_msgs/Wrench _W_l_off
       (send _W_l_off :serialize s)
     ;; geometry_msgs/Wrench _W_r_des
       (send _W_r_des :serialize s)
     ;; geometry_msgs/Wrench _W_l_des
       (send _W_l_des :serialize s)
     ;; geometry_msgs/Wrench _W_r_real_skin
       (send _W_r_real_skin :serialize s)
     ;; geometry_msgs/Wrench _W_l_real_skin
       (send _W_l_real_skin :serialize s)
     ;; geometry_msgs/Wrench _W_r_real_ft
       (send _W_r_real_ft :serialize s)
     ;; geometry_msgs/Wrench _W_l_real_ft
       (send _W_l_real_ft :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _state
     (setq _state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Wrench _W_r_ref
     (send _W_r_ref :deserialize buf ptr-) (incf ptr- (send _W_r_ref :serialization-length))
   ;; geometry_msgs/Wrench _W_l_ref
     (send _W_l_ref :deserialize buf ptr-) (incf ptr- (send _W_l_ref :serialization-length))
   ;; geometry_msgs/Wrench _W_r_off
     (send _W_r_off :deserialize buf ptr-) (incf ptr- (send _W_r_off :serialization-length))
   ;; geometry_msgs/Wrench _W_l_off
     (send _W_l_off :deserialize buf ptr-) (incf ptr- (send _W_l_off :serialization-length))
   ;; geometry_msgs/Wrench _W_r_des
     (send _W_r_des :deserialize buf ptr-) (incf ptr- (send _W_r_des :serialization-length))
   ;; geometry_msgs/Wrench _W_l_des
     (send _W_l_des :deserialize buf ptr-) (incf ptr- (send _W_l_des :serialization-length))
   ;; geometry_msgs/Wrench _W_r_real_skin
     (send _W_r_real_skin :deserialize buf ptr-) (incf ptr- (send _W_r_real_skin :serialization-length))
   ;; geometry_msgs/Wrench _W_l_real_skin
     (send _W_l_real_skin :deserialize buf ptr-) (incf ptr- (send _W_l_real_skin :serialization-length))
   ;; geometry_msgs/Wrench _W_r_real_ft
     (send _W_r_real_ft :deserialize buf ptr-) (incf ptr- (send _W_r_real_ft :serialization-length))
   ;; geometry_msgs/Wrench _W_l_real_ft
     (send _W_l_real_ft :deserialize buf ptr-) (incf ptr- (send _W_l_real_ft :serialization-length))
   ;;
   self)
  )

(setf (get behavior_msgs::BoxManipulation :md5sum-) "1764b3b821c4db67f223b0ac4257c2d9")
(setf (get behavior_msgs::BoxManipulation :datatype-) "behavior_msgs/BoxManipulation")
(setf (get behavior_msgs::BoxManipulation :definition-)
      "std_msgs/Header header

uint8 state

geometry_msgs/Wrench W_r_ref
geometry_msgs/Wrench W_l_ref

geometry_msgs/Wrench W_r_off
geometry_msgs/Wrench W_l_off

geometry_msgs/Wrench W_r_des
geometry_msgs/Wrench W_l_des

geometry_msgs/Wrench W_r_real_skin
geometry_msgs/Wrench W_l_real_skin

geometry_msgs/Wrench W_r_real_ft
geometry_msgs/Wrench W_l_real_ft


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
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

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



(provide :behavior_msgs/BoxManipulation "1764b3b821c4db67f223b0ac4257c2d9")


