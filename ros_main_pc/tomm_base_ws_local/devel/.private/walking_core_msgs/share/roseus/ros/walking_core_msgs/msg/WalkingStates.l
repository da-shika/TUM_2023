;; Auto-generated. Do not edit!


(when (boundp 'walking_core_msgs::WalkingStates)
  (if (not (find-package "WALKING_CORE_MSGS"))
    (make-package "WALKING_CORE_MSGS"))
  (shadow 'WalkingStates (find-package "WALKING_CORE_MSGS")))
(unless (find-package "WALKING_CORE_MSGS::WALKINGSTATES")
  (make-package "WALKING_CORE_MSGS::WALKINGSTATES"))

(in-package "ROS")
;;//! \htmlinclude WalkingStates.msg.html
(if (not (find-package "CONTROL_CORE_MSGS"))
  (ros::roseus-add-msgs "control_core_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass walking_core_msgs::WalkingStates
  :super ros::object
  :slots (_header _phase _stance_foot _swing_foot _elapsed _left_foot_ratio _cur_step _target_step _next_step ))

(defmethod walking_core_msgs::WalkingStates
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:phase __phase) (instance walking_core_msgs::WalkingPhase :init))
    ((:stance_foot __stance_foot) (instance control_core_msgs::BodyId :init))
    ((:swing_foot __swing_foot) (instance control_core_msgs::BodyId :init))
    ((:elapsed __elapsed) (instance std_msgs::Float64 :init))
    ((:left_foot_ratio __left_foot_ratio) (instance std_msgs::Float64 :init))
    ((:cur_step __cur_step) (instance walking_core_msgs::FootStep :init))
    ((:target_step __target_step) (instance walking_core_msgs::FootStep :init))
    ((:next_step __next_step) (instance walking_core_msgs::FootStep :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _phase __phase)
   (setq _stance_foot __stance_foot)
   (setq _swing_foot __swing_foot)
   (setq _elapsed __elapsed)
   (setq _left_foot_ratio __left_foot_ratio)
   (setq _cur_step __cur_step)
   (setq _target_step __target_step)
   (setq _next_step __next_step)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:phase
   (&rest __phase)
   (if (keywordp (car __phase))
       (send* _phase __phase)
     (progn
       (if __phase (setq _phase (car __phase)))
       _phase)))
  (:stance_foot
   (&rest __stance_foot)
   (if (keywordp (car __stance_foot))
       (send* _stance_foot __stance_foot)
     (progn
       (if __stance_foot (setq _stance_foot (car __stance_foot)))
       _stance_foot)))
  (:swing_foot
   (&rest __swing_foot)
   (if (keywordp (car __swing_foot))
       (send* _swing_foot __swing_foot)
     (progn
       (if __swing_foot (setq _swing_foot (car __swing_foot)))
       _swing_foot)))
  (:elapsed
   (&rest __elapsed)
   (if (keywordp (car __elapsed))
       (send* _elapsed __elapsed)
     (progn
       (if __elapsed (setq _elapsed (car __elapsed)))
       _elapsed)))
  (:left_foot_ratio
   (&rest __left_foot_ratio)
   (if (keywordp (car __left_foot_ratio))
       (send* _left_foot_ratio __left_foot_ratio)
     (progn
       (if __left_foot_ratio (setq _left_foot_ratio (car __left_foot_ratio)))
       _left_foot_ratio)))
  (:cur_step
   (&rest __cur_step)
   (if (keywordp (car __cur_step))
       (send* _cur_step __cur_step)
     (progn
       (if __cur_step (setq _cur_step (car __cur_step)))
       _cur_step)))
  (:target_step
   (&rest __target_step)
   (if (keywordp (car __target_step))
       (send* _target_step __target_step)
     (progn
       (if __target_step (setq _target_step (car __target_step)))
       _target_step)))
  (:next_step
   (&rest __next_step)
   (if (keywordp (car __next_step))
       (send* _next_step __next_step)
     (progn
       (if __next_step (setq _next_step (car __next_step)))
       _next_step)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; walking_core_msgs/WalkingPhase _phase
    (send _phase :serialization-length)
    ;; control_core_msgs/BodyId _stance_foot
    (send _stance_foot :serialization-length)
    ;; control_core_msgs/BodyId _swing_foot
    (send _swing_foot :serialization-length)
    ;; std_msgs/Float64 _elapsed
    (send _elapsed :serialization-length)
    ;; std_msgs/Float64 _left_foot_ratio
    (send _left_foot_ratio :serialization-length)
    ;; walking_core_msgs/FootStep _cur_step
    (send _cur_step :serialization-length)
    ;; walking_core_msgs/FootStep _target_step
    (send _target_step :serialization-length)
    ;; walking_core_msgs/FootStep _next_step
    (send _next_step :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; walking_core_msgs/WalkingPhase _phase
       (send _phase :serialize s)
     ;; control_core_msgs/BodyId _stance_foot
       (send _stance_foot :serialize s)
     ;; control_core_msgs/BodyId _swing_foot
       (send _swing_foot :serialize s)
     ;; std_msgs/Float64 _elapsed
       (send _elapsed :serialize s)
     ;; std_msgs/Float64 _left_foot_ratio
       (send _left_foot_ratio :serialize s)
     ;; walking_core_msgs/FootStep _cur_step
       (send _cur_step :serialize s)
     ;; walking_core_msgs/FootStep _target_step
       (send _target_step :serialize s)
     ;; walking_core_msgs/FootStep _next_step
       (send _next_step :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; walking_core_msgs/WalkingPhase _phase
     (send _phase :deserialize buf ptr-) (incf ptr- (send _phase :serialization-length))
   ;; control_core_msgs/BodyId _stance_foot
     (send _stance_foot :deserialize buf ptr-) (incf ptr- (send _stance_foot :serialization-length))
   ;; control_core_msgs/BodyId _swing_foot
     (send _swing_foot :deserialize buf ptr-) (incf ptr- (send _swing_foot :serialization-length))
   ;; std_msgs/Float64 _elapsed
     (send _elapsed :deserialize buf ptr-) (incf ptr- (send _elapsed :serialization-length))
   ;; std_msgs/Float64 _left_foot_ratio
     (send _left_foot_ratio :deserialize buf ptr-) (incf ptr- (send _left_foot_ratio :serialization-length))
   ;; walking_core_msgs/FootStep _cur_step
     (send _cur_step :deserialize buf ptr-) (incf ptr- (send _cur_step :serialization-length))
   ;; walking_core_msgs/FootStep _target_step
     (send _target_step :deserialize buf ptr-) (incf ptr- (send _target_step :serialization-length))
   ;; walking_core_msgs/FootStep _next_step
     (send _next_step :deserialize buf ptr-) (incf ptr- (send _next_step :serialization-length))
   ;;
   self)
  )

(setf (get walking_core_msgs::WalkingStates :md5sum-) "e12756d011ab43b51ff2b08d69aadcb4")
(setf (get walking_core_msgs::WalkingStates :datatype-) "walking_core_msgs/WalkingStates")
(setf (get walking_core_msgs::WalkingStates :definition-)
      "std_msgs/Header header
WalkingPhase phase
control_core_msgs/BodyId stance_foot
control_core_msgs/BodyId swing_foot
std_msgs/Float64 elapsed
std_msgs/Float64 left_foot_ratio
FootStep cur_step
FootStep target_step
FootStep next_step
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
MSG: walking_core_msgs/WalkingPhase
uint8 PHASE_STANCE=0
uint8 PHASE_DOUBLESUPPORT=1
uint8 PHASE_SINGLESUPPORT=2
uint8 phase
================================================================================
MSG: control_core_msgs/BodyId
uint8 ID_LEFT_FOOT=0
uint8 ID_RIGHT_FOOT=1
uint8 ID_LEFT_HAND=2
uint8 ID_RIGHT_HAND=3
uint8 id
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: walking_core_msgs/FootStep
control_core_msgs/Contact contact
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
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :walking_core_msgs/WalkingStates "e12756d011ab43b51ff2b08d69aadcb4")


