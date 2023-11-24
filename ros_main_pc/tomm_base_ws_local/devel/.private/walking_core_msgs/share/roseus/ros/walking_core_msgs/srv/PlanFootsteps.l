;; Auto-generated. Do not edit!


(when (boundp 'walking_core_msgs::PlanFootsteps)
  (if (not (find-package "WALKING_CORE_MSGS"))
    (make-package "WALKING_CORE_MSGS"))
  (shadow 'PlanFootsteps (find-package "WALKING_CORE_MSGS")))
(unless (find-package "WALKING_CORE_MSGS::PLANFOOTSTEPS")
  (make-package "WALKING_CORE_MSGS::PLANFOOTSTEPS"))
(unless (find-package "WALKING_CORE_MSGS::PLANFOOTSTEPSREQUEST")
  (make-package "WALKING_CORE_MSGS::PLANFOOTSTEPSREQUEST"))
(unless (find-package "WALKING_CORE_MSGS::PLANFOOTSTEPSRESPONSE")
  (make-package "WALKING_CORE_MSGS::PLANFOOTSTEPSRESPONSE"))

(in-package "ROS")

(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass walking_core_msgs::PlanFootstepsRequest
  :super ros::object
  :slots (_n_steps _length _lateral _angle ))

(defmethod walking_core_msgs::PlanFootstepsRequest
  (:init
   (&key
    ((:n_steps __n_steps) (instance std_msgs::Int64 :init))
    ((:length __length) (instance std_msgs::Float64 :init))
    ((:lateral __lateral) (instance std_msgs::Float64 :init))
    ((:angle __angle) (instance std_msgs::Float64 :init))
    )
   (send-super :init)
   (setq _n_steps __n_steps)
   (setq _length __length)
   (setq _lateral __lateral)
   (setq _angle __angle)
   self)
  (:n_steps
   (&rest __n_steps)
   (if (keywordp (car __n_steps))
       (send* _n_steps __n_steps)
     (progn
       (if __n_steps (setq _n_steps (car __n_steps)))
       _n_steps)))
  (:length
   (&rest __length)
   (if (keywordp (car __length))
       (send* _length __length)
     (progn
       (if __length (setq _length (car __length)))
       _length)))
  (:lateral
   (&rest __lateral)
   (if (keywordp (car __lateral))
       (send* _lateral __lateral)
     (progn
       (if __lateral (setq _lateral (car __lateral)))
       _lateral)))
  (:angle
   (&rest __angle)
   (if (keywordp (car __angle))
       (send* _angle __angle)
     (progn
       (if __angle (setq _angle (car __angle)))
       _angle)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Int64 _n_steps
    (send _n_steps :serialization-length)
    ;; std_msgs/Float64 _length
    (send _length :serialization-length)
    ;; std_msgs/Float64 _lateral
    (send _lateral :serialization-length)
    ;; std_msgs/Float64 _angle
    (send _angle :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Int64 _n_steps
       (send _n_steps :serialize s)
     ;; std_msgs/Float64 _length
       (send _length :serialize s)
     ;; std_msgs/Float64 _lateral
       (send _lateral :serialize s)
     ;; std_msgs/Float64 _angle
       (send _angle :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Int64 _n_steps
     (send _n_steps :deserialize buf ptr-) (incf ptr- (send _n_steps :serialization-length))
   ;; std_msgs/Float64 _length
     (send _length :deserialize buf ptr-) (incf ptr- (send _length :serialization-length))
   ;; std_msgs/Float64 _lateral
     (send _lateral :deserialize buf ptr-) (incf ptr- (send _lateral :serialization-length))
   ;; std_msgs/Float64 _angle
     (send _angle :deserialize buf ptr-) (incf ptr- (send _angle :serialization-length))
   ;;
   self)
  )

(defclass walking_core_msgs::PlanFootstepsResponse
  :super ros::object
  :slots (_success ))

(defmethod walking_core_msgs::PlanFootstepsResponse
  (:init
   (&key
    ((:success __success) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&rest __success)
   (if (keywordp (car __success))
       (send* _success __success)
     (progn
       (if __success (setq _success (car __success)))
       _success)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Bool _success
    (send _success :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Bool _success
       (send _success :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Bool _success
     (send _success :deserialize buf ptr-) (incf ptr- (send _success :serialization-length))
   ;;
   self)
  )

(defclass walking_core_msgs::PlanFootsteps
  :super ros::object
  :slots ())

(setf (get walking_core_msgs::PlanFootsteps :md5sum-) "6b7b78d5ced6128d564af0ca90b3d6e2")
(setf (get walking_core_msgs::PlanFootsteps :datatype-) "walking_core_msgs/PlanFootsteps")
(setf (get walking_core_msgs::PlanFootsteps :request) walking_core_msgs::PlanFootstepsRequest)
(setf (get walking_core_msgs::PlanFootsteps :response) walking_core_msgs::PlanFootstepsResponse)

(defmethod walking_core_msgs::PlanFootstepsRequest
  (:response () (instance walking_core_msgs::PlanFootstepsResponse :init)))

(setf (get walking_core_msgs::PlanFootstepsRequest :md5sum-) "6b7b78d5ced6128d564af0ca90b3d6e2")
(setf (get walking_core_msgs::PlanFootstepsRequest :datatype-) "walking_core_msgs/PlanFootstepsRequest")
(setf (get walking_core_msgs::PlanFootstepsRequest :definition-)
      "std_msgs/Int64      n_steps
std_msgs/Float64    length
std_msgs/Float64    lateral
std_msgs/Float64    angle

================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Float64
---
std_msgs/Bool success


================================================================================
MSG: std_msgs/Bool
")

(setf (get walking_core_msgs::PlanFootstepsResponse :md5sum-) "6b7b78d5ced6128d564af0ca90b3d6e2")
(setf (get walking_core_msgs::PlanFootstepsResponse :datatype-) "walking_core_msgs/PlanFootstepsResponse")
(setf (get walking_core_msgs::PlanFootstepsResponse :definition-)
      "std_msgs/Int64      n_steps
std_msgs/Float64    length
std_msgs/Float64    lateral
std_msgs/Float64    angle

================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Float64
---
std_msgs/Bool success


================================================================================
MSG: std_msgs/Bool
")



(provide :walking_core_msgs/PlanFootsteps "6b7b78d5ced6128d564af0ca90b3d6e2")


