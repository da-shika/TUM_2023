;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::BodyId)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'BodyId (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::BODYID")
  (make-package "CONTROL_CORE_MSGS::BODYID"))

(in-package "ROS")
;;//! \htmlinclude BodyId.msg.html


(intern "*ID_LEFT_FOOT*" (find-package "CONTROL_CORE_MSGS::BODYID"))
(shadow '*ID_LEFT_FOOT* (find-package "CONTROL_CORE_MSGS::BODYID"))
(defconstant control_core_msgs::BodyId::*ID_LEFT_FOOT* 0)
(intern "*ID_RIGHT_FOOT*" (find-package "CONTROL_CORE_MSGS::BODYID"))
(shadow '*ID_RIGHT_FOOT* (find-package "CONTROL_CORE_MSGS::BODYID"))
(defconstant control_core_msgs::BodyId::*ID_RIGHT_FOOT* 1)
(intern "*ID_LEFT_HAND*" (find-package "CONTROL_CORE_MSGS::BODYID"))
(shadow '*ID_LEFT_HAND* (find-package "CONTROL_CORE_MSGS::BODYID"))
(defconstant control_core_msgs::BodyId::*ID_LEFT_HAND* 2)
(intern "*ID_RIGHT_HAND*" (find-package "CONTROL_CORE_MSGS::BODYID"))
(shadow '*ID_RIGHT_HAND* (find-package "CONTROL_CORE_MSGS::BODYID"))
(defconstant control_core_msgs::BodyId::*ID_RIGHT_HAND* 3)

(defun control_core_msgs::BodyId-to-symbol (const)
  (cond
        ((= const 0) 'control_core_msgs::BodyId::*ID_LEFT_FOOT*)
        ((= const 1) 'control_core_msgs::BodyId::*ID_RIGHT_FOOT*)
        ((= const 2) 'control_core_msgs::BodyId::*ID_LEFT_HAND*)
        ((= const 3) 'control_core_msgs::BodyId::*ID_RIGHT_HAND*)
        (t nil)))

(defclass control_core_msgs::BodyId
  :super ros::object
  :slots (_id ))

(defmethod control_core_msgs::BodyId
  (:init
   (&key
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; uint8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get control_core_msgs::BodyId :md5sum-) "a385781e43d45f99df1603234266a10e")
(setf (get control_core_msgs::BodyId :datatype-) "control_core_msgs/BodyId")
(setf (get control_core_msgs::BodyId :definition-)
      "uint8 ID_LEFT_FOOT=0
uint8 ID_RIGHT_FOOT=1
uint8 ID_LEFT_HAND=2
uint8 ID_RIGHT_HAND=3
uint8 id
")



(provide :control_core_msgs/BodyId "a385781e43d45f99df1603234266a10e")


