;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::JoyKey)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'JoyKey (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::JOYKEY")
  (make-package "CONTROL_CORE_MSGS::JOYKEY"))

(in-package "ROS")
;;//! \htmlinclude JoyKey.msg.html


(intern "*KEY_A*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*KEY_A* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*KEY_A* 0)
(intern "*KEY_B*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*KEY_B* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*KEY_B* 1)
(intern "*KEY_X*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*KEY_X* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*KEY_X* 2)
(intern "*KEY_Y*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*KEY_Y* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*KEY_Y* 3)
(intern "*KEY_LB*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*KEY_LB* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*KEY_LB* 4)
(intern "*KEY_RB*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*KEY_RB* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*KEY_RB* 5)
(intern "*JOY_LX*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*JOY_LX* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*JOY_LX* 0)
(intern "*JOY_LY*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*JOY_LY* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*JOY_LY* 1)
(intern "*JOY_LT*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*JOY_LT* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*JOY_LT* 2)
(intern "*JOY_RX*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*JOY_RX* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*JOY_RX* 3)
(intern "*JOY_RY*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*JOY_RY* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*JOY_RY* 4)
(intern "*JOY_RT*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*JOY_RT* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*JOY_RT* 5)
(intern "*ARROW_LEFT_RIGHT*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*ARROW_LEFT_RIGHT* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*ARROW_LEFT_RIGHT* 6)
(intern "*ARROW_UP_DOWN*" (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(shadow '*ARROW_UP_DOWN* (find-package "CONTROL_CORE_MSGS::JOYKEY"))
(defconstant control_core_msgs::JoyKey::*ARROW_UP_DOWN* 7)

(defun control_core_msgs::JoyKey-to-symbol (const)
  (cond
        ((= const 0) 'control_core_msgs::JoyKey::*KEY_A*)
        ((= const 1) 'control_core_msgs::JoyKey::*KEY_B*)
        ((= const 2) 'control_core_msgs::JoyKey::*KEY_X*)
        ((= const 3) 'control_core_msgs::JoyKey::*KEY_Y*)
        ((= const 4) 'control_core_msgs::JoyKey::*KEY_LB*)
        ((= const 5) 'control_core_msgs::JoyKey::*KEY_RB*)
        ((= const 0) 'control_core_msgs::JoyKey::*JOY_LX*)
        ((= const 1) 'control_core_msgs::JoyKey::*JOY_LY*)
        ((= const 2) 'control_core_msgs::JoyKey::*JOY_LT*)
        ((= const 3) 'control_core_msgs::JoyKey::*JOY_RX*)
        ((= const 4) 'control_core_msgs::JoyKey::*JOY_RY*)
        ((= const 5) 'control_core_msgs::JoyKey::*JOY_RT*)
        ((= const 6) 'control_core_msgs::JoyKey::*ARROW_LEFT_RIGHT*)
        ((= const 7) 'control_core_msgs::JoyKey::*ARROW_UP_DOWN*)
        (t nil)))

(defclass control_core_msgs::JoyKey
  :super ros::object
  :slots ())

(defmethod control_core_msgs::JoyKey
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(setf (get control_core_msgs::JoyKey :md5sum-) "7d7dd2dc15a1d1ee57b8e83eb173156c")
(setf (get control_core_msgs::JoyKey :datatype-) "control_core_msgs/JoyKey")
(setf (get control_core_msgs::JoyKey :definition-)
      "# Keys for the Logitech F710 controller
# Note: The controller mode is set to 'X'

uint8 KEY_A=0
uint8 KEY_B=1
uint8 KEY_X=2
uint8 KEY_Y=3
uint8 KEY_LB=4
uint8 KEY_RB=5
uint8 JOY_LX=0
uint8 JOY_LY=1
uint8 JOY_LT=2
uint8 JOY_RX=3
uint8 JOY_RY=4
uint8 JOY_RT=5
uint8 ARROW_LEFT_RIGHT=6
uint8 ARROW_UP_DOWN=7
")



(provide :control_core_msgs/JoyKey "7d7dd2dc15a1d1ee57b8e83eb173156c")


