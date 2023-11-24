;; Auto-generated. Do not edit!


(when (boundp 'walking_core_msgs::WalkingPhase)
  (if (not (find-package "WALKING_CORE_MSGS"))
    (make-package "WALKING_CORE_MSGS"))
  (shadow 'WalkingPhase (find-package "WALKING_CORE_MSGS")))
(unless (find-package "WALKING_CORE_MSGS::WALKINGPHASE")
  (make-package "WALKING_CORE_MSGS::WALKINGPHASE"))

(in-package "ROS")
;;//! \htmlinclude WalkingPhase.msg.html


(intern "*PHASE_STANCE*" (find-package "WALKING_CORE_MSGS::WALKINGPHASE"))
(shadow '*PHASE_STANCE* (find-package "WALKING_CORE_MSGS::WALKINGPHASE"))
(defconstant walking_core_msgs::WalkingPhase::*PHASE_STANCE* 0)
(intern "*PHASE_DOUBLESUPPORT*" (find-package "WALKING_CORE_MSGS::WALKINGPHASE"))
(shadow '*PHASE_DOUBLESUPPORT* (find-package "WALKING_CORE_MSGS::WALKINGPHASE"))
(defconstant walking_core_msgs::WalkingPhase::*PHASE_DOUBLESUPPORT* 1)
(intern "*PHASE_SINGLESUPPORT*" (find-package "WALKING_CORE_MSGS::WALKINGPHASE"))
(shadow '*PHASE_SINGLESUPPORT* (find-package "WALKING_CORE_MSGS::WALKINGPHASE"))
(defconstant walking_core_msgs::WalkingPhase::*PHASE_SINGLESUPPORT* 2)

(defun walking_core_msgs::WalkingPhase-to-symbol (const)
  (cond
        ((= const 0) 'walking_core_msgs::WalkingPhase::*PHASE_STANCE*)
        ((= const 1) 'walking_core_msgs::WalkingPhase::*PHASE_DOUBLESUPPORT*)
        ((= const 2) 'walking_core_msgs::WalkingPhase::*PHASE_SINGLESUPPORT*)
        (t nil)))

(defclass walking_core_msgs::WalkingPhase
  :super ros::object
  :slots (_phase ))

(defmethod walking_core_msgs::WalkingPhase
  (:init
   (&key
    ((:phase __phase) 0)
    )
   (send-super :init)
   (setq _phase (round __phase))
   self)
  (:phase
   (&optional __phase)
   (if __phase (setq _phase __phase)) _phase)
  (:serialization-length
   ()
   (+
    ;; uint8 _phase
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint8 _phase
       (write-byte _phase s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint8 _phase
     (setq _phase (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get walking_core_msgs::WalkingPhase :md5sum-) "1712835dbfb69f4b5728c67900acc6bf")
(setf (get walking_core_msgs::WalkingPhase :datatype-) "walking_core_msgs/WalkingPhase")
(setf (get walking_core_msgs::WalkingPhase :definition-)
      "uint8 PHASE_STANCE=0
uint8 PHASE_DOUBLESUPPORT=1
uint8 PHASE_SINGLESUPPORT=2
uint8 phase
")



(provide :walking_core_msgs/WalkingPhase "1712835dbfb69f4b5728c67900acc6bf")


