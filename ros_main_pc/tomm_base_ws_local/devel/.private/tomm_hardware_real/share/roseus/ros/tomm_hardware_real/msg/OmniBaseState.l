;; Auto-generated. Do not edit!


(when (boundp 'tomm_hardware_real::OmniBaseState)
  (if (not (find-package "TOMM_HARDWARE_REAL"))
    (make-package "TOMM_HARDWARE_REAL"))
  (shadow 'OmniBaseState (find-package "TOMM_HARDWARE_REAL")))
(unless (find-package "TOMM_HARDWARE_REAL::OMNIBASESTATE")
  (make-package "TOMM_HARDWARE_REAL::OMNIBASESTATE"))

(in-package "ROS")
;;//! \htmlinclude OmniBaseState.msg.html


(defclass tomm_hardware_real::OmniBaseState
  :super ros::object
  :slots (_state ))

(defmethod tomm_hardware_real::OmniBaseState
  (:init
   (&key
    ((:state __state) ())
    )
   (send-super :init)
   (setq _state __state)
   self)
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
    ;; tomm_hardware_real/DriverState[] _state
    (apply #'+ (send-all _state :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; tomm_hardware_real/DriverState[] _state
     (write-long (length _state) s)
     (dolist (elem _state)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; tomm_hardware_real/DriverState[] _state
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _state (let (r) (dotimes (i n) (push (instance tomm_hardware_real::DriverState :init) r)) r))
     (dolist (elem- _state)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get tomm_hardware_real::OmniBaseState :md5sum-) "fc9f3e21e19731c56a4c57b6a5b80bb7")
(setf (get tomm_hardware_real::OmniBaseState :datatype-) "tomm_hardware_real/OmniBaseState")
(setf (get tomm_hardware_real::OmniBaseState :definition-)
      "DriverState[] state
================================================================================
MSG: tomm_hardware_real/DriverState
int32 position
uint32 digital_inputs
int32 velocity
uint16 status
")



(provide :tomm_hardware_real/OmniBaseState "fc9f3e21e19731c56a4c57b6a5b80bb7")


