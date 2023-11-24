;; Auto-generated. Do not edit!


(when (boundp 'tomm_hardware_real::DriverState)
  (if (not (find-package "TOMM_HARDWARE_REAL"))
    (make-package "TOMM_HARDWARE_REAL"))
  (shadow 'DriverState (find-package "TOMM_HARDWARE_REAL")))
(unless (find-package "TOMM_HARDWARE_REAL::DRIVERSTATE")
  (make-package "TOMM_HARDWARE_REAL::DRIVERSTATE"))

(in-package "ROS")
;;//! \htmlinclude DriverState.msg.html


(defclass tomm_hardware_real::DriverState
  :super ros::object
  :slots (_position _digital_inputs _velocity _status ))

(defmethod tomm_hardware_real::DriverState
  (:init
   (&key
    ((:position __position) 0)
    ((:digital_inputs __digital_inputs) 0)
    ((:velocity __velocity) 0)
    ((:status __status) 0)
    )
   (send-super :init)
   (setq _position (round __position))
   (setq _digital_inputs (round __digital_inputs))
   (setq _velocity (round __velocity))
   (setq _status (round __status))
   self)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:digital_inputs
   (&optional __digital_inputs)
   (if __digital_inputs (setq _digital_inputs __digital_inputs)) _digital_inputs)
  (:velocity
   (&optional __velocity)
   (if __velocity (setq _velocity __velocity)) _velocity)
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; int32 _position
    4
    ;; uint32 _digital_inputs
    4
    ;; int32 _velocity
    4
    ;; uint16 _status
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _position
       (write-long _position s)
     ;; uint32 _digital_inputs
       (write-long _digital_inputs s)
     ;; int32 _velocity
       (write-long _velocity s)
     ;; uint16 _status
       (write-word _status s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _digital_inputs
     (setq _digital_inputs (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _velocity
     (setq _velocity (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint16 _status
     (setq _status (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get tomm_hardware_real::DriverState :md5sum-) "a4fed67ceab1a7997c41bd586c46cc74")
(setf (get tomm_hardware_real::DriverState :datatype-) "tomm_hardware_real/DriverState")
(setf (get tomm_hardware_real::DriverState :definition-)
      "int32 position
uint32 digital_inputs
int32 velocity
uint16 status
")



(provide :tomm_hardware_real/DriverState "a4fed67ceab1a7997c41bd586c46cc74")


