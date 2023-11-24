;; Auto-generated. Do not edit!


(when (boundp 'control_core_msgs::Vector)
  (if (not (find-package "CONTROL_CORE_MSGS"))
    (make-package "CONTROL_CORE_MSGS"))
  (shadow 'Vector (find-package "CONTROL_CORE_MSGS")))
(unless (find-package "CONTROL_CORE_MSGS::VECTOR")
  (make-package "CONTROL_CORE_MSGS::VECTOR"))

(in-package "ROS")
;;//! \htmlinclude Vector.msg.html


(defclass control_core_msgs::Vector
  :super ros::object
  :slots (_data ))

(defmethod control_core_msgs::Vector
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float64[] _data
    (* 8    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get control_core_msgs::Vector :md5sum-) "788898178a3da2c3718461eecda8f714")
(setf (get control_core_msgs::Vector :datatype-) "control_core_msgs/Vector")
(setf (get control_core_msgs::Vector :definition-)
      "float64[] data
")



(provide :control_core_msgs/Vector "788898178a3da2c3718461eecda8f714")


