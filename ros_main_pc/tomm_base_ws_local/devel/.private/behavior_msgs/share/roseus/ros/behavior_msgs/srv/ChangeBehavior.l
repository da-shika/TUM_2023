;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::ChangeBehavior)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'ChangeBehavior (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::CHANGEBEHAVIOR")
  (make-package "BEHAVIOR_MSGS::CHANGEBEHAVIOR"))
(unless (find-package "BEHAVIOR_MSGS::CHANGEBEHAVIORREQUEST")
  (make-package "BEHAVIOR_MSGS::CHANGEBEHAVIORREQUEST"))
(unless (find-package "BEHAVIOR_MSGS::CHANGEBEHAVIORRESPONSE")
  (make-package "BEHAVIOR_MSGS::CHANGEBEHAVIORRESPONSE"))

(in-package "ROS")





(defclass behavior_msgs::ChangeBehaviorRequest
  :super ros::object
  :slots (_start_behaviors _stop_behaviors ))

(defmethod behavior_msgs::ChangeBehaviorRequest
  (:init
   (&key
    ((:start_behaviors __start_behaviors) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:stop_behaviors __stop_behaviors) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _start_behaviors __start_behaviors)
   (setq _stop_behaviors __stop_behaviors)
   self)
  (:start_behaviors
   (&optional __start_behaviors)
   (if __start_behaviors (setq _start_behaviors __start_behaviors)) _start_behaviors)
  (:stop_behaviors
   (&optional __stop_behaviors)
   (if __stop_behaviors (setq _stop_behaviors __stop_behaviors)) _stop_behaviors)
  (:serialization-length
   ()
   (+
    ;; string[] _start_behaviors
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _start_behaviors)) 4
    ;; string[] _stop_behaviors
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _stop_behaviors)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _start_behaviors
     (write-long (length _start_behaviors) s)
     (dolist (elem _start_behaviors)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _stop_behaviors
     (write-long (length _stop_behaviors) s)
     (dolist (elem _stop_behaviors)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _start_behaviors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start_behaviors (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _start_behaviors i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _stop_behaviors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _stop_behaviors (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _stop_behaviors i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass behavior_msgs::ChangeBehaviorResponse
  :super ros::object
  :slots (_ok ))

(defmethod behavior_msgs::ChangeBehaviorResponse
  (:init
   (&key
    ((:ok __ok) nil)
    )
   (send-super :init)
   (setq _ok __ok)
   self)
  (:ok
   (&optional (__ok :null))
   (if (not (eq __ok :null)) (setq _ok __ok)) _ok)
  (:serialization-length
   ()
   (+
    ;; bool _ok
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _ok
       (if _ok (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _ok
     (setq _ok (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass behavior_msgs::ChangeBehavior
  :super ros::object
  :slots ())

(setf (get behavior_msgs::ChangeBehavior :md5sum-) "aa4bf15e9a641af0b18ba2be792bf867")
(setf (get behavior_msgs::ChangeBehavior :datatype-) "behavior_msgs/ChangeBehavior")
(setf (get behavior_msgs::ChangeBehavior :request) behavior_msgs::ChangeBehaviorRequest)
(setf (get behavior_msgs::ChangeBehavior :response) behavior_msgs::ChangeBehaviorResponse)

(defmethod behavior_msgs::ChangeBehaviorRequest
  (:response () (instance behavior_msgs::ChangeBehaviorResponse :init)))

(setf (get behavior_msgs::ChangeBehaviorRequest :md5sum-) "aa4bf15e9a641af0b18ba2be792bf867")
(setf (get behavior_msgs::ChangeBehaviorRequest :datatype-) "behavior_msgs/ChangeBehaviorRequest")
(setf (get behavior_msgs::ChangeBehaviorRequest :definition-)
      "string[] start_behaviors
string[] stop_behaviors
---
bool ok
")

(setf (get behavior_msgs::ChangeBehaviorResponse :md5sum-) "aa4bf15e9a641af0b18ba2be792bf867")
(setf (get behavior_msgs::ChangeBehaviorResponse :datatype-) "behavior_msgs/ChangeBehaviorResponse")
(setf (get behavior_msgs::ChangeBehaviorResponse :definition-)
      "string[] start_behaviors
string[] stop_behaviors
---
bool ok
")



(provide :behavior_msgs/ChangeBehavior "aa4bf15e9a641af0b18ba2be792bf867")


