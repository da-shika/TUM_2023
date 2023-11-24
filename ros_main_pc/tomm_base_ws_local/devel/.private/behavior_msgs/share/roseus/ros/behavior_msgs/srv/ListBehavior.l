;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::ListBehavior)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'ListBehavior (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::LISTBEHAVIOR")
  (make-package "BEHAVIOR_MSGS::LISTBEHAVIOR"))
(unless (find-package "BEHAVIOR_MSGS::LISTBEHAVIORREQUEST")
  (make-package "BEHAVIOR_MSGS::LISTBEHAVIORREQUEST"))
(unless (find-package "BEHAVIOR_MSGS::LISTBEHAVIORRESPONSE")
  (make-package "BEHAVIOR_MSGS::LISTBEHAVIORRESPONSE"))

(in-package "ROS")





(defclass behavior_msgs::ListBehaviorRequest
  :super ros::object
  :slots ())

(defmethod behavior_msgs::ListBehaviorRequest
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

(defclass behavior_msgs::ListBehaviorResponse
  :super ros::object
  :slots (_loaded_behaviors _running_behaviors ))

(defmethod behavior_msgs::ListBehaviorResponse
  (:init
   (&key
    ((:loaded_behaviors __loaded_behaviors) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:running_behaviors __running_behaviors) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _loaded_behaviors __loaded_behaviors)
   (setq _running_behaviors __running_behaviors)
   self)
  (:loaded_behaviors
   (&optional __loaded_behaviors)
   (if __loaded_behaviors (setq _loaded_behaviors __loaded_behaviors)) _loaded_behaviors)
  (:running_behaviors
   (&optional __running_behaviors)
   (if __running_behaviors (setq _running_behaviors __running_behaviors)) _running_behaviors)
  (:serialization-length
   ()
   (+
    ;; string[] _loaded_behaviors
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _loaded_behaviors)) 4
    ;; string[] _running_behaviors
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _running_behaviors)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _loaded_behaviors
     (write-long (length _loaded_behaviors) s)
     (dolist (elem _loaded_behaviors)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _running_behaviors
     (write-long (length _running_behaviors) s)
     (dolist (elem _running_behaviors)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _loaded_behaviors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _loaded_behaviors (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _loaded_behaviors i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _running_behaviors
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _running_behaviors (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _running_behaviors i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass behavior_msgs::ListBehavior
  :super ros::object
  :slots ())

(setf (get behavior_msgs::ListBehavior :md5sum-) "db7d6a6bd85a4f8fde354aa4e08d9629")
(setf (get behavior_msgs::ListBehavior :datatype-) "behavior_msgs/ListBehavior")
(setf (get behavior_msgs::ListBehavior :request) behavior_msgs::ListBehaviorRequest)
(setf (get behavior_msgs::ListBehavior :response) behavior_msgs::ListBehaviorResponse)

(defmethod behavior_msgs::ListBehaviorRequest
  (:response () (instance behavior_msgs::ListBehaviorResponse :init)))

(setf (get behavior_msgs::ListBehaviorRequest :md5sum-) "db7d6a6bd85a4f8fde354aa4e08d9629")
(setf (get behavior_msgs::ListBehaviorRequest :datatype-) "behavior_msgs/ListBehaviorRequest")
(setf (get behavior_msgs::ListBehaviorRequest :definition-)
      "---
string[] loaded_behaviors
string[] running_behaviors
")

(setf (get behavior_msgs::ListBehaviorResponse :md5sum-) "db7d6a6bd85a4f8fde354aa4e08d9629")
(setf (get behavior_msgs::ListBehaviorResponse :datatype-) "behavior_msgs/ListBehaviorResponse")
(setf (get behavior_msgs::ListBehaviorResponse :definition-)
      "---
string[] loaded_behaviors
string[] running_behaviors
")



(provide :behavior_msgs/ListBehavior "db7d6a6bd85a4f8fde354aa4e08d9629")


