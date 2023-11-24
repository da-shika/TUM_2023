;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::ChangeContact)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'ChangeContact (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::CHANGECONTACT")
  (make-package "BEHAVIOR_MSGS::CHANGECONTACT"))
(unless (find-package "BEHAVIOR_MSGS::CHANGECONTACTREQUEST")
  (make-package "BEHAVIOR_MSGS::CHANGECONTACTREQUEST"))
(unless (find-package "BEHAVIOR_MSGS::CHANGECONTACTRESPONSE")
  (make-package "BEHAVIOR_MSGS::CHANGECONTACTRESPONSE"))

(in-package "ROS")





(defclass behavior_msgs::ChangeContactRequest
  :super ros::object
  :slots (_activate _deactivate ))

(defmethod behavior_msgs::ChangeContactRequest
  (:init
   (&key
    ((:activate __activate) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:deactivate __deactivate) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _activate __activate)
   (setq _deactivate __deactivate)
   self)
  (:activate
   (&optional __activate)
   (if __activate (setq _activate __activate)) _activate)
  (:deactivate
   (&optional __deactivate)
   (if __deactivate (setq _deactivate __deactivate)) _deactivate)
  (:serialization-length
   ()
   (+
    ;; string[] _activate
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _activate)) 4
    ;; string[] _deactivate
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _deactivate)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _activate
     (write-long (length _activate) s)
     (dolist (elem _activate)
       (write-long (length elem) s) (princ elem s)
       )
     ;; string[] _deactivate
     (write-long (length _deactivate) s)
     (dolist (elem _deactivate)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _activate
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _activate (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _activate i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; string[] _deactivate
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _deactivate (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _deactivate i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(defclass behavior_msgs::ChangeContactResponse
  :super ros::object
  :slots (_ok ))

(defmethod behavior_msgs::ChangeContactResponse
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

(defclass behavior_msgs::ChangeContact
  :super ros::object
  :slots ())

(setf (get behavior_msgs::ChangeContact :md5sum-) "fb67b79c35378211caa96d9ff91d121b")
(setf (get behavior_msgs::ChangeContact :datatype-) "behavior_msgs/ChangeContact")
(setf (get behavior_msgs::ChangeContact :request) behavior_msgs::ChangeContactRequest)
(setf (get behavior_msgs::ChangeContact :response) behavior_msgs::ChangeContactResponse)

(defmethod behavior_msgs::ChangeContactRequest
  (:response () (instance behavior_msgs::ChangeContactResponse :init)))

(setf (get behavior_msgs::ChangeContactRequest :md5sum-) "fb67b79c35378211caa96d9ff91d121b")
(setf (get behavior_msgs::ChangeContactRequest :datatype-) "behavior_msgs/ChangeContactRequest")
(setf (get behavior_msgs::ChangeContactRequest :definition-)
      "string[] activate
string[] deactivate
---
bool ok
")

(setf (get behavior_msgs::ChangeContactResponse :md5sum-) "fb67b79c35378211caa96d9ff91d121b")
(setf (get behavior_msgs::ChangeContactResponse :datatype-) "behavior_msgs/ChangeContactResponse")
(setf (get behavior_msgs::ChangeContactResponse :definition-)
      "string[] activate
string[] deactivate
---
bool ok
")



(provide :behavior_msgs/ChangeContact "fb67b79c35378211caa96d9ff91d121b")


