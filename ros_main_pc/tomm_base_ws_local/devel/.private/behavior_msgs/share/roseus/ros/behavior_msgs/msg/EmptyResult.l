;; Auto-generated. Do not edit!


(when (boundp 'behavior_msgs::EmptyResult)
  (if (not (find-package "BEHAVIOR_MSGS"))
    (make-package "BEHAVIOR_MSGS"))
  (shadow 'EmptyResult (find-package "BEHAVIOR_MSGS")))
(unless (find-package "BEHAVIOR_MSGS::EMPTYRESULT")
  (make-package "BEHAVIOR_MSGS::EMPTYRESULT"))

(in-package "ROS")
;;//! \htmlinclude EmptyResult.msg.html


(defclass behavior_msgs::EmptyResult
  :super ros::object
  :slots ())

(defmethod behavior_msgs::EmptyResult
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

(setf (get behavior_msgs::EmptyResult :md5sum-) "d41d8cd98f00b204e9800998ecf8427e")
(setf (get behavior_msgs::EmptyResult :datatype-) "behavior_msgs/EmptyResult")
(setf (get behavior_msgs::EmptyResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======

")



(provide :behavior_msgs/EmptyResult "d41d8cd98f00b204e9800998ecf8427e")

