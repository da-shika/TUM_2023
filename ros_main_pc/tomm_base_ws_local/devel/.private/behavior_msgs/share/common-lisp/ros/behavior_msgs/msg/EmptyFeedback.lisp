; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-msg)


;//! \htmlinclude EmptyFeedback.msg.html

(cl:defclass <EmptyFeedback> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass EmptyFeedback (<EmptyFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmptyFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmptyFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-msg:<EmptyFeedback> is deprecated: use behavior_msgs-msg:EmptyFeedback instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmptyFeedback>) ostream)
  "Serializes a message object of type '<EmptyFeedback>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmptyFeedback>) istream)
  "Deserializes a message object of type '<EmptyFeedback>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmptyFeedback>)))
  "Returns string type for a message object of type '<EmptyFeedback>"
  "behavior_msgs/EmptyFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmptyFeedback)))
  "Returns string type for a message object of type 'EmptyFeedback"
  "behavior_msgs/EmptyFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmptyFeedback>)))
  "Returns md5sum for a message object of type '<EmptyFeedback>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmptyFeedback)))
  "Returns md5sum for a message object of type 'EmptyFeedback"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmptyFeedback>)))
  "Returns full string definition for message of type '<EmptyFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmptyFeedback)))
  "Returns full string definition for message of type 'EmptyFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmptyFeedback>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmptyFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'EmptyFeedback
))
