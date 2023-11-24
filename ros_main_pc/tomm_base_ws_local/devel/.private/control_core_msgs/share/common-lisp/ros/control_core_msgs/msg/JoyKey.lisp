; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude JoyKey.msg.html

(cl:defclass <JoyKey> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass JoyKey (<JoyKey>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JoyKey>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JoyKey)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<JoyKey> is deprecated: use control_core_msgs-msg:JoyKey instead.")))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<JoyKey>)))
    "Constants for message type '<JoyKey>"
  '((:KEY_A . 0)
    (:KEY_B . 1)
    (:KEY_X . 2)
    (:KEY_Y . 3)
    (:KEY_LB . 4)
    (:KEY_RB . 5)
    (:JOY_LX . 0)
    (:JOY_LY . 1)
    (:JOY_LT . 2)
    (:JOY_RX . 3)
    (:JOY_RY . 4)
    (:JOY_RT . 5)
    (:ARROW_LEFT_RIGHT . 6)
    (:ARROW_UP_DOWN . 7))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'JoyKey)))
    "Constants for message type 'JoyKey"
  '((:KEY_A . 0)
    (:KEY_B . 1)
    (:KEY_X . 2)
    (:KEY_Y . 3)
    (:KEY_LB . 4)
    (:KEY_RB . 5)
    (:JOY_LX . 0)
    (:JOY_LY . 1)
    (:JOY_LT . 2)
    (:JOY_RX . 3)
    (:JOY_RY . 4)
    (:JOY_RT . 5)
    (:ARROW_LEFT_RIGHT . 6)
    (:ARROW_UP_DOWN . 7))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JoyKey>) ostream)
  "Serializes a message object of type '<JoyKey>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JoyKey>) istream)
  "Deserializes a message object of type '<JoyKey>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JoyKey>)))
  "Returns string type for a message object of type '<JoyKey>"
  "control_core_msgs/JoyKey")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JoyKey)))
  "Returns string type for a message object of type 'JoyKey"
  "control_core_msgs/JoyKey")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JoyKey>)))
  "Returns md5sum for a message object of type '<JoyKey>"
  "7d7dd2dc15a1d1ee57b8e83eb173156c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JoyKey)))
  "Returns md5sum for a message object of type 'JoyKey"
  "7d7dd2dc15a1d1ee57b8e83eb173156c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JoyKey>)))
  "Returns full string definition for message of type '<JoyKey>"
  (cl:format cl:nil "# Keys for the Logitech F710 controller~%# Note: The controller mode is set to 'X'~%~%uint8 KEY_A=0~%uint8 KEY_B=1~%uint8 KEY_X=2~%uint8 KEY_Y=3~%uint8 KEY_LB=4~%uint8 KEY_RB=5~%uint8 JOY_LX=0~%uint8 JOY_LY=1~%uint8 JOY_LT=2~%uint8 JOY_RX=3~%uint8 JOY_RY=4~%uint8 JOY_RT=5~%uint8 ARROW_LEFT_RIGHT=6~%uint8 ARROW_UP_DOWN=7~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JoyKey)))
  "Returns full string definition for message of type 'JoyKey"
  (cl:format cl:nil "# Keys for the Logitech F710 controller~%# Note: The controller mode is set to 'X'~%~%uint8 KEY_A=0~%uint8 KEY_B=1~%uint8 KEY_X=2~%uint8 KEY_Y=3~%uint8 KEY_LB=4~%uint8 KEY_RB=5~%uint8 JOY_LX=0~%uint8 JOY_LY=1~%uint8 JOY_LT=2~%uint8 JOY_RX=3~%uint8 JOY_RY=4~%uint8 JOY_RT=5~%uint8 ARROW_LEFT_RIGHT=6~%uint8 ARROW_UP_DOWN=7~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JoyKey>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JoyKey>))
  "Converts a ROS message object to a list"
  (cl:list 'JoyKey
))
