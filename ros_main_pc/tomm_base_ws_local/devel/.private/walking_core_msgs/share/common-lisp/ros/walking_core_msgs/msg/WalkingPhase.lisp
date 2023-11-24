; Auto-generated. Do not edit!


(cl:in-package walking_core_msgs-msg)


;//! \htmlinclude WalkingPhase.msg.html

(cl:defclass <WalkingPhase> (roslisp-msg-protocol:ros-message)
  ((phase
    :reader phase
    :initarg :phase
    :type cl:fixnum
    :initform 0))
)

(cl:defclass WalkingPhase (<WalkingPhase>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WalkingPhase>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WalkingPhase)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walking_core_msgs-msg:<WalkingPhase> is deprecated: use walking_core_msgs-msg:WalkingPhase instead.")))

(cl:ensure-generic-function 'phase-val :lambda-list '(m))
(cl:defmethod phase-val ((m <WalkingPhase>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:phase-val is deprecated.  Use walking_core_msgs-msg:phase instead.")
  (phase m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<WalkingPhase>)))
    "Constants for message type '<WalkingPhase>"
  '((:PHASE_STANCE . 0)
    (:PHASE_DOUBLESUPPORT . 1)
    (:PHASE_SINGLESUPPORT . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'WalkingPhase)))
    "Constants for message type 'WalkingPhase"
  '((:PHASE_STANCE . 0)
    (:PHASE_DOUBLESUPPORT . 1)
    (:PHASE_SINGLESUPPORT . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WalkingPhase>) ostream)
  "Serializes a message object of type '<WalkingPhase>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'phase)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WalkingPhase>) istream)
  "Deserializes a message object of type '<WalkingPhase>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'phase)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WalkingPhase>)))
  "Returns string type for a message object of type '<WalkingPhase>"
  "walking_core_msgs/WalkingPhase")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WalkingPhase)))
  "Returns string type for a message object of type 'WalkingPhase"
  "walking_core_msgs/WalkingPhase")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WalkingPhase>)))
  "Returns md5sum for a message object of type '<WalkingPhase>"
  "1712835dbfb69f4b5728c67900acc6bf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WalkingPhase)))
  "Returns md5sum for a message object of type 'WalkingPhase"
  "1712835dbfb69f4b5728c67900acc6bf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WalkingPhase>)))
  "Returns full string definition for message of type '<WalkingPhase>"
  (cl:format cl:nil "uint8 PHASE_STANCE=0~%uint8 PHASE_DOUBLESUPPORT=1~%uint8 PHASE_SINGLESUPPORT=2~%uint8 phase~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WalkingPhase)))
  "Returns full string definition for message of type 'WalkingPhase"
  (cl:format cl:nil "uint8 PHASE_STANCE=0~%uint8 PHASE_DOUBLESUPPORT=1~%uint8 PHASE_SINGLESUPPORT=2~%uint8 phase~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WalkingPhase>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WalkingPhase>))
  "Converts a ROS message object to a list"
  (cl:list 'WalkingPhase
    (cl:cons ':phase (phase msg))
))
