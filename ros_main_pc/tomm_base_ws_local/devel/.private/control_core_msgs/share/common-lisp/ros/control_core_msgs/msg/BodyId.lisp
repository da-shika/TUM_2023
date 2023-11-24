; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude BodyId.msg.html

(cl:defclass <BodyId> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass BodyId (<BodyId>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyId>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyId)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<BodyId> is deprecated: use control_core_msgs-msg:BodyId instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <BodyId>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:id-val is deprecated.  Use control_core_msgs-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<BodyId>)))
    "Constants for message type '<BodyId>"
  '((:ID_LEFT_FOOT . 0)
    (:ID_RIGHT_FOOT . 1)
    (:ID_LEFT_HAND . 2)
    (:ID_RIGHT_HAND . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'BodyId)))
    "Constants for message type 'BodyId"
  '((:ID_LEFT_FOOT . 0)
    (:ID_RIGHT_FOOT . 1)
    (:ID_LEFT_HAND . 2)
    (:ID_RIGHT_HAND . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyId>) ostream)
  "Serializes a message object of type '<BodyId>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyId>) istream)
  "Deserializes a message object of type '<BodyId>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'id)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyId>)))
  "Returns string type for a message object of type '<BodyId>"
  "control_core_msgs/BodyId")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyId)))
  "Returns string type for a message object of type 'BodyId"
  "control_core_msgs/BodyId")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyId>)))
  "Returns md5sum for a message object of type '<BodyId>"
  "a385781e43d45f99df1603234266a10e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyId)))
  "Returns md5sum for a message object of type 'BodyId"
  "a385781e43d45f99df1603234266a10e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyId>)))
  "Returns full string definition for message of type '<BodyId>"
  (cl:format cl:nil "uint8 ID_LEFT_FOOT=0~%uint8 ID_RIGHT_FOOT=1~%uint8 ID_LEFT_HAND=2~%uint8 ID_RIGHT_HAND=3~%uint8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyId)))
  "Returns full string definition for message of type 'BodyId"
  (cl:format cl:nil "uint8 ID_LEFT_FOOT=0~%uint8 ID_RIGHT_FOOT=1~%uint8 ID_LEFT_HAND=2~%uint8 ID_RIGHT_HAND=3~%uint8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyId>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyId>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyId
    (cl:cons ':id (id msg))
))
