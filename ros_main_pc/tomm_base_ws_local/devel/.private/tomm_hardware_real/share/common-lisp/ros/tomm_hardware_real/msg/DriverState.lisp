; Auto-generated. Do not edit!


(cl:in-package tomm_hardware_real-msg)


;//! \htmlinclude DriverState.msg.html

(cl:defclass <DriverState> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0)
   (digital_inputs
    :reader digital_inputs
    :initarg :digital_inputs
    :type cl:integer
    :initform 0)
   (velocity
    :reader velocity
    :initarg :velocity
    :type cl:integer
    :initform 0)
   (status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass DriverState (<DriverState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DriverState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DriverState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomm_hardware_real-msg:<DriverState> is deprecated: use tomm_hardware_real-msg:DriverState instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <DriverState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomm_hardware_real-msg:position-val is deprecated.  Use tomm_hardware_real-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'digital_inputs-val :lambda-list '(m))
(cl:defmethod digital_inputs-val ((m <DriverState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomm_hardware_real-msg:digital_inputs-val is deprecated.  Use tomm_hardware_real-msg:digital_inputs instead.")
  (digital_inputs m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <DriverState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomm_hardware_real-msg:velocity-val is deprecated.  Use tomm_hardware_real-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <DriverState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomm_hardware_real-msg:status-val is deprecated.  Use tomm_hardware_real-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DriverState>) ostream)
  "Serializes a message object of type '<DriverState>"
  (cl:let* ((signed (cl:slot-value msg 'position)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'digital_inputs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'digital_inputs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'digital_inputs)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'digital_inputs)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'velocity)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DriverState>) istream)
  "Deserializes a message object of type '<DriverState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'position) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'digital_inputs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'digital_inputs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'digital_inputs)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'digital_inputs)) (cl:read-byte istream))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'velocity) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DriverState>)))
  "Returns string type for a message object of type '<DriverState>"
  "tomm_hardware_real/DriverState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DriverState)))
  "Returns string type for a message object of type 'DriverState"
  "tomm_hardware_real/DriverState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DriverState>)))
  "Returns md5sum for a message object of type '<DriverState>"
  "a4fed67ceab1a7997c41bd586c46cc74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DriverState)))
  "Returns md5sum for a message object of type 'DriverState"
  "a4fed67ceab1a7997c41bd586c46cc74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DriverState>)))
  "Returns full string definition for message of type '<DriverState>"
  (cl:format cl:nil "int32 position~%uint32 digital_inputs~%int32 velocity~%uint16 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DriverState)))
  "Returns full string definition for message of type 'DriverState"
  (cl:format cl:nil "int32 position~%uint32 digital_inputs~%int32 velocity~%uint16 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DriverState>))
  (cl:+ 0
     4
     4
     4
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DriverState>))
  "Converts a ROS message object to a list"
  (cl:list 'DriverState
    (cl:cons ':position (position msg))
    (cl:cons ':digital_inputs (digital_inputs msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':status (status msg))
))
