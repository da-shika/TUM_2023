; Auto-generated. Do not edit!


(cl:in-package tomm_hardware_real-msg)


;//! \htmlinclude OmniBaseState.msg.html

(cl:defclass <OmniBaseState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type (cl:vector tomm_hardware_real-msg:DriverState)
   :initform (cl:make-array 0 :element-type 'tomm_hardware_real-msg:DriverState :initial-element (cl:make-instance 'tomm_hardware_real-msg:DriverState))))
)

(cl:defclass OmniBaseState (<OmniBaseState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OmniBaseState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OmniBaseState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tomm_hardware_real-msg:<OmniBaseState> is deprecated: use tomm_hardware_real-msg:OmniBaseState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <OmniBaseState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tomm_hardware_real-msg:state-val is deprecated.  Use tomm_hardware_real-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OmniBaseState>) ostream)
  "Serializes a message object of type '<OmniBaseState>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'state))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OmniBaseState>) istream)
  "Deserializes a message object of type '<OmniBaseState>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'state) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'state)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'tomm_hardware_real-msg:DriverState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OmniBaseState>)))
  "Returns string type for a message object of type '<OmniBaseState>"
  "tomm_hardware_real/OmniBaseState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OmniBaseState)))
  "Returns string type for a message object of type 'OmniBaseState"
  "tomm_hardware_real/OmniBaseState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OmniBaseState>)))
  "Returns md5sum for a message object of type '<OmniBaseState>"
  "fc9f3e21e19731c56a4c57b6a5b80bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OmniBaseState)))
  "Returns md5sum for a message object of type 'OmniBaseState"
  "fc9f3e21e19731c56a4c57b6a5b80bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OmniBaseState>)))
  "Returns full string definition for message of type '<OmniBaseState>"
  (cl:format cl:nil "DriverState[] state~%================================================================================~%MSG: tomm_hardware_real/DriverState~%int32 position~%uint32 digital_inputs~%int32 velocity~%uint16 status~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OmniBaseState)))
  "Returns full string definition for message of type 'OmniBaseState"
  (cl:format cl:nil "DriverState[] state~%================================================================================~%MSG: tomm_hardware_real/DriverState~%int32 position~%uint32 digital_inputs~%int32 velocity~%uint16 status~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OmniBaseState>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'state) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OmniBaseState>))
  "Converts a ROS message object to a list"
  (cl:list 'OmniBaseState
    (cl:cons ':state (state msg))
))
