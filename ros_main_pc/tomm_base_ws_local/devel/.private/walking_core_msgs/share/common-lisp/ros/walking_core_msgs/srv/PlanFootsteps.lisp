; Auto-generated. Do not edit!


(cl:in-package walking_core_msgs-srv)


;//! \htmlinclude PlanFootsteps-request.msg.html

(cl:defclass <PlanFootsteps-request> (roslisp-msg-protocol:ros-message)
  ((n_steps
    :reader n_steps
    :initarg :n_steps
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (length
    :reader length
    :initarg :length
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (lateral
    :reader lateral
    :initarg :lateral
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (angle
    :reader angle
    :initarg :angle
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass PlanFootsteps-request (<PlanFootsteps-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanFootsteps-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanFootsteps-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walking_core_msgs-srv:<PlanFootsteps-request> is deprecated: use walking_core_msgs-srv:PlanFootsteps-request instead.")))

(cl:ensure-generic-function 'n_steps-val :lambda-list '(m))
(cl:defmethod n_steps-val ((m <PlanFootsteps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-srv:n_steps-val is deprecated.  Use walking_core_msgs-srv:n_steps instead.")
  (n_steps m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <PlanFootsteps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-srv:length-val is deprecated.  Use walking_core_msgs-srv:length instead.")
  (length m))

(cl:ensure-generic-function 'lateral-val :lambda-list '(m))
(cl:defmethod lateral-val ((m <PlanFootsteps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-srv:lateral-val is deprecated.  Use walking_core_msgs-srv:lateral instead.")
  (lateral m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <PlanFootsteps-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-srv:angle-val is deprecated.  Use walking_core_msgs-srv:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanFootsteps-request>) ostream)
  "Serializes a message object of type '<PlanFootsteps-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'n_steps) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'length) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'lateral) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angle) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanFootsteps-request>) istream)
  "Deserializes a message object of type '<PlanFootsteps-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'n_steps) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'length) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'lateral) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angle) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanFootsteps-request>)))
  "Returns string type for a service object of type '<PlanFootsteps-request>"
  "walking_core_msgs/PlanFootstepsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanFootsteps-request)))
  "Returns string type for a service object of type 'PlanFootsteps-request"
  "walking_core_msgs/PlanFootstepsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanFootsteps-request>)))
  "Returns md5sum for a message object of type '<PlanFootsteps-request>"
  "6b7b78d5ced6128d564af0ca90b3d6e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanFootsteps-request)))
  "Returns md5sum for a message object of type 'PlanFootsteps-request"
  "6b7b78d5ced6128d564af0ca90b3d6e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanFootsteps-request>)))
  "Returns full string definition for message of type '<PlanFootsteps-request>"
  (cl:format cl:nil "std_msgs/Int64      n_steps~%std_msgs/Float64    length~%std_msgs/Float64    lateral~%std_msgs/Float64    angle~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanFootsteps-request)))
  "Returns full string definition for message of type 'PlanFootsteps-request"
  (cl:format cl:nil "std_msgs/Int64      n_steps~%std_msgs/Float64    length~%std_msgs/Float64    lateral~%std_msgs/Float64    angle~%~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanFootsteps-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'n_steps))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'length))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'lateral))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angle))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanFootsteps-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanFootsteps-request
    (cl:cons ':n_steps (n_steps msg))
    (cl:cons ':length (length msg))
    (cl:cons ':lateral (lateral msg))
    (cl:cons ':angle (angle msg))
))
;//! \htmlinclude PlanFootsteps-response.msg.html

(cl:defclass <PlanFootsteps-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass PlanFootsteps-response (<PlanFootsteps-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlanFootsteps-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlanFootsteps-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walking_core_msgs-srv:<PlanFootsteps-response> is deprecated: use walking_core_msgs-srv:PlanFootsteps-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <PlanFootsteps-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-srv:success-val is deprecated.  Use walking_core_msgs-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlanFootsteps-response>) ostream)
  "Serializes a message object of type '<PlanFootsteps-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'success) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlanFootsteps-response>) istream)
  "Deserializes a message object of type '<PlanFootsteps-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'success) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlanFootsteps-response>)))
  "Returns string type for a service object of type '<PlanFootsteps-response>"
  "walking_core_msgs/PlanFootstepsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanFootsteps-response)))
  "Returns string type for a service object of type 'PlanFootsteps-response"
  "walking_core_msgs/PlanFootstepsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlanFootsteps-response>)))
  "Returns md5sum for a message object of type '<PlanFootsteps-response>"
  "6b7b78d5ced6128d564af0ca90b3d6e2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlanFootsteps-response)))
  "Returns md5sum for a message object of type 'PlanFootsteps-response"
  "6b7b78d5ced6128d564af0ca90b3d6e2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlanFootsteps-response>)))
  "Returns full string definition for message of type '<PlanFootsteps-response>"
  (cl:format cl:nil "std_msgs/Bool success~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlanFootsteps-response)))
  "Returns full string definition for message of type 'PlanFootsteps-response"
  (cl:format cl:nil "std_msgs/Bool success~%~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlanFootsteps-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'success))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlanFootsteps-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PlanFootsteps-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PlanFootsteps)))
  'PlanFootsteps-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PlanFootsteps)))
  'PlanFootsteps-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlanFootsteps)))
  "Returns string type for a service object of type '<PlanFootsteps>"
  "walking_core_msgs/PlanFootsteps")