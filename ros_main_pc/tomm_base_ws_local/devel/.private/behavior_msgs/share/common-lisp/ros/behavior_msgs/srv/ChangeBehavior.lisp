; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-srv)


;//! \htmlinclude ChangeBehavior-request.msg.html

(cl:defclass <ChangeBehavior-request> (roslisp-msg-protocol:ros-message)
  ((start_behaviors
    :reader start_behaviors
    :initarg :start_behaviors
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (stop_behaviors
    :reader stop_behaviors
    :initarg :stop_behaviors
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ChangeBehavior-request (<ChangeBehavior-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeBehavior-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeBehavior-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-srv:<ChangeBehavior-request> is deprecated: use behavior_msgs-srv:ChangeBehavior-request instead.")))

(cl:ensure-generic-function 'start_behaviors-val :lambda-list '(m))
(cl:defmethod start_behaviors-val ((m <ChangeBehavior-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:start_behaviors-val is deprecated.  Use behavior_msgs-srv:start_behaviors instead.")
  (start_behaviors m))

(cl:ensure-generic-function 'stop_behaviors-val :lambda-list '(m))
(cl:defmethod stop_behaviors-val ((m <ChangeBehavior-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:stop_behaviors-val is deprecated.  Use behavior_msgs-srv:stop_behaviors instead.")
  (stop_behaviors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeBehavior-request>) ostream)
  "Serializes a message object of type '<ChangeBehavior-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'start_behaviors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'start_behaviors))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'stop_behaviors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'stop_behaviors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeBehavior-request>) istream)
  "Deserializes a message object of type '<ChangeBehavior-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'start_behaviors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'start_behaviors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'stop_behaviors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'stop_behaviors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeBehavior-request>)))
  "Returns string type for a service object of type '<ChangeBehavior-request>"
  "behavior_msgs/ChangeBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeBehavior-request)))
  "Returns string type for a service object of type 'ChangeBehavior-request"
  "behavior_msgs/ChangeBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeBehavior-request>)))
  "Returns md5sum for a message object of type '<ChangeBehavior-request>"
  "aa4bf15e9a641af0b18ba2be792bf867")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeBehavior-request)))
  "Returns md5sum for a message object of type 'ChangeBehavior-request"
  "aa4bf15e9a641af0b18ba2be792bf867")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeBehavior-request>)))
  "Returns full string definition for message of type '<ChangeBehavior-request>"
  (cl:format cl:nil "string[] start_behaviors~%string[] stop_behaviors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeBehavior-request)))
  "Returns full string definition for message of type 'ChangeBehavior-request"
  (cl:format cl:nil "string[] start_behaviors~%string[] stop_behaviors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeBehavior-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'start_behaviors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'stop_behaviors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeBehavior-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeBehavior-request
    (cl:cons ':start_behaviors (start_behaviors msg))
    (cl:cons ':stop_behaviors (stop_behaviors msg))
))
;//! \htmlinclude ChangeBehavior-response.msg.html

(cl:defclass <ChangeBehavior-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ChangeBehavior-response (<ChangeBehavior-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeBehavior-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeBehavior-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-srv:<ChangeBehavior-response> is deprecated: use behavior_msgs-srv:ChangeBehavior-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <ChangeBehavior-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:ok-val is deprecated.  Use behavior_msgs-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeBehavior-response>) ostream)
  "Serializes a message object of type '<ChangeBehavior-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeBehavior-response>) istream)
  "Deserializes a message object of type '<ChangeBehavior-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeBehavior-response>)))
  "Returns string type for a service object of type '<ChangeBehavior-response>"
  "behavior_msgs/ChangeBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeBehavior-response)))
  "Returns string type for a service object of type 'ChangeBehavior-response"
  "behavior_msgs/ChangeBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeBehavior-response>)))
  "Returns md5sum for a message object of type '<ChangeBehavior-response>"
  "aa4bf15e9a641af0b18ba2be792bf867")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeBehavior-response)))
  "Returns md5sum for a message object of type 'ChangeBehavior-response"
  "aa4bf15e9a641af0b18ba2be792bf867")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeBehavior-response>)))
  "Returns full string definition for message of type '<ChangeBehavior-response>"
  (cl:format cl:nil "bool ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeBehavior-response)))
  "Returns full string definition for message of type 'ChangeBehavior-response"
  (cl:format cl:nil "bool ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeBehavior-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeBehavior-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeBehavior-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangeBehavior)))
  'ChangeBehavior-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangeBehavior)))
  'ChangeBehavior-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeBehavior)))
  "Returns string type for a service object of type '<ChangeBehavior>"
  "behavior_msgs/ChangeBehavior")