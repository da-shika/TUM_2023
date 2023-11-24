; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-srv)


;//! \htmlinclude ListBehavior-request.msg.html

(cl:defclass <ListBehavior-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ListBehavior-request (<ListBehavior-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListBehavior-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListBehavior-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-srv:<ListBehavior-request> is deprecated: use behavior_msgs-srv:ListBehavior-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListBehavior-request>) ostream)
  "Serializes a message object of type '<ListBehavior-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListBehavior-request>) istream)
  "Deserializes a message object of type '<ListBehavior-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListBehavior-request>)))
  "Returns string type for a service object of type '<ListBehavior-request>"
  "behavior_msgs/ListBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListBehavior-request)))
  "Returns string type for a service object of type 'ListBehavior-request"
  "behavior_msgs/ListBehaviorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListBehavior-request>)))
  "Returns md5sum for a message object of type '<ListBehavior-request>"
  "db7d6a6bd85a4f8fde354aa4e08d9629")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListBehavior-request)))
  "Returns md5sum for a message object of type 'ListBehavior-request"
  "db7d6a6bd85a4f8fde354aa4e08d9629")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListBehavior-request>)))
  "Returns full string definition for message of type '<ListBehavior-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListBehavior-request)))
  "Returns full string definition for message of type 'ListBehavior-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListBehavior-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListBehavior-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ListBehavior-request
))
;//! \htmlinclude ListBehavior-response.msg.html

(cl:defclass <ListBehavior-response> (roslisp-msg-protocol:ros-message)
  ((loaded_behaviors
    :reader loaded_behaviors
    :initarg :loaded_behaviors
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (running_behaviors
    :reader running_behaviors
    :initarg :running_behaviors
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ListBehavior-response (<ListBehavior-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ListBehavior-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ListBehavior-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-srv:<ListBehavior-response> is deprecated: use behavior_msgs-srv:ListBehavior-response instead.")))

(cl:ensure-generic-function 'loaded_behaviors-val :lambda-list '(m))
(cl:defmethod loaded_behaviors-val ((m <ListBehavior-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:loaded_behaviors-val is deprecated.  Use behavior_msgs-srv:loaded_behaviors instead.")
  (loaded_behaviors m))

(cl:ensure-generic-function 'running_behaviors-val :lambda-list '(m))
(cl:defmethod running_behaviors-val ((m <ListBehavior-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:running_behaviors-val is deprecated.  Use behavior_msgs-srv:running_behaviors instead.")
  (running_behaviors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ListBehavior-response>) ostream)
  "Serializes a message object of type '<ListBehavior-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'loaded_behaviors))))
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
   (cl:slot-value msg 'loaded_behaviors))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'running_behaviors))))
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
   (cl:slot-value msg 'running_behaviors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ListBehavior-response>) istream)
  "Deserializes a message object of type '<ListBehavior-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'loaded_behaviors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'loaded_behaviors)))
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
  (cl:setf (cl:slot-value msg 'running_behaviors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'running_behaviors)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ListBehavior-response>)))
  "Returns string type for a service object of type '<ListBehavior-response>"
  "behavior_msgs/ListBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListBehavior-response)))
  "Returns string type for a service object of type 'ListBehavior-response"
  "behavior_msgs/ListBehaviorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ListBehavior-response>)))
  "Returns md5sum for a message object of type '<ListBehavior-response>"
  "db7d6a6bd85a4f8fde354aa4e08d9629")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ListBehavior-response)))
  "Returns md5sum for a message object of type 'ListBehavior-response"
  "db7d6a6bd85a4f8fde354aa4e08d9629")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ListBehavior-response>)))
  "Returns full string definition for message of type '<ListBehavior-response>"
  (cl:format cl:nil "string[] loaded_behaviors~%string[] running_behaviors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ListBehavior-response)))
  "Returns full string definition for message of type 'ListBehavior-response"
  (cl:format cl:nil "string[] loaded_behaviors~%string[] running_behaviors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ListBehavior-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'loaded_behaviors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'running_behaviors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ListBehavior-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ListBehavior-response
    (cl:cons ':loaded_behaviors (loaded_behaviors msg))
    (cl:cons ':running_behaviors (running_behaviors msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ListBehavior)))
  'ListBehavior-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ListBehavior)))
  'ListBehavior-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ListBehavior)))
  "Returns string type for a service object of type '<ListBehavior>"
  "behavior_msgs/ListBehavior")