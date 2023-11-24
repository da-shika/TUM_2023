; Auto-generated. Do not edit!


(cl:in-package behavior_msgs-srv)


;//! \htmlinclude ChangeContact-request.msg.html

(cl:defclass <ChangeContact-request> (roslisp-msg-protocol:ros-message)
  ((activate
    :reader activate
    :initarg :activate
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (deactivate
    :reader deactivate
    :initarg :deactivate
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass ChangeContact-request (<ChangeContact-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeContact-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeContact-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-srv:<ChangeContact-request> is deprecated: use behavior_msgs-srv:ChangeContact-request instead.")))

(cl:ensure-generic-function 'activate-val :lambda-list '(m))
(cl:defmethod activate-val ((m <ChangeContact-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:activate-val is deprecated.  Use behavior_msgs-srv:activate instead.")
  (activate m))

(cl:ensure-generic-function 'deactivate-val :lambda-list '(m))
(cl:defmethod deactivate-val ((m <ChangeContact-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:deactivate-val is deprecated.  Use behavior_msgs-srv:deactivate instead.")
  (deactivate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeContact-request>) ostream)
  "Serializes a message object of type '<ChangeContact-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'activate))))
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
   (cl:slot-value msg 'activate))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'deactivate))))
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
   (cl:slot-value msg 'deactivate))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeContact-request>) istream)
  "Deserializes a message object of type '<ChangeContact-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'activate) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'activate)))
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
  (cl:setf (cl:slot-value msg 'deactivate) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'deactivate)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeContact-request>)))
  "Returns string type for a service object of type '<ChangeContact-request>"
  "behavior_msgs/ChangeContactRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeContact-request)))
  "Returns string type for a service object of type 'ChangeContact-request"
  "behavior_msgs/ChangeContactRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeContact-request>)))
  "Returns md5sum for a message object of type '<ChangeContact-request>"
  "fb67b79c35378211caa96d9ff91d121b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeContact-request)))
  "Returns md5sum for a message object of type 'ChangeContact-request"
  "fb67b79c35378211caa96d9ff91d121b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeContact-request>)))
  "Returns full string definition for message of type '<ChangeContact-request>"
  (cl:format cl:nil "string[] activate~%string[] deactivate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeContact-request)))
  "Returns full string definition for message of type 'ChangeContact-request"
  (cl:format cl:nil "string[] activate~%string[] deactivate~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeContact-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'activate) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'deactivate) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeContact-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeContact-request
    (cl:cons ':activate (activate msg))
    (cl:cons ':deactivate (deactivate msg))
))
;//! \htmlinclude ChangeContact-response.msg.html

(cl:defclass <ChangeContact-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ChangeContact-response (<ChangeContact-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ChangeContact-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ChangeContact-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name behavior_msgs-srv:<ChangeContact-response> is deprecated: use behavior_msgs-srv:ChangeContact-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <ChangeContact-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader behavior_msgs-srv:ok-val is deprecated.  Use behavior_msgs-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ChangeContact-response>) ostream)
  "Serializes a message object of type '<ChangeContact-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ChangeContact-response>) istream)
  "Deserializes a message object of type '<ChangeContact-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ChangeContact-response>)))
  "Returns string type for a service object of type '<ChangeContact-response>"
  "behavior_msgs/ChangeContactResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeContact-response)))
  "Returns string type for a service object of type 'ChangeContact-response"
  "behavior_msgs/ChangeContactResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ChangeContact-response>)))
  "Returns md5sum for a message object of type '<ChangeContact-response>"
  "fb67b79c35378211caa96d9ff91d121b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ChangeContact-response)))
  "Returns md5sum for a message object of type 'ChangeContact-response"
  "fb67b79c35378211caa96d9ff91d121b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ChangeContact-response>)))
  "Returns full string definition for message of type '<ChangeContact-response>"
  (cl:format cl:nil "bool ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ChangeContact-response)))
  "Returns full string definition for message of type 'ChangeContact-response"
  (cl:format cl:nil "bool ok~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ChangeContact-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ChangeContact-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ChangeContact-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ChangeContact)))
  'ChangeContact-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ChangeContact)))
  'ChangeContact-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ChangeContact)))
  "Returns string type for a service object of type '<ChangeContact>"
  "behavior_msgs/ChangeContact")