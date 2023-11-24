; Auto-generated. Do not edit!


(cl:in-package ur_script_manager-srv)


;//! \htmlinclude getScriptManagerStates-request.msg.html

(cl:defclass <getScriptManagerStates-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getScriptManagerStates-request (<getScriptManagerStates-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getScriptManagerStates-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getScriptManagerStates-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_script_manager-srv:<getScriptManagerStates-request> is deprecated: use ur_script_manager-srv:getScriptManagerStates-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getScriptManagerStates-request>) ostream)
  "Serializes a message object of type '<getScriptManagerStates-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getScriptManagerStates-request>) istream)
  "Deserializes a message object of type '<getScriptManagerStates-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getScriptManagerStates-request>)))
  "Returns string type for a service object of type '<getScriptManagerStates-request>"
  "ur_script_manager/getScriptManagerStatesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getScriptManagerStates-request)))
  "Returns string type for a service object of type 'getScriptManagerStates-request"
  "ur_script_manager/getScriptManagerStatesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getScriptManagerStates-request>)))
  "Returns md5sum for a message object of type '<getScriptManagerStates-request>"
  "44bb719088af7b4a0bc3023901924bc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getScriptManagerStates-request)))
  "Returns md5sum for a message object of type 'getScriptManagerStates-request"
  "44bb719088af7b4a0bc3023901924bc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getScriptManagerStates-request>)))
  "Returns full string definition for message of type '<getScriptManagerStates-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getScriptManagerStates-request)))
  "Returns full string definition for message of type 'getScriptManagerStates-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getScriptManagerStates-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getScriptManagerStates-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getScriptManagerStates-request
))
;//! \htmlinclude getScriptManagerStates-response.msg.html

(cl:defclass <getScriptManagerStates-response> (roslisp-msg-protocol:ros-message)
  ((names
    :reader names
    :initarg :names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (enabled
    :reader enabled
    :initarg :enabled
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass getScriptManagerStates-response (<getScriptManagerStates-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getScriptManagerStates-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getScriptManagerStates-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_script_manager-srv:<getScriptManagerStates-response> is deprecated: use ur_script_manager-srv:getScriptManagerStates-response instead.")))

(cl:ensure-generic-function 'names-val :lambda-list '(m))
(cl:defmethod names-val ((m <getScriptManagerStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_script_manager-srv:names-val is deprecated.  Use ur_script_manager-srv:names instead.")
  (names m))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <getScriptManagerStates-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_script_manager-srv:enabled-val is deprecated.  Use ur_script_manager-srv:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getScriptManagerStates-response>) ostream)
  "Serializes a message object of type '<getScriptManagerStates-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'names))))
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
   (cl:slot-value msg 'names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'enabled))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'enabled))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getScriptManagerStates-response>) istream)
  "Deserializes a message object of type '<getScriptManagerStates-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'names)))
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
  (cl:setf (cl:slot-value msg 'enabled) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'enabled)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getScriptManagerStates-response>)))
  "Returns string type for a service object of type '<getScriptManagerStates-response>"
  "ur_script_manager/getScriptManagerStatesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getScriptManagerStates-response)))
  "Returns string type for a service object of type 'getScriptManagerStates-response"
  "ur_script_manager/getScriptManagerStatesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getScriptManagerStates-response>)))
  "Returns md5sum for a message object of type '<getScriptManagerStates-response>"
  "44bb719088af7b4a0bc3023901924bc8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getScriptManagerStates-response)))
  "Returns md5sum for a message object of type 'getScriptManagerStates-response"
  "44bb719088af7b4a0bc3023901924bc8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getScriptManagerStates-response>)))
  "Returns full string definition for message of type '<getScriptManagerStates-response>"
  (cl:format cl:nil "string[] names		# the names of the script sub programs~%bool[] enabled		# the states of the script sub programs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getScriptManagerStates-response)))
  "Returns full string definition for message of type 'getScriptManagerStates-response"
  (cl:format cl:nil "string[] names		# the names of the script sub programs~%bool[] enabled		# the states of the script sub programs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getScriptManagerStates-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'enabled) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getScriptManagerStates-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getScriptManagerStates-response
    (cl:cons ':names (names msg))
    (cl:cons ':enabled (enabled msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getScriptManagerStates)))
  'getScriptManagerStates-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getScriptManagerStates)))
  'getScriptManagerStates-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getScriptManagerStates)))
  "Returns string type for a service object of type '<getScriptManagerStates>"
  "ur_script_manager/getScriptManagerStates")