; Auto-generated. Do not edit!


(cl:in-package ur_script_manager-srv)


;//! \htmlinclude setScriptManagerState-request.msg.html

(cl:defclass <setScriptManagerState-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (enable
    :reader enable
    :initarg :enable
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setScriptManagerState-request (<setScriptManagerState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setScriptManagerState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setScriptManagerState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_script_manager-srv:<setScriptManagerState-request> is deprecated: use ur_script_manager-srv:setScriptManagerState-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <setScriptManagerState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_script_manager-srv:name-val is deprecated.  Use ur_script_manager-srv:name instead.")
  (name m))

(cl:ensure-generic-function 'enable-val :lambda-list '(m))
(cl:defmethod enable-val ((m <setScriptManagerState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_script_manager-srv:enable-val is deprecated.  Use ur_script_manager-srv:enable instead.")
  (enable m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setScriptManagerState-request>) ostream)
  "Serializes a message object of type '<setScriptManagerState-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enable) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setScriptManagerState-request>) istream)
  "Deserializes a message object of type '<setScriptManagerState-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'enable) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setScriptManagerState-request>)))
  "Returns string type for a service object of type '<setScriptManagerState-request>"
  "ur_script_manager/setScriptManagerStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setScriptManagerState-request)))
  "Returns string type for a service object of type 'setScriptManagerState-request"
  "ur_script_manager/setScriptManagerStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setScriptManagerState-request>)))
  "Returns md5sum for a message object of type '<setScriptManagerState-request>"
  "f7e852bc0e000ce749738a3d7c423da7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setScriptManagerState-request)))
  "Returns md5sum for a message object of type 'setScriptManagerState-request"
  "f7e852bc0e000ce749738a3d7c423da7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setScriptManagerState-request>)))
  "Returns full string definition for message of type '<setScriptManagerState-request>"
  (cl:format cl:nil "string name	# the name of the script sub program ~%bool enable	# enable/disable program~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setScriptManagerState-request)))
  "Returns full string definition for message of type 'setScriptManagerState-request"
  (cl:format cl:nil "string name	# the name of the script sub program ~%bool enable	# enable/disable program~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setScriptManagerState-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setScriptManagerState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'setScriptManagerState-request
    (cl:cons ':name (name msg))
    (cl:cons ':enable (enable msg))
))
;//! \htmlinclude setScriptManagerState-response.msg.html

(cl:defclass <setScriptManagerState-response> (roslisp-msg-protocol:ros-message)
  ((ok
    :reader ok
    :initarg :ok
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass setScriptManagerState-response (<setScriptManagerState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <setScriptManagerState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'setScriptManagerState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ur_script_manager-srv:<setScriptManagerState-response> is deprecated: use ur_script_manager-srv:setScriptManagerState-response instead.")))

(cl:ensure-generic-function 'ok-val :lambda-list '(m))
(cl:defmethod ok-val ((m <setScriptManagerState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ur_script_manager-srv:ok-val is deprecated.  Use ur_script_manager-srv:ok instead.")
  (ok m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <setScriptManagerState-response>) ostream)
  "Serializes a message object of type '<setScriptManagerState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ok) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <setScriptManagerState-response>) istream)
  "Deserializes a message object of type '<setScriptManagerState-response>"
    (cl:setf (cl:slot-value msg 'ok) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<setScriptManagerState-response>)))
  "Returns string type for a service object of type '<setScriptManagerState-response>"
  "ur_script_manager/setScriptManagerStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setScriptManagerState-response)))
  "Returns string type for a service object of type 'setScriptManagerState-response"
  "ur_script_manager/setScriptManagerStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<setScriptManagerState-response>)))
  "Returns md5sum for a message object of type '<setScriptManagerState-response>"
  "f7e852bc0e000ce749738a3d7c423da7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'setScriptManagerState-response)))
  "Returns md5sum for a message object of type 'setScriptManagerState-response"
  "f7e852bc0e000ce749738a3d7c423da7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<setScriptManagerState-response>)))
  "Returns full string definition for message of type '<setScriptManagerState-response>"
  (cl:format cl:nil "bool ok		# change of the state was successfull/failed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'setScriptManagerState-response)))
  "Returns full string definition for message of type 'setScriptManagerState-response"
  (cl:format cl:nil "bool ok		# change of the state was successfull/failed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <setScriptManagerState-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <setScriptManagerState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'setScriptManagerState-response
    (cl:cons ':ok (ok msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'setScriptManagerState)))
  'setScriptManagerState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'setScriptManagerState)))
  'setScriptManagerState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'setScriptManagerState)))
  "Returns string type for a service object of type '<setScriptManagerState>"
  "ur_script_manager/setScriptManagerState")