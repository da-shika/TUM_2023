; Auto-generated. Do not edit!


(cl:in-package walking_core_msgs-msg)


;//! \htmlinclude FootStep.msg.html

(cl:defclass <FootStep> (roslisp-msg-protocol:ros-message)
  ((contact
    :reader contact
    :initarg :contact
    :type control_core_msgs-msg:Contact
    :initform (cl:make-instance 'control_core_msgs-msg:Contact))
   (body_id
    :reader body_id
    :initarg :body_id
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64))
   (final_step
    :reader final_step
    :initarg :final_step
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool))
   (n_step
    :reader n_step
    :initarg :n_step
    :type std_msgs-msg:Int64
    :initform (cl:make-instance 'std_msgs-msg:Int64)))
)

(cl:defclass FootStep (<FootStep>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootStep>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootStep)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name walking_core_msgs-msg:<FootStep> is deprecated: use walking_core_msgs-msg:FootStep instead.")))

(cl:ensure-generic-function 'contact-val :lambda-list '(m))
(cl:defmethod contact-val ((m <FootStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:contact-val is deprecated.  Use walking_core_msgs-msg:contact instead.")
  (contact m))

(cl:ensure-generic-function 'body_id-val :lambda-list '(m))
(cl:defmethod body_id-val ((m <FootStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:body_id-val is deprecated.  Use walking_core_msgs-msg:body_id instead.")
  (body_id m))

(cl:ensure-generic-function 'final_step-val :lambda-list '(m))
(cl:defmethod final_step-val ((m <FootStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:final_step-val is deprecated.  Use walking_core_msgs-msg:final_step instead.")
  (final_step m))

(cl:ensure-generic-function 'n_step-val :lambda-list '(m))
(cl:defmethod n_step-val ((m <FootStep>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader walking_core_msgs-msg:n_step-val is deprecated.  Use walking_core_msgs-msg:n_step instead.")
  (n_step m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootStep>) ostream)
  "Serializes a message object of type '<FootStep>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'contact) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'body_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'final_step) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'n_step) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootStep>) istream)
  "Deserializes a message object of type '<FootStep>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'contact) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'body_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'final_step) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'n_step) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootStep>)))
  "Returns string type for a message object of type '<FootStep>"
  "walking_core_msgs/FootStep")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootStep)))
  "Returns string type for a message object of type 'FootStep"
  "walking_core_msgs/FootStep")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootStep>)))
  "Returns md5sum for a message object of type '<FootStep>"
  "9d0f09bb8a0492982eab7b79dd33028a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootStep)))
  "Returns md5sum for a message object of type 'FootStep"
  "9d0f09bb8a0492982eab7b79dd33028a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootStep>)))
  "Returns full string definition for message of type '<FootStep>"
  (cl:format cl:nil "control_core_msgs/Contact contact~%std_msgs/Int64 body_id~%std_msgs/Bool final_step~%std_msgs/Int64 n_step~%================================================================================~%MSG: control_core_msgs/Contact~%geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootStep)))
  "Returns full string definition for message of type 'FootStep"
  (cl:format cl:nil "control_core_msgs/Contact contact~%std_msgs/Int64 body_id~%std_msgs/Bool final_step~%std_msgs/Int64 n_step~%================================================================================~%MSG: control_core_msgs/Contact~%geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: std_msgs/Int64~%int64 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootStep>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'contact))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'body_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'final_step))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'n_step))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootStep>))
  "Converts a ROS message object to a list"
  (cl:list 'FootStep
    (cl:cons ':contact (contact msg))
    (cl:cons ':body_id (body_id msg))
    (cl:cons ':final_step (final_step msg))
    (cl:cons ':n_step (n_step msg))
))
