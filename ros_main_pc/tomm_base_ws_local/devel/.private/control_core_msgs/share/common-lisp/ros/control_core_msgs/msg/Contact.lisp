; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude Contact.msg.html

(cl:defclass <Contact> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (hull
    :reader hull
    :initarg :hull
    :type geometry_msgs-msg:Polygon
    :initform (cl:make-instance 'geometry_msgs-msg:Polygon))
   (offset
    :reader offset
    :initarg :offset
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (friction
    :reader friction
    :initarg :friction
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64)))
)

(cl:defclass Contact (<Contact>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Contact>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Contact)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<Contact> is deprecated: use control_core_msgs-msg:Contact instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:pose-val is deprecated.  Use control_core_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'hull-val :lambda-list '(m))
(cl:defmethod hull-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:hull-val is deprecated.  Use control_core_msgs-msg:hull instead.")
  (hull m))

(cl:ensure-generic-function 'offset-val :lambda-list '(m))
(cl:defmethod offset-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:offset-val is deprecated.  Use control_core_msgs-msg:offset instead.")
  (offset m))

(cl:ensure-generic-function 'friction-val :lambda-list '(m))
(cl:defmethod friction-val ((m <Contact>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:friction-val is deprecated.  Use control_core_msgs-msg:friction instead.")
  (friction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Contact>) ostream)
  "Serializes a message object of type '<Contact>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hull) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'offset) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'friction) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Contact>) istream)
  "Deserializes a message object of type '<Contact>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hull) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'offset) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'friction) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Contact>)))
  "Returns string type for a message object of type '<Contact>"
  "control_core_msgs/Contact")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Contact)))
  "Returns string type for a message object of type 'Contact"
  "control_core_msgs/Contact")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Contact>)))
  "Returns md5sum for a message object of type '<Contact>"
  "092ef89bc484cc9b681ba742d6241a00")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Contact)))
  "Returns md5sum for a message object of type 'Contact"
  "092ef89bc484cc9b681ba742d6241a00")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Contact>)))
  "Returns full string definition for message of type '<Contact>"
  (cl:format cl:nil "geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Contact)))
  "Returns full string definition for message of type 'Contact"
  (cl:format cl:nil "geometry_msgs/Pose pose~%geometry_msgs/Polygon hull~%geometry_msgs/Point offset~%std_msgs/Float64 friction~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Contact>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hull))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'offset))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'friction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Contact>))
  "Converts a ROS message object to a list"
  (cl:list 'Contact
    (cl:cons ':pose (pose msg))
    (cl:cons ':hull (hull msg))
    (cl:cons ':offset (offset msg))
    (cl:cons ':friction (friction msg))
))
