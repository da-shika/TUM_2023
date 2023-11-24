; Auto-generated. Do not edit!


(cl:in-package control_core_msgs-msg)


;//! \htmlinclude SkinModality.msg.html

(cl:defclass <SkinModality> (roslisp-msg-protocol:ros-message)
  ((min
    :reader min
    :initarg :min
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (max
    :reader max
    :initarg :max
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (area
    :reader area
    :initarg :area
    :type std_msgs-msg:Float64
    :initform (cl:make-instance 'std_msgs-msg:Float64))
   (cop
    :reader cop
    :initarg :cop
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (wrench
    :reader wrench
    :initarg :wrench
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (hull
    :reader hull
    :initarg :hull
    :type geometry_msgs-msg:Polygon
    :initform (cl:make-instance 'geometry_msgs-msg:Polygon)))
)

(cl:defclass SkinModality (<SkinModality>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SkinModality>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SkinModality)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name control_core_msgs-msg:<SkinModality> is deprecated: use control_core_msgs-msg:SkinModality instead.")))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <SkinModality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:min-val is deprecated.  Use control_core_msgs-msg:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <SkinModality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:max-val is deprecated.  Use control_core_msgs-msg:max instead.")
  (max m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <SkinModality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:area-val is deprecated.  Use control_core_msgs-msg:area instead.")
  (area m))

(cl:ensure-generic-function 'cop-val :lambda-list '(m))
(cl:defmethod cop-val ((m <SkinModality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:cop-val is deprecated.  Use control_core_msgs-msg:cop instead.")
  (cop m))

(cl:ensure-generic-function 'wrench-val :lambda-list '(m))
(cl:defmethod wrench-val ((m <SkinModality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:wrench-val is deprecated.  Use control_core_msgs-msg:wrench instead.")
  (wrench m))

(cl:ensure-generic-function 'hull-val :lambda-list '(m))
(cl:defmethod hull-val ((m <SkinModality>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader control_core_msgs-msg:hull-val is deprecated.  Use control_core_msgs-msg:hull instead.")
  (hull m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SkinModality>) ostream)
  "Serializes a message object of type '<SkinModality>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'area) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cop) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'hull) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SkinModality>) istream)
  "Deserializes a message object of type '<SkinModality>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'area) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cop) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'hull) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SkinModality>)))
  "Returns string type for a message object of type '<SkinModality>"
  "control_core_msgs/SkinModality")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SkinModality)))
  "Returns string type for a message object of type 'SkinModality"
  "control_core_msgs/SkinModality")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SkinModality>)))
  "Returns md5sum for a message object of type '<SkinModality>"
  "db74e377abd390ca1fca36f1294853e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SkinModality)))
  "Returns md5sum for a message object of type 'SkinModality"
  "db74e377abd390ca1fca36f1294853e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SkinModality>)))
  "Returns full string definition for message of type '<SkinModality>"
  (cl:format cl:nil "std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SkinModality)))
  "Returns full string definition for message of type 'SkinModality"
  (cl:format cl:nil "std_msgs/Float64 min~%std_msgs/Float64 max~%std_msgs/Float64 area~%geometry_msgs/Point cop~%geometry_msgs/Wrench wrench~%geometry_msgs/Polygon hull~%================================================================================~%MSG: std_msgs/Float64~%float64 data~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, separated into~%# its linear and angular parts.~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Polygon~%#A specification of a polygon where the first and last points are assumed to be connected~%Point32[] points~%~%================================================================================~%MSG: geometry_msgs/Point32~%# This contains the position of a point in free space(with 32 bits of precision).~%# It is recommeded to use Point wherever possible instead of Point32.  ~%# ~%# This recommendation is to promote interoperability.  ~%#~%# This message is designed to take up less space when sending~%# lots of points at once, as in the case of a PointCloud.  ~%~%float32 x~%float32 y~%float32 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SkinModality>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'area))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cop))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'hull))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SkinModality>))
  "Converts a ROS message object to a list"
  (cl:list 'SkinModality
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
    (cl:cons ':area (area msg))
    (cl:cons ':cop (cop msg))
    (cl:cons ':wrench (wrench msg))
    (cl:cons ':hull (hull msg))
))
