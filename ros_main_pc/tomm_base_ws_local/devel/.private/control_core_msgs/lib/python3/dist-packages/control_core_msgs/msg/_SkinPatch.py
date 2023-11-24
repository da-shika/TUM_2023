# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from control_core_msgs/SkinPatch.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import control_core_msgs.msg
import geometry_msgs.msg
import std_msgs.msg

class SkinPatch(genpy.Message):
  _md5sum = "a5c4ba5fbedd1c63abbdd1a1450dda87"
  _type = "control_core_msgs/SkinPatch"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
geometry_msgs/Pose pose
SkinModality force
SkinModality proximity
std_msgs/Float64 min_dist
std_msgs/Float64 max_dist
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: control_core_msgs/SkinModality
std_msgs/Float64 min
std_msgs/Float64 max
std_msgs/Float64 area
geometry_msgs/Point cop
geometry_msgs/Wrench wrench
geometry_msgs/Polygon hull
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: geometry_msgs/Wrench
# This represents force in free space, separated into
# its linear and angular parts.
Vector3  force
Vector3  torque

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Polygon
#A specification of a polygon where the first and last points are assumed to be connected
Point32[] points

================================================================================
MSG: geometry_msgs/Point32
# This contains the position of a point in free space(with 32 bits of precision).
# It is recommeded to use Point wherever possible instead of Point32.  
# 
# This recommendation is to promote interoperability.  
#
# This message is designed to take up less space when sending
# lots of points at once, as in the case of a PointCloud.  

float32 x
float32 y
float32 z"""
  __slots__ = ['header','pose','force','proximity','min_dist','max_dist']
  _slot_types = ['std_msgs/Header','geometry_msgs/Pose','control_core_msgs/SkinModality','control_core_msgs/SkinModality','std_msgs/Float64','std_msgs/Float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,pose,force,proximity,min_dist,max_dist

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(SkinPatch, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.force is None:
        self.force = control_core_msgs.msg.SkinModality()
      if self.proximity is None:
        self.proximity = control_core_msgs.msg.SkinModality()
      if self.min_dist is None:
        self.min_dist = std_msgs.msg.Float64()
      if self.max_dist is None:
        self.max_dist = std_msgs.msg.Float64()
    else:
      self.header = std_msgs.msg.Header()
      self.pose = geometry_msgs.msg.Pose()
      self.force = control_core_msgs.msg.SkinModality()
      self.proximity = control_core_msgs.msg.SkinModality()
      self.min_dist = std_msgs.msg.Float64()
      self.max_dist = std_msgs.msg.Float64()

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_19d().pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.force.min.data, _x.force.max.data, _x.force.area.data, _x.force.cop.x, _x.force.cop.y, _x.force.cop.z, _x.force.wrench.force.x, _x.force.wrench.force.y, _x.force.wrench.force.z, _x.force.wrench.torque.x, _x.force.wrench.torque.y, _x.force.wrench.torque.z))
      length = len(self.force.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.force.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_12d().pack(_x.proximity.min.data, _x.proximity.max.data, _x.proximity.area.data, _x.proximity.cop.x, _x.proximity.cop.y, _x.proximity.cop.z, _x.proximity.wrench.force.x, _x.proximity.wrench.force.y, _x.proximity.wrench.force.z, _x.proximity.wrench.torque.x, _x.proximity.wrench.torque.y, _x.proximity.wrench.torque.z))
      length = len(self.proximity.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.proximity.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_2d().pack(_x.min_dist.data, _x.max_dist.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.force is None:
        self.force = control_core_msgs.msg.SkinModality()
      if self.proximity is None:
        self.proximity = control_core_msgs.msg.SkinModality()
      if self.min_dist is None:
        self.min_dist = std_msgs.msg.Float64()
      if self.max_dist is None:
        self.max_dist = std_msgs.msg.Float64()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 152
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.force.min.data, _x.force.max.data, _x.force.area.data, _x.force.cop.x, _x.force.cop.y, _x.force.cop.z, _x.force.wrench.force.x, _x.force.wrench.force.y, _x.force.wrench.force.z, _x.force.wrench.torque.x, _x.force.wrench.torque.y, _x.force.wrench.torque.z,) = _get_struct_19d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.force.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.force.hull.points.append(val1)
      _x = self
      start = end
      end += 96
      (_x.proximity.min.data, _x.proximity.max.data, _x.proximity.area.data, _x.proximity.cop.x, _x.proximity.cop.y, _x.proximity.cop.z, _x.proximity.wrench.force.x, _x.proximity.wrench.force.y, _x.proximity.wrench.force.z, _x.proximity.wrench.torque.x, _x.proximity.wrench.torque.y, _x.proximity.wrench.torque.z,) = _get_struct_12d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.proximity.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.proximity.hull.points.append(val1)
      _x = self
      start = end
      end += 16
      (_x.min_dist.data, _x.max_dist.data,) = _get_struct_2d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_3I().pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      _x = self
      buff.write(_get_struct_19d().pack(_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.force.min.data, _x.force.max.data, _x.force.area.data, _x.force.cop.x, _x.force.cop.y, _x.force.cop.z, _x.force.wrench.force.x, _x.force.wrench.force.y, _x.force.wrench.force.z, _x.force.wrench.torque.x, _x.force.wrench.torque.y, _x.force.wrench.torque.z))
      length = len(self.force.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.force.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_12d().pack(_x.proximity.min.data, _x.proximity.max.data, _x.proximity.area.data, _x.proximity.cop.x, _x.proximity.cop.y, _x.proximity.cop.z, _x.proximity.wrench.force.x, _x.proximity.wrench.force.y, _x.proximity.wrench.force.z, _x.proximity.wrench.torque.x, _x.proximity.wrench.torque.y, _x.proximity.wrench.torque.z))
      length = len(self.proximity.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.proximity.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_2d().pack(_x.min_dist.data, _x.max_dist.data))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.pose is None:
        self.pose = geometry_msgs.msg.Pose()
      if self.force is None:
        self.force = control_core_msgs.msg.SkinModality()
      if self.proximity is None:
        self.proximity = control_core_msgs.msg.SkinModality()
      if self.min_dist is None:
        self.min_dist = std_msgs.msg.Float64()
      if self.max_dist is None:
        self.max_dist = std_msgs.msg.Float64()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _get_struct_3I().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 152
      (_x.pose.position.x, _x.pose.position.y, _x.pose.position.z, _x.pose.orientation.x, _x.pose.orientation.y, _x.pose.orientation.z, _x.pose.orientation.w, _x.force.min.data, _x.force.max.data, _x.force.area.data, _x.force.cop.x, _x.force.cop.y, _x.force.cop.z, _x.force.wrench.force.x, _x.force.wrench.force.y, _x.force.wrench.force.z, _x.force.wrench.torque.x, _x.force.wrench.torque.y, _x.force.wrench.torque.z,) = _get_struct_19d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.force.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.force.hull.points.append(val1)
      _x = self
      start = end
      end += 96
      (_x.proximity.min.data, _x.proximity.max.data, _x.proximity.area.data, _x.proximity.cop.x, _x.proximity.cop.y, _x.proximity.cop.z, _x.proximity.wrench.force.x, _x.proximity.wrench.force.y, _x.proximity.wrench.force.z, _x.proximity.wrench.torque.x, _x.proximity.wrench.torque.y, _x.proximity.wrench.torque.z,) = _get_struct_12d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.proximity.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.proximity.hull.points.append(val1)
      _x = self
      start = end
      end += 16
      (_x.min_dist.data, _x.max_dist.data,) = _get_struct_2d().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_12d = None
def _get_struct_12d():
    global _struct_12d
    if _struct_12d is None:
        _struct_12d = struct.Struct("<12d")
    return _struct_12d
_struct_19d = None
def _get_struct_19d():
    global _struct_19d
    if _struct_19d is None:
        _struct_19d = struct.Struct("<19d")
    return _struct_19d
_struct_2d = None
def _get_struct_2d():
    global _struct_2d
    if _struct_2d is None:
        _struct_2d = struct.Struct("<2d")
    return _struct_2d
_struct_3I = None
def _get_struct_3I():
    global _struct_3I
    if _struct_3I is None:
        _struct_3I = struct.Struct("<3I")
    return _struct_3I
_struct_3f = None
def _get_struct_3f():
    global _struct_3f
    if _struct_3f is None:
        _struct_3f = struct.Struct("<3f")
    return _struct_3f
