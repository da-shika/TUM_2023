# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from walking_core_msgs/WalkingStates.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import control_core_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
import walking_core_msgs.msg

class WalkingStates(genpy.Message):
  _md5sum = "e12756d011ab43b51ff2b08d69aadcb4"
  _type = "walking_core_msgs/WalkingStates"
  _has_header = True  # flag to mark the presence of a Header object
  _full_text = """std_msgs/Header header
WalkingPhase phase
control_core_msgs/BodyId stance_foot
control_core_msgs/BodyId swing_foot
std_msgs/Float64 elapsed
std_msgs/Float64 left_foot_ratio
FootStep cur_step
FootStep target_step
FootStep next_step
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
MSG: walking_core_msgs/WalkingPhase
uint8 PHASE_STANCE=0
uint8 PHASE_DOUBLESUPPORT=1
uint8 PHASE_SINGLESUPPORT=2
uint8 phase
================================================================================
MSG: control_core_msgs/BodyId
uint8 ID_LEFT_FOOT=0
uint8 ID_RIGHT_FOOT=1
uint8 ID_LEFT_HAND=2
uint8 ID_RIGHT_HAND=3
uint8 id
================================================================================
MSG: std_msgs/Float64
float64 data
================================================================================
MSG: walking_core_msgs/FootStep
control_core_msgs/Contact contact
std_msgs/Int64 body_id
std_msgs/Bool final_step
std_msgs/Int64 n_step
================================================================================
MSG: control_core_msgs/Contact
geometry_msgs/Pose pose
geometry_msgs/Polygon hull
geometry_msgs/Point offset
std_msgs/Float64 friction
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
float32 z
================================================================================
MSG: std_msgs/Int64
int64 data
================================================================================
MSG: std_msgs/Bool
bool data"""
  __slots__ = ['header','phase','stance_foot','swing_foot','elapsed','left_foot_ratio','cur_step','target_step','next_step']
  _slot_types = ['std_msgs/Header','walking_core_msgs/WalkingPhase','control_core_msgs/BodyId','control_core_msgs/BodyId','std_msgs/Float64','std_msgs/Float64','walking_core_msgs/FootStep','walking_core_msgs/FootStep','walking_core_msgs/FootStep']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,phase,stance_foot,swing_foot,elapsed,left_foot_ratio,cur_step,target_step,next_step

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(WalkingStates, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.phase is None:
        self.phase = walking_core_msgs.msg.WalkingPhase()
      if self.stance_foot is None:
        self.stance_foot = control_core_msgs.msg.BodyId()
      if self.swing_foot is None:
        self.swing_foot = control_core_msgs.msg.BodyId()
      if self.elapsed is None:
        self.elapsed = std_msgs.msg.Float64()
      if self.left_foot_ratio is None:
        self.left_foot_ratio = std_msgs.msg.Float64()
      if self.cur_step is None:
        self.cur_step = walking_core_msgs.msg.FootStep()
      if self.target_step is None:
        self.target_step = walking_core_msgs.msg.FootStep()
      if self.next_step is None:
        self.next_step = walking_core_msgs.msg.FootStep()
    else:
      self.header = std_msgs.msg.Header()
      self.phase = walking_core_msgs.msg.WalkingPhase()
      self.stance_foot = control_core_msgs.msg.BodyId()
      self.swing_foot = control_core_msgs.msg.BodyId()
      self.elapsed = std_msgs.msg.Float64()
      self.left_foot_ratio = std_msgs.msg.Float64()
      self.cur_step = walking_core_msgs.msg.FootStep()
      self.target_step = walking_core_msgs.msg.FootStep()
      self.next_step = walking_core_msgs.msg.FootStep()

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
      buff.write(_get_struct_3B9d().pack(_x.phase.phase, _x.stance_foot.id, _x.swing_foot.id, _x.elapsed.data, _x.left_foot_ratio.data, _x.cur_step.contact.pose.position.x, _x.cur_step.contact.pose.position.y, _x.cur_step.contact.pose.position.z, _x.cur_step.contact.pose.orientation.x, _x.cur_step.contact.pose.orientation.y, _x.cur_step.contact.pose.orientation.z, _x.cur_step.contact.pose.orientation.w))
      length = len(self.cur_step.contact.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.cur_step.contact.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_4dqBq7d().pack(_x.cur_step.contact.offset.x, _x.cur_step.contact.offset.y, _x.cur_step.contact.offset.z, _x.cur_step.contact.friction.data, _x.cur_step.body_id.data, _x.cur_step.final_step.data, _x.cur_step.n_step.data, _x.target_step.contact.pose.position.x, _x.target_step.contact.pose.position.y, _x.target_step.contact.pose.position.z, _x.target_step.contact.pose.orientation.x, _x.target_step.contact.pose.orientation.y, _x.target_step.contact.pose.orientation.z, _x.target_step.contact.pose.orientation.w))
      length = len(self.target_step.contact.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.target_step.contact.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_4dqBq7d().pack(_x.target_step.contact.offset.x, _x.target_step.contact.offset.y, _x.target_step.contact.offset.z, _x.target_step.contact.friction.data, _x.target_step.body_id.data, _x.target_step.final_step.data, _x.target_step.n_step.data, _x.next_step.contact.pose.position.x, _x.next_step.contact.pose.position.y, _x.next_step.contact.pose.position.z, _x.next_step.contact.pose.orientation.x, _x.next_step.contact.pose.orientation.y, _x.next_step.contact.pose.orientation.z, _x.next_step.contact.pose.orientation.w))
      length = len(self.next_step.contact.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.next_step.contact.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_4dqBq().pack(_x.next_step.contact.offset.x, _x.next_step.contact.offset.y, _x.next_step.contact.offset.z, _x.next_step.contact.friction.data, _x.next_step.body_id.data, _x.next_step.final_step.data, _x.next_step.n_step.data))
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
      if self.phase is None:
        self.phase = walking_core_msgs.msg.WalkingPhase()
      if self.stance_foot is None:
        self.stance_foot = control_core_msgs.msg.BodyId()
      if self.swing_foot is None:
        self.swing_foot = control_core_msgs.msg.BodyId()
      if self.elapsed is None:
        self.elapsed = std_msgs.msg.Float64()
      if self.left_foot_ratio is None:
        self.left_foot_ratio = std_msgs.msg.Float64()
      if self.cur_step is None:
        self.cur_step = walking_core_msgs.msg.FootStep()
      if self.target_step is None:
        self.target_step = walking_core_msgs.msg.FootStep()
      if self.next_step is None:
        self.next_step = walking_core_msgs.msg.FootStep()
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
      end += 75
      (_x.phase.phase, _x.stance_foot.id, _x.swing_foot.id, _x.elapsed.data, _x.left_foot_ratio.data, _x.cur_step.contact.pose.position.x, _x.cur_step.contact.pose.position.y, _x.cur_step.contact.pose.position.z, _x.cur_step.contact.pose.orientation.x, _x.cur_step.contact.pose.orientation.y, _x.cur_step.contact.pose.orientation.z, _x.cur_step.contact.pose.orientation.w,) = _get_struct_3B9d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cur_step.contact.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.cur_step.contact.hull.points.append(val1)
      _x = self
      start = end
      end += 105
      (_x.cur_step.contact.offset.x, _x.cur_step.contact.offset.y, _x.cur_step.contact.offset.z, _x.cur_step.contact.friction.data, _x.cur_step.body_id.data, _x.cur_step.final_step.data, _x.cur_step.n_step.data, _x.target_step.contact.pose.position.x, _x.target_step.contact.pose.position.y, _x.target_step.contact.pose.position.z, _x.target_step.contact.pose.orientation.x, _x.target_step.contact.pose.orientation.y, _x.target_step.contact.pose.orientation.z, _x.target_step.contact.pose.orientation.w,) = _get_struct_4dqBq7d().unpack(str[start:end])
      self.cur_step.final_step.data = bool(self.cur_step.final_step.data)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.target_step.contact.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.target_step.contact.hull.points.append(val1)
      _x = self
      start = end
      end += 105
      (_x.target_step.contact.offset.x, _x.target_step.contact.offset.y, _x.target_step.contact.offset.z, _x.target_step.contact.friction.data, _x.target_step.body_id.data, _x.target_step.final_step.data, _x.target_step.n_step.data, _x.next_step.contact.pose.position.x, _x.next_step.contact.pose.position.y, _x.next_step.contact.pose.position.z, _x.next_step.contact.pose.orientation.x, _x.next_step.contact.pose.orientation.y, _x.next_step.contact.pose.orientation.z, _x.next_step.contact.pose.orientation.w,) = _get_struct_4dqBq7d().unpack(str[start:end])
      self.target_step.final_step.data = bool(self.target_step.final_step.data)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.next_step.contact.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.next_step.contact.hull.points.append(val1)
      _x = self
      start = end
      end += 49
      (_x.next_step.contact.offset.x, _x.next_step.contact.offset.y, _x.next_step.contact.offset.z, _x.next_step.contact.friction.data, _x.next_step.body_id.data, _x.next_step.final_step.data, _x.next_step.n_step.data,) = _get_struct_4dqBq().unpack(str[start:end])
      self.next_step.final_step.data = bool(self.next_step.final_step.data)
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
      buff.write(_get_struct_3B9d().pack(_x.phase.phase, _x.stance_foot.id, _x.swing_foot.id, _x.elapsed.data, _x.left_foot_ratio.data, _x.cur_step.contact.pose.position.x, _x.cur_step.contact.pose.position.y, _x.cur_step.contact.pose.position.z, _x.cur_step.contact.pose.orientation.x, _x.cur_step.contact.pose.orientation.y, _x.cur_step.contact.pose.orientation.z, _x.cur_step.contact.pose.orientation.w))
      length = len(self.cur_step.contact.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.cur_step.contact.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_4dqBq7d().pack(_x.cur_step.contact.offset.x, _x.cur_step.contact.offset.y, _x.cur_step.contact.offset.z, _x.cur_step.contact.friction.data, _x.cur_step.body_id.data, _x.cur_step.final_step.data, _x.cur_step.n_step.data, _x.target_step.contact.pose.position.x, _x.target_step.contact.pose.position.y, _x.target_step.contact.pose.position.z, _x.target_step.contact.pose.orientation.x, _x.target_step.contact.pose.orientation.y, _x.target_step.contact.pose.orientation.z, _x.target_step.contact.pose.orientation.w))
      length = len(self.target_step.contact.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.target_step.contact.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_4dqBq7d().pack(_x.target_step.contact.offset.x, _x.target_step.contact.offset.y, _x.target_step.contact.offset.z, _x.target_step.contact.friction.data, _x.target_step.body_id.data, _x.target_step.final_step.data, _x.target_step.n_step.data, _x.next_step.contact.pose.position.x, _x.next_step.contact.pose.position.y, _x.next_step.contact.pose.position.z, _x.next_step.contact.pose.orientation.x, _x.next_step.contact.pose.orientation.y, _x.next_step.contact.pose.orientation.z, _x.next_step.contact.pose.orientation.w))
      length = len(self.next_step.contact.hull.points)
      buff.write(_struct_I.pack(length))
      for val1 in self.next_step.contact.hull.points:
        _x = val1
        buff.write(_get_struct_3f().pack(_x.x, _x.y, _x.z))
      _x = self
      buff.write(_get_struct_4dqBq().pack(_x.next_step.contact.offset.x, _x.next_step.contact.offset.y, _x.next_step.contact.offset.z, _x.next_step.contact.friction.data, _x.next_step.body_id.data, _x.next_step.final_step.data, _x.next_step.n_step.data))
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
      if self.phase is None:
        self.phase = walking_core_msgs.msg.WalkingPhase()
      if self.stance_foot is None:
        self.stance_foot = control_core_msgs.msg.BodyId()
      if self.swing_foot is None:
        self.swing_foot = control_core_msgs.msg.BodyId()
      if self.elapsed is None:
        self.elapsed = std_msgs.msg.Float64()
      if self.left_foot_ratio is None:
        self.left_foot_ratio = std_msgs.msg.Float64()
      if self.cur_step is None:
        self.cur_step = walking_core_msgs.msg.FootStep()
      if self.target_step is None:
        self.target_step = walking_core_msgs.msg.FootStep()
      if self.next_step is None:
        self.next_step = walking_core_msgs.msg.FootStep()
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
      end += 75
      (_x.phase.phase, _x.stance_foot.id, _x.swing_foot.id, _x.elapsed.data, _x.left_foot_ratio.data, _x.cur_step.contact.pose.position.x, _x.cur_step.contact.pose.position.y, _x.cur_step.contact.pose.position.z, _x.cur_step.contact.pose.orientation.x, _x.cur_step.contact.pose.orientation.y, _x.cur_step.contact.pose.orientation.z, _x.cur_step.contact.pose.orientation.w,) = _get_struct_3B9d().unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.cur_step.contact.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.cur_step.contact.hull.points.append(val1)
      _x = self
      start = end
      end += 105
      (_x.cur_step.contact.offset.x, _x.cur_step.contact.offset.y, _x.cur_step.contact.offset.z, _x.cur_step.contact.friction.data, _x.cur_step.body_id.data, _x.cur_step.final_step.data, _x.cur_step.n_step.data, _x.target_step.contact.pose.position.x, _x.target_step.contact.pose.position.y, _x.target_step.contact.pose.position.z, _x.target_step.contact.pose.orientation.x, _x.target_step.contact.pose.orientation.y, _x.target_step.contact.pose.orientation.z, _x.target_step.contact.pose.orientation.w,) = _get_struct_4dqBq7d().unpack(str[start:end])
      self.cur_step.final_step.data = bool(self.cur_step.final_step.data)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.target_step.contact.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.target_step.contact.hull.points.append(val1)
      _x = self
      start = end
      end += 105
      (_x.target_step.contact.offset.x, _x.target_step.contact.offset.y, _x.target_step.contact.offset.z, _x.target_step.contact.friction.data, _x.target_step.body_id.data, _x.target_step.final_step.data, _x.target_step.n_step.data, _x.next_step.contact.pose.position.x, _x.next_step.contact.pose.position.y, _x.next_step.contact.pose.position.z, _x.next_step.contact.pose.orientation.x, _x.next_step.contact.pose.orientation.y, _x.next_step.contact.pose.orientation.z, _x.next_step.contact.pose.orientation.w,) = _get_struct_4dqBq7d().unpack(str[start:end])
      self.target_step.final_step.data = bool(self.target_step.final_step.data)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.next_step.contact.hull.points = []
      for i in range(0, length):
        val1 = geometry_msgs.msg.Point32()
        _x = val1
        start = end
        end += 12
        (_x.x, _x.y, _x.z,) = _get_struct_3f().unpack(str[start:end])
        self.next_step.contact.hull.points.append(val1)
      _x = self
      start = end
      end += 49
      (_x.next_step.contact.offset.x, _x.next_step.contact.offset.y, _x.next_step.contact.offset.z, _x.next_step.contact.friction.data, _x.next_step.body_id.data, _x.next_step.final_step.data, _x.next_step.n_step.data,) = _get_struct_4dqBq().unpack(str[start:end])
      self.next_step.final_step.data = bool(self.next_step.final_step.data)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_3B9d = None
def _get_struct_3B9d():
    global _struct_3B9d
    if _struct_3B9d is None:
        _struct_3B9d = struct.Struct("<3B9d")
    return _struct_3B9d
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
_struct_4dqBq = None
def _get_struct_4dqBq():
    global _struct_4dqBq
    if _struct_4dqBq is None:
        _struct_4dqBq = struct.Struct("<4dqBq")
    return _struct_4dqBq
_struct_4dqBq7d = None
def _get_struct_4dqBq7d():
    global _struct_4dqBq7d
    if _struct_4dqBq7d is None:
        _struct_4dqBq7d = struct.Struct("<4dqBq7d")
    return _struct_4dqBq7d
