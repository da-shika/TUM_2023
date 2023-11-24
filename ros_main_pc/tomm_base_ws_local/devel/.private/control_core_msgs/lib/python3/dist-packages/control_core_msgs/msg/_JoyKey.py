# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from control_core_msgs/JoyKey.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class JoyKey(genpy.Message):
  _md5sum = "7d7dd2dc15a1d1ee57b8e83eb173156c"
  _type = "control_core_msgs/JoyKey"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """# Keys for the Logitech F710 controller
# Note: The controller mode is set to 'X'

uint8 KEY_A=0
uint8 KEY_B=1
uint8 KEY_X=2
uint8 KEY_Y=3
uint8 KEY_LB=4
uint8 KEY_RB=5
uint8 JOY_LX=0
uint8 JOY_LY=1
uint8 JOY_LT=2
uint8 JOY_RX=3
uint8 JOY_RY=4
uint8 JOY_RT=5
uint8 ARROW_LEFT_RIGHT=6
uint8 ARROW_UP_DOWN=7"""
  # Pseudo-constants
  KEY_A = 0
  KEY_B = 1
  KEY_X = 2
  KEY_Y = 3
  KEY_LB = 4
  KEY_RB = 5
  JOY_LX = 0
  JOY_LY = 1
  JOY_LT = 2
  JOY_RX = 3
  JOY_RY = 4
  JOY_RT = 5
  ARROW_LEFT_RIGHT = 6
  ARROW_UP_DOWN = 7

  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(JoyKey, self).__init__(*args, **kwds)

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
      pass
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
      end = 0
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
      pass
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
      end = 0
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
