# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pal_walking_msgs/WalkingStep.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import genpy
import humanoid_nav_msgs.msg
import std_msgs.msg

class WalkingStep(genpy.Message):
  _md5sum = "71c9b6abe3794dbf1d1e8705aadecb82"
  _type = "pal_walking_msgs/WalkingStep"
  _has_header = True #flag to mark the presence of a Header object
  _full_text = """# Data about walking step being executed

Header header

humanoid_nav_msgs/StepTarget step_target

std_msgs/Duration           step_duration

int32  step_list_size

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
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: humanoid_nav_msgs/StepTarget
# Target for a single stepping motion of a humanoid's leg

geometry_msgs/Pose2D pose   # step pose as relative offset to last leg
uint8 leg                   # which leg to use (left/right, see below)

uint8 right=0               # right leg constant
uint8 left=1                # left leg constant

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
================================================================================
MSG: std_msgs/Duration
duration data

"""
  __slots__ = ['header','step_target','step_duration','step_list_size']
  _slot_types = ['std_msgs/Header','humanoid_nav_msgs/StepTarget','std_msgs/Duration','int32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       header,step_target,step_duration,step_list_size

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(WalkingStep, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.step_target is None:
        self.step_target = humanoid_nav_msgs.msg.StepTarget()
      if self.step_duration is None:
        self.step_duration = std_msgs.msg.Duration()
      if self.step_list_size is None:
        self.step_list_size = 0
    else:
      self.header = std_msgs.msg.Header()
      self.step_target = humanoid_nav_msgs.msg.StepTarget()
      self.step_duration = std_msgs.msg.Duration()
      self.step_list_size = 0

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
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3dB3i.pack(_x.step_target.pose.x, _x.step_target.pose.y, _x.step_target.pose.theta, _x.step_target.leg, _x.step_duration.data.secs, _x.step_duration.data.nsecs, _x.step_list_size))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.step_target is None:
        self.step_target = humanoid_nav_msgs.msg.StepTarget()
      if self.step_duration is None:
        self.step_duration = std_msgs.msg.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 37
      (_x.step_target.pose.x, _x.step_target.pose.y, _x.step_target.pose.theta, _x.step_target.leg, _x.step_duration.data.secs, _x.step_duration.data.nsecs, _x.step_list_size,) = _struct_3dB3i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_3I.pack(_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs))
      _x = self.header.frame_id
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_3dB3i.pack(_x.step_target.pose.x, _x.step_target.pose.y, _x.step_target.pose.theta, _x.step_target.leg, _x.step_duration.data.secs, _x.step_duration.data.nsecs, _x.step_list_size))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.header is None:
        self.header = std_msgs.msg.Header()
      if self.step_target is None:
        self.step_target = humanoid_nav_msgs.msg.StepTarget()
      if self.step_duration is None:
        self.step_duration = std_msgs.msg.Duration()
      end = 0
      _x = self
      start = end
      end += 12
      (_x.header.seq, _x.header.stamp.secs, _x.header.stamp.nsecs,) = _struct_3I.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.header.frame_id = str[start:end].decode('utf-8')
      else:
        self.header.frame_id = str[start:end]
      _x = self
      start = end
      end += 37
      (_x.step_target.pose.x, _x.step_target.pose.y, _x.step_target.pose.theta, _x.step_target.leg, _x.step_duration.data.secs, _x.step_duration.data.nsecs, _x.step_list_size,) = _struct_3dB3i.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_3dB3i = struct.Struct("<3dB3i")
_struct_3I = struct.Struct("<3I")