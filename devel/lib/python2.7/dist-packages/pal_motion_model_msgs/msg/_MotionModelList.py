# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pal_motion_model_msgs/MotionModelList.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import pal_motion_model_msgs.msg

class MotionModelList(genpy.Message):
  _md5sum = "f24b8d6d6a0a1542de28f172c2da67ff"
  _type = "pal_motion_model_msgs/MotionModelList"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """#list of motion models learnt in a specific place

MotionModel[] models




================================================================================
MSG: pal_motion_model_msgs/MotionModel
## Contains the 2D motion model of a robot at a specific location

#Heading direction is represented through a gaussian pdf.
float32  heading_mean
float32  heading_std_dev

#For statistics we store a pdf over the robot speeds
float32  linear_speed_mean
float32  linear_speed_std_dev
float32  angular_speed_mean
float32  angular_speed_std_dev


"""
  __slots__ = ['models']
  _slot_types = ['pal_motion_model_msgs/MotionModel[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       models

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(MotionModelList, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.models is None:
        self.models = []
    else:
      self.models = []

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
      length = len(self.models)
      buff.write(_struct_I.pack(length))
      for val1 in self.models:
        _x = val1
        buff.write(_struct_6f.pack(_x.heading_mean, _x.heading_std_dev, _x.linear_speed_mean, _x.linear_speed_std_dev, _x.angular_speed_mean, _x.angular_speed_std_dev))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.models is None:
        self.models = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.models = []
      for i in range(0, length):
        val1 = pal_motion_model_msgs.msg.MotionModel()
        _x = val1
        start = end
        end += 24
        (_x.heading_mean, _x.heading_std_dev, _x.linear_speed_mean, _x.linear_speed_std_dev, _x.angular_speed_mean, _x.angular_speed_std_dev,) = _struct_6f.unpack(str[start:end])
        self.models.append(val1)
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
      length = len(self.models)
      buff.write(_struct_I.pack(length))
      for val1 in self.models:
        _x = val1
        buff.write(_struct_6f.pack(_x.heading_mean, _x.heading_std_dev, _x.linear_speed_mean, _x.linear_speed_std_dev, _x.angular_speed_mean, _x.angular_speed_std_dev))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.models is None:
        self.models = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.models = []
      for i in range(0, length):
        val1 = pal_motion_model_msgs.msg.MotionModel()
        _x = val1
        start = end
        end += 24
        (_x.heading_mean, _x.heading_std_dev, _x.linear_speed_mean, _x.linear_speed_std_dev, _x.angular_speed_mean, _x.angular_speed_std_dev,) = _struct_6f.unpack(str[start:end])
        self.models.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_6f = struct.Struct("<6f")
