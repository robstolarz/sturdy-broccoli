# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pal_interaction_msgs/AudioDeviceDescription.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class AudioDeviceDescription(genpy.Message):
  _md5sum = "e55171ed98aeb49e34453bc5efb47184"
  _type = "pal_interaction_msgs/AudioDeviceDescription"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Description of an audio device.

int8 SIGNED = 0
int8 UNSIGNED = 1


# name of the device (usually a portaudio name)
string device_name

# sample rate of the acquired audio signal
uint32 sample_rate

# sample size in bits
int8 sample_size

# format can be SIGNED or UNSIGNED
int8 format

# buffer size used to acquire data.
uint16 buffer_size

# number of recorded channels
int8 number_of_channels
"""
  # Pseudo-constants
  SIGNED = 0
  UNSIGNED = 1

  __slots__ = ['device_name','sample_rate','sample_size','format','buffer_size','number_of_channels']
  _slot_types = ['string','uint32','int8','int8','uint16','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       device_name,sample_rate,sample_size,format,buffer_size,number_of_channels

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(AudioDeviceDescription, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.device_name is None:
        self.device_name = ''
      if self.sample_rate is None:
        self.sample_rate = 0
      if self.sample_size is None:
        self.sample_size = 0
      if self.format is None:
        self.format = 0
      if self.buffer_size is None:
        self.buffer_size = 0
      if self.number_of_channels is None:
        self.number_of_channels = 0
    else:
      self.device_name = ''
      self.sample_rate = 0
      self.sample_size = 0
      self.format = 0
      self.buffer_size = 0
      self.number_of_channels = 0

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
      _x = self.device_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_I2bHb.pack(_x.sample_rate, _x.sample_size, _x.format, _x.buffer_size, _x.number_of_channels))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.device_name = str[start:end].decode('utf-8')
      else:
        self.device_name = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.sample_rate, _x.sample_size, _x.format, _x.buffer_size, _x.number_of_channels,) = _struct_I2bHb.unpack(str[start:end])
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
      _x = self.device_name
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_I2bHb.pack(_x.sample_rate, _x.sample_size, _x.format, _x.buffer_size, _x.number_of_channels))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.device_name = str[start:end].decode('utf-8')
      else:
        self.device_name = str[start:end]
      _x = self
      start = end
      end += 9
      (_x.sample_rate, _x.sample_size, _x.format, _x.buffer_size, _x.number_of_channels,) = _struct_I2bHb.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_I2bHb = struct.Struct("<I2bHb")
