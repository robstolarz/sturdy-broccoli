# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from pal_vision_msgs/HogDetection.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import geometry_msgs.msg
import pal_vision_msgs.msg
import std_msgs.msg

class HogDetection(genpy.Message):
  _md5sum = "33e1731149b6e078eff6e4b55c75f260"
  _type = "pal_vision_msgs/HogDetection"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """## Contains data relative to the detection of a person using the HOG descriptor

pal_vision_msgs/Rectangle  imageBoundingBox         # bounding box of image region in which the person has been detected
geometry_msgs/Vector3      direction                # unitary vector expressing in what direction wrt the robot base frame the person is
float32[]                  hog                      # HOG descriptor of the person region
std_msgs/ColorRGBA         principalEigenVectorRGB  # Eigen vector (remember this is an unitary vector) corresponding to the maximum eigen value of all the RGB values in the person region. 
std_msgs/ColorRGBA[]       rgbClusterCenters        # RGB cluster centers of the person region in the image obtained using k-means. The rgb components are expressed in [0..1]
uint32[]                   rgbDescriptor1           # Descriptor based on binarized RGB gradients between adjacent image blocks (version 1)
uint32[]                   rgbDescriptor2           # Descriptor based on binarized RGB gradients between adjacent image blocks (version 2)




================================================================================
MSG: pal_vision_msgs/Rectangle
## Rectangle defined by a point, its width and height
# corresponds to the openCV struct : CvRect

# coordinates of the top left corner of the box
int64 x
int64 y

# wigth of the box
int64 width

# height of the box
int64 height

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

"""
  __slots__ = ['imageBoundingBox','direction','hog','principalEigenVectorRGB','rgbClusterCenters','rgbDescriptor1','rgbDescriptor2']
  _slot_types = ['pal_vision_msgs/Rectangle','geometry_msgs/Vector3','float32[]','std_msgs/ColorRGBA','std_msgs/ColorRGBA[]','uint32[]','uint32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       imageBoundingBox,direction,hog,principalEigenVectorRGB,rgbClusterCenters,rgbDescriptor1,rgbDescriptor2

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(HogDetection, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.imageBoundingBox is None:
        self.imageBoundingBox = pal_vision_msgs.msg.Rectangle()
      if self.direction is None:
        self.direction = geometry_msgs.msg.Vector3()
      if self.hog is None:
        self.hog = []
      if self.principalEigenVectorRGB is None:
        self.principalEigenVectorRGB = std_msgs.msg.ColorRGBA()
      if self.rgbClusterCenters is None:
        self.rgbClusterCenters = []
      if self.rgbDescriptor1 is None:
        self.rgbDescriptor1 = []
      if self.rgbDescriptor2 is None:
        self.rgbDescriptor2 = []
    else:
      self.imageBoundingBox = pal_vision_msgs.msg.Rectangle()
      self.direction = geometry_msgs.msg.Vector3()
      self.hog = []
      self.principalEigenVectorRGB = std_msgs.msg.ColorRGBA()
      self.rgbClusterCenters = []
      self.rgbDescriptor1 = []
      self.rgbDescriptor2 = []

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
      buff.write(_struct_4q3d.pack(_x.imageBoundingBox.x, _x.imageBoundingBox.y, _x.imageBoundingBox.width, _x.imageBoundingBox.height, _x.direction.x, _x.direction.y, _x.direction.z))
      length = len(self.hog)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.pack(pattern, *self.hog))
      _x = self
      buff.write(_struct_4f.pack(_x.principalEigenVectorRGB.r, _x.principalEigenVectorRGB.g, _x.principalEigenVectorRGB.b, _x.principalEigenVectorRGB.a))
      length = len(self.rgbClusterCenters)
      buff.write(_struct_I.pack(length))
      for val1 in self.rgbClusterCenters:
        _x = val1
        buff.write(_struct_4f.pack(_x.r, _x.g, _x.b, _x.a))
      length = len(self.rgbDescriptor1)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.rgbDescriptor1))
      length = len(self.rgbDescriptor2)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(struct.pack(pattern, *self.rgbDescriptor2))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.imageBoundingBox is None:
        self.imageBoundingBox = pal_vision_msgs.msg.Rectangle()
      if self.direction is None:
        self.direction = geometry_msgs.msg.Vector3()
      if self.principalEigenVectorRGB is None:
        self.principalEigenVectorRGB = std_msgs.msg.ColorRGBA()
      if self.rgbClusterCenters is None:
        self.rgbClusterCenters = None
      end = 0
      _x = self
      start = end
      end += 56
      (_x.imageBoundingBox.x, _x.imageBoundingBox.y, _x.imageBoundingBox.width, _x.imageBoundingBox.height, _x.direction.x, _x.direction.y, _x.direction.z,) = _struct_4q3d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.hog = struct.unpack(pattern, str[start:end])
      _x = self
      start = end
      end += 16
      (_x.principalEigenVectorRGB.r, _x.principalEigenVectorRGB.g, _x.principalEigenVectorRGB.b, _x.principalEigenVectorRGB.a,) = _struct_4f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.rgbClusterCenters = []
      for i in range(0, length):
        val1 = std_msgs.msg.ColorRGBA()
        _x = val1
        start = end
        end += 16
        (_x.r, _x.g, _x.b, _x.a,) = _struct_4f.unpack(str[start:end])
        self.rgbClusterCenters.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.rgbDescriptor1 = struct.unpack(pattern, str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.rgbDescriptor2 = struct.unpack(pattern, str[start:end])
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
      buff.write(_struct_4q3d.pack(_x.imageBoundingBox.x, _x.imageBoundingBox.y, _x.imageBoundingBox.width, _x.imageBoundingBox.height, _x.direction.x, _x.direction.y, _x.direction.z))
      length = len(self.hog)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.hog.tostring())
      _x = self
      buff.write(_struct_4f.pack(_x.principalEigenVectorRGB.r, _x.principalEigenVectorRGB.g, _x.principalEigenVectorRGB.b, _x.principalEigenVectorRGB.a))
      length = len(self.rgbClusterCenters)
      buff.write(_struct_I.pack(length))
      for val1 in self.rgbClusterCenters:
        _x = val1
        buff.write(_struct_4f.pack(_x.r, _x.g, _x.b, _x.a))
      length = len(self.rgbDescriptor1)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.rgbDescriptor1.tostring())
      length = len(self.rgbDescriptor2)
      buff.write(_struct_I.pack(length))
      pattern = '<%sI'%length
      buff.write(self.rgbDescriptor2.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(_x))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(_x))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.imageBoundingBox is None:
        self.imageBoundingBox = pal_vision_msgs.msg.Rectangle()
      if self.direction is None:
        self.direction = geometry_msgs.msg.Vector3()
      if self.principalEigenVectorRGB is None:
        self.principalEigenVectorRGB = std_msgs.msg.ColorRGBA()
      if self.rgbClusterCenters is None:
        self.rgbClusterCenters = None
      end = 0
      _x = self
      start = end
      end += 56
      (_x.imageBoundingBox.x, _x.imageBoundingBox.y, _x.imageBoundingBox.width, _x.imageBoundingBox.height, _x.direction.x, _x.direction.y, _x.direction.z,) = _struct_4q3d.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      end += struct.calcsize(pattern)
      self.hog = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      _x = self
      start = end
      end += 16
      (_x.principalEigenVectorRGB.r, _x.principalEigenVectorRGB.g, _x.principalEigenVectorRGB.b, _x.principalEigenVectorRGB.a,) = _struct_4f.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.rgbClusterCenters = []
      for i in range(0, length):
        val1 = std_msgs.msg.ColorRGBA()
        _x = val1
        start = end
        end += 16
        (_x.r, _x.g, _x.b, _x.a,) = _struct_4f.unpack(str[start:end])
        self.rgbClusterCenters.append(val1)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.rgbDescriptor1 = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sI'%length
      start = end
      end += struct.calcsize(pattern)
      self.rgbDescriptor2 = numpy.frombuffer(str[start:end], dtype=numpy.uint32, count=length)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_4f = struct.Struct("<4f")
_struct_4q3d = struct.Struct("<4q3d")
