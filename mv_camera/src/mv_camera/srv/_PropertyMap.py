"""autogenerated by genpy from mv_camera/PropertyMapRequest.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PropertyMapRequest(genpy.Message):
  _md5sum = "b0ad6e60308bc563e681de044965d49d"
  _type = "mv_camera/PropertyMapRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """int8 GET_PROPERTY_LIST=0
int8 SET_PROPERTY=1
int8 GET_PROPERTY_INFO=2
int8 SEARCH_PROPERTY_MAP=3
int8 SAVE_SETTINGS=4
int8 LOAD_SETTINGS=5

int8 START_CAPTURE_PROCESS=6
int8 STOP_CAPTURE_PROCESS=7
int8 CAPTURE_SINGLE_FRAME=8
int8 RESTART_DEVICE=9
int8 CLOSE_DEVICE=10
int8 OPEN_DEVICE=11
int8 LOAD_PROPERTIES=12

string SHOW_FLAGS=flagsOn
string SHOW_VALUES=valuesOn

int8 command
string identifier
string value
float64 testd=1.23456

"""
  # Pseudo-constants
  GET_PROPERTY_LIST = 0
  SET_PROPERTY = 1
  GET_PROPERTY_INFO = 2
  SEARCH_PROPERTY_MAP = 3
  SAVE_SETTINGS = 4
  LOAD_SETTINGS = 5
  START_CAPTURE_PROCESS = 6
  STOP_CAPTURE_PROCESS = 7
  CAPTURE_SINGLE_FRAME = 8
  RESTART_DEVICE = 9
  CLOSE_DEVICE = 10
  OPEN_DEVICE = 11
  LOAD_PROPERTIES = 12
  SHOW_FLAGS = 'flagsOn'
  SHOW_VALUES = 'valuesOn'
  testd = 1.23456

  __slots__ = ['command','identifier','value']
  _slot_types = ['int8','string','string']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       command,identifier,value

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PropertyMapRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.command is None:
        self.command = 0
      if self.identifier is None:
        self.identifier = ''
      if self.value is None:
        self.value = ''
    else:
      self.command = 0
      self.identifier = ''
      self.value = ''

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
      buff.write(_struct_b.pack(self.command))
      _x = self.identifier
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.value
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
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
      end += 1
      (self.command,) = _struct_b.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.identifier = str[start:end].decode('utf-8')
      else:
        self.identifier = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.value = str[start:end].decode('utf-8')
      else:
        self.value = str[start:end]
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
      buff.write(_struct_b.pack(self.command))
      _x = self.identifier
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self.value
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
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
      end += 1
      (self.command,) = _struct_b.unpack(str[start:end])
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.identifier = str[start:end].decode('utf-8')
      else:
        self.identifier = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.value = str[start:end].decode('utf-8')
      else:
        self.value = str[start:end]
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
"""autogenerated by genpy from mv_camera/PropertyMapResponse.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class PropertyMapResponse(genpy.Message):
  _md5sum = "27b333159efc3e544e859f255534a46f"
  _type = "mv_camera/PropertyMapResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string result
int8 status


"""
  __slots__ = ['result','status']
  _slot_types = ['string','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       result,status

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(PropertyMapResponse, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.result is None:
        self.result = ''
      if self.status is None:
        self.status = 0
    else:
      self.result = ''
      self.status = 0

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
      _x = self.result
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_b.pack(self.status))
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
        self.result = str[start:end].decode('utf-8')
      else:
        self.result = str[start:end]
      start = end
      end += 1
      (self.status,) = _struct_b.unpack(str[start:end])
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
      _x = self.result
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      if python3:
        buff.write(struct.pack('<I%sB'%length, length, *_x))
      else:
        buff.write(struct.pack('<I%ss'%length, length, _x))
      buff.write(_struct_b.pack(self.status))
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
        self.result = str[start:end].decode('utf-8')
      else:
        self.result = str[start:end]
      start = end
      end += 1
      (self.status,) = _struct_b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_b = struct.Struct("<b")
class PropertyMap(object):
  _type          = 'mv_camera/PropertyMap'
  _md5sum = 'e57467d958b91c16e19833e5c8c4fa20'
  _request_class  = PropertyMapRequest
  _response_class = PropertyMapResponse
