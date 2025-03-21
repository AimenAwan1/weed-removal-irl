# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: sensor_data/gnss_data.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='sensor_data/gnss_data.proto',
  package='meercat',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x1bsensor_data/gnss_data.proto\x12\x07meercat\"N\n\tGNSS_Data\x12\x14\n\x0ctimestamp_ms\x18\x01 \x01(\x04\x12\x15\n\rlongitude_deg\x18\x02 \x01(\x01\x12\x14\n\x0clatitude_deg\x18\x03 \x01(\x01\x62\x06proto3'
)




_GNSS_DATA = _descriptor.Descriptor(
  name='GNSS_Data',
  full_name='meercat.GNSS_Data',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='timestamp_ms', full_name='meercat.GNSS_Data.timestamp_ms', index=0,
      number=1, type=4, cpp_type=4, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='longitude_deg', full_name='meercat.GNSS_Data.longitude_deg', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='latitude_deg', full_name='meercat.GNSS_Data.latitude_deg', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto3',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=40,
  serialized_end=118,
)

DESCRIPTOR.message_types_by_name['GNSS_Data'] = _GNSS_DATA
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

GNSS_Data = _reflection.GeneratedProtocolMessageType('GNSS_Data', (_message.Message,), {
  'DESCRIPTOR' : _GNSS_DATA,
  '__module__' : 'sensor_data.gnss_data_pb2'
  # @@protoc_insertion_point(class_scope:meercat.GNSS_Data)
  })
_sym_db.RegisterMessage(GNSS_Data)


# @@protoc_insertion_point(module_scope)
