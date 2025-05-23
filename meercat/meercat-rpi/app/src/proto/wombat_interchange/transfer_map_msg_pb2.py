# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: wombat_interchange/transfer_map_msg.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from sensor_data import gnss_data_pb2 as sensor__data_dot_gnss__data__pb2
from sensor_data import imu_data_pb2 as sensor__data_dot_imu__data__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='wombat_interchange/transfer_map_msg.proto',
  package='meercat',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n)wombat_interchange/transfer_map_msg.proto\x12\x07meercat\x1a\x1bsensor_data/gnss_data.proto\x1a\x1asensor_data/imu_data.proto\"t\n\x10Transfer_Map_Msg\x12\x14\n\x0csequence_num\x18\x01 \x01(\r\x12%\n\tgnss_data\x18\x02 \x03(\x0b\x32\x12.meercat.GNSS_Data\x12#\n\x08imu_data\x18\x03 \x03(\x0b\x32\x11.meercat.IMU_Datab\x06proto3'
  ,
  dependencies=[sensor__data_dot_gnss__data__pb2.DESCRIPTOR,sensor__data_dot_imu__data__pb2.DESCRIPTOR,])




_TRANSFER_MAP_MSG = _descriptor.Descriptor(
  name='Transfer_Map_Msg',
  full_name='meercat.Transfer_Map_Msg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='sequence_num', full_name='meercat.Transfer_Map_Msg.sequence_num', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gnss_data', full_name='meercat.Transfer_Map_Msg.gnss_data', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='imu_data', full_name='meercat.Transfer_Map_Msg.imu_data', index=2,
      number=3, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=111,
  serialized_end=227,
)

_TRANSFER_MAP_MSG.fields_by_name['gnss_data'].message_type = sensor__data_dot_gnss__data__pb2._GNSS_DATA
_TRANSFER_MAP_MSG.fields_by_name['imu_data'].message_type = sensor__data_dot_imu__data__pb2._IMU_DATA
DESCRIPTOR.message_types_by_name['Transfer_Map_Msg'] = _TRANSFER_MAP_MSG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Transfer_Map_Msg = _reflection.GeneratedProtocolMessageType('Transfer_Map_Msg', (_message.Message,), {
  'DESCRIPTOR' : _TRANSFER_MAP_MSG,
  '__module__' : 'wombat_interchange.transfer_map_msg_pb2'
  # @@protoc_insertion_point(class_scope:meercat.Transfer_Map_Msg)
  })
_sym_db.RegisterMessage(Transfer_Map_Msg)


# @@protoc_insertion_point(module_scope)
