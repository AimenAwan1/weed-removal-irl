# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: wombat_interchange/sensor_update_msg.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from sensor_data import gnss_data_pb2 as sensor__data_dot_gnss__data__pb2
from sensor_data import imu_data_pb2 as sensor__data_dot_imu__data__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='wombat_interchange/sensor_update_msg.proto',
  package='meercat',
  syntax='proto3',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n*wombat_interchange/sensor_update_msg.proto\x12\x07meercat\x1a\x1bsensor_data/gnss_data.proto\x1a\x1asensor_data/imu_data.proto\"\xb0\x01\n\x11Sensor_Update_Msg\x12\x14\n\x0csequence_num\x18\x01 \x01(\r\x12\x14\n\x0ctimestamp_ms\x18\x02 \x01(\r\x12*\n\tgnss_data\x18\x03 \x01(\x0b\x32\x12.meercat.GNSS_DataH\x00\x88\x01\x01\x12(\n\x08imu_data\x18\x04 \x01(\x0b\x32\x11.meercat.IMU_DataH\x01\x88\x01\x01\x42\x0c\n\n_gnss_dataB\x0b\n\t_imu_datab\x06proto3'
  ,
  dependencies=[sensor__data_dot_gnss__data__pb2.DESCRIPTOR,sensor__data_dot_imu__data__pb2.DESCRIPTOR,])




_SENSOR_UPDATE_MSG = _descriptor.Descriptor(
  name='Sensor_Update_Msg',
  full_name='meercat.Sensor_Update_Msg',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='sequence_num', full_name='meercat.Sensor_Update_Msg.sequence_num', index=0,
      number=1, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='timestamp_ms', full_name='meercat.Sensor_Update_Msg.timestamp_ms', index=1,
      number=2, type=13, cpp_type=3, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='gnss_data', full_name='meercat.Sensor_Update_Msg.gnss_data', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='imu_data', full_name='meercat.Sensor_Update_Msg.imu_data', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
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
    _descriptor.OneofDescriptor(
      name='_gnss_data', full_name='meercat.Sensor_Update_Msg._gnss_data',
      index=0, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
    _descriptor.OneofDescriptor(
      name='_imu_data', full_name='meercat.Sensor_Update_Msg._imu_data',
      index=1, containing_type=None,
      create_key=_descriptor._internal_create_key,
    fields=[]),
  ],
  serialized_start=113,
  serialized_end=289,
)

_SENSOR_UPDATE_MSG.fields_by_name['gnss_data'].message_type = sensor__data_dot_gnss__data__pb2._GNSS_DATA
_SENSOR_UPDATE_MSG.fields_by_name['imu_data'].message_type = sensor__data_dot_imu__data__pb2._IMU_DATA
_SENSOR_UPDATE_MSG.oneofs_by_name['_gnss_data'].fields.append(
  _SENSOR_UPDATE_MSG.fields_by_name['gnss_data'])
_SENSOR_UPDATE_MSG.fields_by_name['gnss_data'].containing_oneof = _SENSOR_UPDATE_MSG.oneofs_by_name['_gnss_data']
_SENSOR_UPDATE_MSG.oneofs_by_name['_imu_data'].fields.append(
  _SENSOR_UPDATE_MSG.fields_by_name['imu_data'])
_SENSOR_UPDATE_MSG.fields_by_name['imu_data'].containing_oneof = _SENSOR_UPDATE_MSG.oneofs_by_name['_imu_data']
DESCRIPTOR.message_types_by_name['Sensor_Update_Msg'] = _SENSOR_UPDATE_MSG
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Sensor_Update_Msg = _reflection.GeneratedProtocolMessageType('Sensor_Update_Msg', (_message.Message,), {
  'DESCRIPTOR' : _SENSOR_UPDATE_MSG,
  '__module__' : 'wombat_interchange.sensor_update_msg_pb2'
  # @@protoc_insertion_point(class_scope:meercat.Sensor_Update_Msg)
  })
_sym_db.RegisterMessage(Sensor_Update_Msg)


# @@protoc_insertion_point(module_scope)