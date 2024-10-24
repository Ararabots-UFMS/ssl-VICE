# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: ssl_autoref_ci.proto
# Protobuf Python Version: 5.28.0
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    28,
    0,
    '',
    'ssl_autoref_ci.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import ssl_vision_wrapper_tracked_pb2 as ssl__vision__wrapper__tracked__pb2
import ssl_vision_detection_pb2 as ssl__vision__detection__pb2
import ssl_vision_geometry_pb2 as ssl__vision__geometry__pb2
import ssl_gc_referee_message_pb2 as ssl__gc__referee__message__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x14ssl_autoref_ci.proto\x1a ssl_vision_wrapper_tracked.proto\x1a\x1assl_vision_detection.proto\x1a\x19ssl_vision_geometry.proto\x1a\x1cssl_gc_referee_message.proto\"\xb7\x01\n\x0e\x41utoRefCiInput\x12!\n\x0freferee_message\x18\x01 \x01(\x0b\x32\x08.Referee\x12\x35\n\x16tracker_wrapper_packet\x18\x02 \x01(\x0b\x32\x15.TrackerWrapperPacket\x12&\n\tdetection\x18\x03 \x03(\x0b\x32\x13.SSL_DetectionFrame\x12#\n\x08geometry\x18\x04 \x01(\x0b\x32\x11.SSL_GeometryData\"H\n\x0f\x41utoRefCiOutput\x12\x35\n\x16tracker_wrapper_packet\x18\x01 \x01(\x0b\x32\x15.TrackerWrapperPacketBDZBgithub.com/RoboCup-SSL/ssl-game-controller/internal/app/ci/autoref')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'ssl_autoref_ci_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  _globals['DESCRIPTOR']._loaded_options = None
  _globals['DESCRIPTOR']._serialized_options = b'ZBgithub.com/RoboCup-SSL/ssl-game-controller/internal/app/ci/autoref'
  _globals['_AUTOREFCIINPUT']._serialized_start=144
  _globals['_AUTOREFCIINPUT']._serialized_end=327
  _globals['_AUTOREFCIOUTPUT']._serialized_start=329
  _globals['_AUTOREFCIOUTPUT']._serialized_end=401
# @@protoc_insertion_point(module_scope)
