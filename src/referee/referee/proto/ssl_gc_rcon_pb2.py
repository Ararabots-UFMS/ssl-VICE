# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: ssl_gc_rcon.proto
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
    'ssl_gc_rcon.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x11ssl_gc_rcon.proto\"\xa1\x02\n\x0f\x43ontrollerReply\x12\x30\n\x0bstatus_code\x18\x01 \x01(\x0e\x32\x1b.ControllerReply.StatusCode\x12\x0e\n\x06reason\x18\x02 \x01(\t\x12\x12\n\nnext_token\x18\x03 \x01(\t\x12\x33\n\x0cverification\x18\x04 \x01(\x0e\x32\x1d.ControllerReply.Verification\";\n\nStatusCode\x12\x17\n\x13UNKNOWN_STATUS_CODE\x10\x00\x12\x06\n\x02OK\x10\x01\x12\x0c\n\x08REJECTED\x10\x02\"F\n\x0cVerification\x12\x18\n\x14UNKNOWN_VERIFICATION\x10\x00\x12\x0c\n\x08VERIFIED\x10\x01\x12\x0e\n\nUNVERIFIED\x10\x02\",\n\tSignature\x12\r\n\x05token\x18\x01 \x02(\t\x12\x10\n\x08pkcs1v15\x18\x02 \x02(\x0c\x42>Z<github.com/RoboCup-SSL/ssl-game-controller/internal/app/rcon')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'ssl_gc_rcon_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  _globals['DESCRIPTOR']._loaded_options = None
  _globals['DESCRIPTOR']._serialized_options = b'Z<github.com/RoboCup-SSL/ssl-game-controller/internal/app/rcon'
  _globals['_CONTROLLERREPLY']._serialized_start=22
  _globals['_CONTROLLERREPLY']._serialized_end=311
  _globals['_CONTROLLERREPLY_STATUSCODE']._serialized_start=180
  _globals['_CONTROLLERREPLY_STATUSCODE']._serialized_end=239
  _globals['_CONTROLLERREPLY_VERIFICATION']._serialized_start=241
  _globals['_CONTROLLERREPLY_VERIFICATION']._serialized_end=311
  _globals['_SIGNATURE']._serialized_start=313
  _globals['_SIGNATURE']._serialized_end=357
# @@protoc_insertion_point(module_scope)
