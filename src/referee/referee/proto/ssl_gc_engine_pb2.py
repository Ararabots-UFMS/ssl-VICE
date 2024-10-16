# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: ssl_gc_engine.proto
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
    'ssl_gc_engine.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import ssl_gc_geometry_pb2 as ssl__gc__geometry__pb2
import ssl_gc_common_pb2 as ssl__gc__common__pb2
import timestamp_pb2 as timestamp__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13ssl_gc_engine.proto\x1a\x15ssl_gc_geometry.proto\x1a\x13ssl_gc_common.proto\x1a\x0ftimestamp.proto\"\x9d\x03\n\x07GcState\x12+\n\nteam_state\x18\x01 \x03(\x0b\x32\x17.GcState.TeamStateEntry\x12\x32\n\x0e\x61uto_ref_state\x18\x02 \x03(\x0b\x32\x1a.GcState.AutoRefStateEntry\x12(\n\x08trackers\x18\x03 \x03(\x0b\x32\x16.GcState.TrackersEntry\x12)\n\x10\x63ontinue_actions\x18\x04 \x03(\x0b\x32\x0f.ContinueAction\x12%\n\x0e\x63ontinue_hints\x18\x05 \x03(\x0b\x32\r.ContinueHint\x1a>\n\x0eTeamStateEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\x1b\n\x05value\x18\x02 \x01(\x0b\x32\x0c.GcStateTeam:\x02\x38\x01\x1a\x44\n\x11\x41utoRefStateEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\x1e\n\x05value\x18\x02 \x01(\x0b\x32\x0f.GcStateAutoRef:\x02\x38\x01\x1a/\n\rTrackersEntry\x12\x0b\n\x03key\x18\x01 \x01(\t\x12\r\n\x05value\x18\x02 \x01(\t:\x02\x38\x01\"\xbb\x01\n\x0bGcStateTeam\x12\x11\n\tconnected\x18\x01 \x01(\x08\x12\x1b\n\x13\x63onnection_verified\x18\x02 \x01(\x08\x12 \n\x18remote_control_connected\x18\x03 \x01(\x08\x12*\n\"remote_control_connection_verified\x18\x04 \x01(\x08\x12.\n\x10\x61\x64vantage_choice\x18\x05 \x01(\x0b\x32\x14.TeamAdvantageChoice\"v\n\x13TeamAdvantageChoice\x12\x34\n\x06\x63hoice\x18\x01 \x01(\x0e\x32$.TeamAdvantageChoice.AdvantageChoice\")\n\x0f\x41\x64vantageChoice\x12\x08\n\x04STOP\x10\x00\x12\x0c\n\x08\x43ONTINUE\x10\x01\"-\n\x0eGcStateAutoRef\x12\x1b\n\x13\x63onnection_verified\x18\x01 \x01(\x08\"`\n\x0eGcStateTracker\x12\x13\n\x0bsource_name\x18\x01 \x01(\t\x12\x0c\n\x04uuid\x18\x04 \x01(\t\x12\x13\n\x04\x62\x61ll\x18\x02 \x01(\x0b\x32\x05.Ball\x12\x16\n\x06robots\x18\x03 \x03(\x0b\x32\x06.Robot\"4\n\x04\x42\x61ll\x12\x15\n\x03pos\x18\x01 \x01(\x0b\x32\x08.Vector3\x12\x15\n\x03vel\x18\x02 \x01(\x0b\x32\x08.Vector3\"4\n\x05Robot\x12\x14\n\x02id\x18\x01 \x01(\x0b\x32\x08.RobotId\x12\x15\n\x03pos\x18\x02 \x01(\x0b\x32\x08.Vector2\"\xc4\x05\n\x0e\x43ontinueAction\x12\"\n\x04type\x18\x01 \x02(\x0e\x32\x14.ContinueAction.Type\x12\x17\n\x08\x66or_team\x18\x02 \x02(\x0e\x32\x05.Team\x12\x1b\n\x13\x63ontinuation_issues\x18\x03 \x03(\t\x12,\n\x08ready_at\x18\x04 \x01(\x0b\x32\x1a.google.protobuf.Timestamp\x12$\n\x05state\x18\x05 \x01(\x0e\x32\x15.ContinueAction.State\"\x9d\x03\n\x04Type\x12\x10\n\x0cTYPE_UNKNOWN\x10\x00\x12\x08\n\x04HALT\x10\x01\x12\x14\n\x10RESUME_FROM_HALT\x10\n\x12\r\n\tSTOP_GAME\x10\x02\x12\x0f\n\x0b\x46ORCE_START\x10\x0b\x12\r\n\tFREE_KICK\x10\x11\x12\x10\n\x0cNEXT_COMMAND\x10\x03\x12\x18\n\x14\x42\x41LL_PLACEMENT_START\x10\x04\x12\x19\n\x15\x42\x41LL_PLACEMENT_CANCEL\x10\t\x12\x1b\n\x17\x42\x41LL_PLACEMENT_COMPLETE\x10\x0e\x12\x17\n\x13\x42\x41LL_PLACEMENT_FAIL\x10\x0f\x12\x11\n\rTIMEOUT_START\x10\x05\x12\x10\n\x0cTIMEOUT_STOP\x10\x06\x12\x14\n\x10\x42OT_SUBSTITUTION\x10\x07\x12\x0e\n\nNEXT_STAGE\x10\x08\x12\x0c\n\x08\x45ND_GAME\x10\x10\x12\x0f\n\x0b\x41\x43\x43\x45PT_GOAL\x10\x0c\x12\x0f\n\x0bREJECT_GOAL\x10\x14\x12\x10\n\x0cNORMAL_START\x10\r\x12\x14\n\x10\x43HALLENGE_ACCEPT\x10\x12\x12\x14\n\x10\x43HALLENGE_REJECT\x10\x13\"d\n\x05State\x12\x11\n\rSTATE_UNKNOWN\x10\x00\x12\x0b\n\x07\x42LOCKED\x10\x01\x12\x0b\n\x07WAITING\x10\x02\x12\x0e\n\nREADY_AUTO\x10\x03\x12\x10\n\x0cREADY_MANUAL\x10\x04\x12\x0c\n\x08\x44ISABLED\x10\x05\"\x1f\n\x0c\x43ontinueHint\x12\x0f\n\x07message\x18\x01 \x02(\tB@Z>github.com/RoboCup-SSL/ssl-game-controller/internal/app/engine')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'ssl_gc_engine_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  _globals['DESCRIPTOR']._loaded_options = None
  _globals['DESCRIPTOR']._serialized_options = b'Z>github.com/RoboCup-SSL/ssl-game-controller/internal/app/engine'
  _globals['_GCSTATE_TEAMSTATEENTRY']._loaded_options = None
  _globals['_GCSTATE_TEAMSTATEENTRY']._serialized_options = b'8\001'
  _globals['_GCSTATE_AUTOREFSTATEENTRY']._loaded_options = None
  _globals['_GCSTATE_AUTOREFSTATEENTRY']._serialized_options = b'8\001'
  _globals['_GCSTATE_TRACKERSENTRY']._loaded_options = None
  _globals['_GCSTATE_TRACKERSENTRY']._serialized_options = b'8\001'
  _globals['_GCSTATE']._serialized_start=85
  _globals['_GCSTATE']._serialized_end=498
  _globals['_GCSTATE_TEAMSTATEENTRY']._serialized_start=317
  _globals['_GCSTATE_TEAMSTATEENTRY']._serialized_end=379
  _globals['_GCSTATE_AUTOREFSTATEENTRY']._serialized_start=381
  _globals['_GCSTATE_AUTOREFSTATEENTRY']._serialized_end=449
  _globals['_GCSTATE_TRACKERSENTRY']._serialized_start=451
  _globals['_GCSTATE_TRACKERSENTRY']._serialized_end=498
  _globals['_GCSTATETEAM']._serialized_start=501
  _globals['_GCSTATETEAM']._serialized_end=688
  _globals['_TEAMADVANTAGECHOICE']._serialized_start=690
  _globals['_TEAMADVANTAGECHOICE']._serialized_end=808
  _globals['_TEAMADVANTAGECHOICE_ADVANTAGECHOICE']._serialized_start=767
  _globals['_TEAMADVANTAGECHOICE_ADVANTAGECHOICE']._serialized_end=808
  _globals['_GCSTATEAUTOREF']._serialized_start=810
  _globals['_GCSTATEAUTOREF']._serialized_end=855
  _globals['_GCSTATETRACKER']._serialized_start=857
  _globals['_GCSTATETRACKER']._serialized_end=953
  _globals['_BALL']._serialized_start=955
  _globals['_BALL']._serialized_end=1007
  _globals['_ROBOT']._serialized_start=1009
  _globals['_ROBOT']._serialized_end=1061
  _globals['_CONTINUEACTION']._serialized_start=1064
  _globals['_CONTINUEACTION']._serialized_end=1772
  _globals['_CONTINUEACTION_TYPE']._serialized_start=1257
  _globals['_CONTINUEACTION_TYPE']._serialized_end=1670
  _globals['_CONTINUEACTION_STATE']._serialized_start=1672
  _globals['_CONTINUEACTION_STATE']._serialized_end=1772
  _globals['_CONTINUEHINT']._serialized_start=1774
  _globals['_CONTINUEHINT']._serialized_end=1805
# @@protoc_insertion_point(module_scope)
