import pytest
from vision.world_message import wrap_message
from system_interfaces.msg import VisionMessage

def test_invalid_values_arediscarded():
    id = ID(id=0, isball=True)
    obj = Object([[999999.0], [999999.0]], id, confidence=1.0)
    obj.update(999999.0, 999999.0, 1.0)
    objects = {id_: obj}


    msg = wrap_message(objects)
    assert len(msg.balls) == 0