from vision.tracker import ObjectTracker
from vision.objects import Object, ID
from vision.world_message import wrap_message

from system_interfaces.msg import VisionMessage

from typing import List, Dict, Tuple, Union

def unify_object(objects: Dict[ID, Object]) -> List[Object]:
    '''
    Retuns the object with highest confidence.
    '''
    uni_objects = []
    for id in objects.keys():
        uni_objects.append(max(objects[id], key=lambda obj: obj.confidence))

    return uni_objects

def merge_trackers(trackers: List[ObjectTracker]) -> VisionMessage:
    '''
    Function to merge all cameras data into one world state message by combining a list of trackers.
    '''
    objects_dict: Dict[ID, List[Object]] = {}

    # TODO Maybe find a way to not use 2 nested loops to do this...
    for tracker in trackers:
        for obj in tracker.objects:
            try:
                # If object is not in the dictionary, add it.
                objects_dict[obj.id].append(obj)
            except KeyError:
                # If object is in the dictionary, append to it.
                objects_dict[obj.id] = [obj]

    return wrap_message(unify_object(objects_dict))
