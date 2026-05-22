from vision.kalman_filter import ExtendedKalmanFilterClass2D, ExtendedKalmanFilterClass1D
from system_interfaces.msg import VisionMessage
from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
import time
import numpy as np
from typing import Optional, List

# TODO refactor variables, classes and messages names, a lot of similar names being used.

class ID():
    def __init__(self, id: int, is_ball: bool, is_blue: Optional[bool] = None):
        self.id = id
        self.is_ball = is_ball
        self.is_blue = is_blue

    # Defining equivalent objects
    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return self.__dict__ == other.__dict__

    # Defining hash to use ID class as dict key in merge_trackers
    def __hash__(self):
        return hash((self.id, self.is_blue))

    def __ne__(self, other):
        return not(self == other)


class Object(object):
    '''
    Tracked object class, mainly robots, but ball also.
    '''

    def __init__(self, detections, Id: ID, confidence: float, orientation: Optional[float] = None):
        self.prediction = np.asarray(detections)
        self.id = Id
        self.confidence = confidence
        self.KF = ExtendedKalmanFilterClass2D()
        self.last_seen = time.time()
        self.orientation = orientation
        self.orientation_KF = ExtendedKalmanFilterClass1D()

    def update(self, x: float, y: float, confidence: float, orientation: Optional[float] = None):
        self.prediction = self.KF.update([[x], [y]])
        if not self.id.is_ball and orientation is not None:
            self.orientation = self.orientation_KF.update(np.matrix([[orientation]]))
        self.confidence = confidence
        self.last_seen = time.time()

    def predict(self):
        dt = time.time() - self.last_seen
        self.KF.predict(dt)
        if not self.id.is_ball:
            self.orientation_KF.predict(dt)

class ObjectTracker(object):
    '''
    Object tracker class. It handles the position and velocity of all the objects being detected.

    Args:
        max_frame_skipped: int   = Max object detection memory, if object is not detected for too long, it deletes it

    Reference: 

    - "https://github.com/NickNair/Multiple-Object-Tracking-using-Kalman-Filter"

    Complementary documentation:

    - "https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/"
    - "https://github.com/mabhisharma/Multi-Object-Tracking-with-Kalman-Filter/blob/master/kalmanFilter.py"
    
    '''
    def __init__(self, max_time_undetected: float):
        self.max_time_undetected = max_time_undetected
        self.objects = {}
        self.last_time_stamp = 0

    def delete_undetected_objects(self, received_objects_id: List[ID]) -> None:
        now = time.time()
        for id, obj in list(self.objects.items()):
            if id not in received_objects_id:
                obj.confidence = 0
                if now - obj.last_seen > self.max_time_undetected:
                    del self.objects[id]

    def read_object_from_message(self, object_, is_ball, is_blue=None) -> ID:
        if is_ball:
            id = ID(id=0, is_ball=True)
            orientation = None
        else:
            id = ID(object_.robot_id, is_ball=False, is_blue=is_blue)
            orientation = object_.orientation

        if id in self.objects:
            self.objects[id].update(object_.x, object_.y, object_.confidence, orientation)
        else:
            self.objects[id] = Object([[object_.x], [object_.y]], id, object_.confidence, orientation)

        return id

    def predict_undetected(self, received_objects_id: List[ID]) -> None:
        for id, obj in self.objects.items():
            if id not in received_objects_id:
                obj.predict()

    def update(self, message: SSL_WrapperPacket) -> None:
        received_objects_id = []
        self.last_time_stamp = message.detection.t_capture

        for yellow_robot in message.detection.robots_yellow:
            robot_id = self.read_object_from_message(yellow_robot, is_ball=False, is_blue=False)
            received_objects_id.append(robot_id)

        for blue_robot in message.detection.robots_blue:
            robot_id = self.read_object_from_message(blue_robot, is_ball=False, is_blue=True)
            received_objects_id.append(robot_id)

        if message.detection.balls:
            best_ball = max(message.detection.balls, key=lambda b: b.confidence)
            ball_id = self.read_object_from_message(best_ball, is_ball=True)
            received_objects_id.append(ball_id)

        self.predict_undetected(received_objects_id)
        self.delete_undetected_objects(received_objects_id)