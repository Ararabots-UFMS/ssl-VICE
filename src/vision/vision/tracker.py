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

    def __init__(self, detections, Id: ID, confidence: float, orientation: Optional[float] = None,  friction: float = 0.01):
        self.prediction = np.asarray(detections)
        self.id = Id
        self.confidence = confidence
        self.friction = friction
        self.KF = ExtendedKalmanFilterClass2D(friction=self.friction)
        self.last_seen = time.time()
        self.last_prediction = self.last_seen
        self.orientation = orientation
        self.orientation_KF = ExtendedKalmanFilterClass1D(friction=self.friction)

        # initialize() seeds the state and widens the covariance; assigning the state
        # alone leaves P = I, which the filter reads as near-certainty.
        self.KF.initialize(detections[0][0], detections[1][0])
        if orientation is not None and not self.id.is_ball:
            self.orientation_KF.initialize(orientation)

    def update(self, x: float, y: float, confidence: float, orientation: Optional[float] = None):
        z = np.matrix([[x], [y]])
        self.prediction = self.KF.update(z)
        if not self.id.is_ball and orientation is not None:
            z_orientation = np.matrix([[orientation]])
            self.orientation = self.orientation_KF.update(z_orientation)[0, 0]
        self.confidence = confidence
        self.last_seen = time.time()
        self.last_prediction = self.last_seen

    def predict(self):
        now = time.time()
        dt = now - self.last_prediction
        if dt <= 0:
            return
        self.KF.predict(dt)
        if not self.id.is_ball:
            self.orientation_KF.predict(dt)
        self.last_prediction = now
            
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
    def __init__(self, max_time_undetected: float, friction: float = 0.01):
        self.max_time_undetected = max_time_undetected
        self.friction = friction
        self.objects = {}
        # Instant the current estimates refer to, in the ssl-vision camera clock. None
        # until the first packet arrives, so "no data yet" is distinguishable from t=0.
        self.last_capture_stamp = None
        # The same instant on the ROS clock. Only this one can be differenced against
        # "now" to age the estimates; capture_stamp is comparable only against itself.
        self.last_wall_stamp = 0.0

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
            self.objects[id] = Object([[object_.x], [object_.y]], id, object_.confidence, orientation=orientation, friction=self.friction)

        return id

    def update(self, message: SSL_WrapperPacket, wall_stamp: float = 0.0) -> None:
        received_objects_id = []

        # Recorded before any filtering so the published estimates carry the instant
        # they describe, not the instant they happened to be published.
        self.last_capture_stamp = message.detection.t_capture
        self.last_wall_stamp = wall_stamp

        for obj in self.objects.values():
            obj.predict()

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

        self.delete_undetected_objects(received_objects_id)