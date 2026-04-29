from .kalman_filter import KalmanFilterClass2D, KalmanFilterClass1D
from system_interfaces.msg import VisionMessage
from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

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
        self.KF = KalmanFilterClass2D()
        self.skip_count = 0
        # Orientation buffer, orientation needs proper processing.
        self.orientation = orientation
        self.orientation_KF = KalmanFilterClass1D()

    def update(self, x: float, y: float, confidence: float, orientation: Optional[float] = None):
        # Atualiza posição
        self.prediction = self.KF.update([[x], [y]])
        # Atualiza orientação se não for bola
        if not self.id.is_ball and orientation is not None:
            self.orientation = self.orientation_KF.update(orientation)
        self.confidence = confidence
        self.skip_count = 0

    def predict(self, dt: float):
        # Predição de posição
        self.KF.predict(dt)
        # Predição de orientação se não for bola
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
    def __init__(self, max_frame_skipped: int):
        
        self.max_frame_skipped = max_frame_skipped
        self.objects = {}
        self.last_time_stamp = 0
        
    def delete_undetected_objects(self, recieved_objects_id: List[ID]) -> None:
        for id, obj in list(self.objects.items()):
            if id not in recieved_objects_id:
                obj.skip_count += 1
                obj.confidence = 0
                if obj.skip_count > self.max_frame_skipped:
                    del self.objects[id]
            else:
                obj.skip_count = 0

    def read_object_from_message(self, object_, is_ball, is_blue=None) -> ID:
        if is_ball:
            id = ID(id=0, is_ball=True)
            orientation = None
        else:
            id = ID(object_.robot_id, is_ball=False, is_blue=is_blue)
            orientation = object_.orientation

        if id in self.objects:
            self.objects[id].prediction = self.objects[id].KF.update([[object_.x], [object_.y]])
            if not id.is_ball:
                self.objects[id].orientation = self.objects[id].orientation_KF.update(orientation)
            self.objects[id].confidence = object_.confidence
            self.objects[id].skip_count = 0
        else:
            self.objects[id] = Object([[object_.x], [object_.y]], id, object_.confidence, orientation)

        return id
    
    def predict(self) -> None:
        for obj in self.objects.values():
            obj.predict(self.dt)
    
    def update(self, message: SSL_WrapperPacket) -> VisionMessage:
        '''
        Updates the position and velocity of objects based on the detections.
        '''
        # TODO Implement a Hungarian algorithm to give the balls an id?
        recieved_objects_id, time_stamp = [], message.detection.t_capture
        
        self.dt = time_stamp - self.last_time_stamp
        self.last_time_stamp = time_stamp

        if message.detection.robots_yellow:
            for yellow_robot in message.detection.robots_yellow:
                robot_id = self.read_object_from_message(yellow_robot, is_ball = False, is_blue = False)
                recieved_objects_id.append(robot_id)
        
        if message.detection.robots_blue:
            for blue_robot in message.detection.robots_blue:
                robot_id = self.read_object_from_message(blue_robot, is_ball = False, is_blue = True)
                recieved_objects_id.append(robot_id)
        
        # Balls don't have ids, so we track only one ball.
        # Currently, we choose the ball with the highest confidence.
        if message.detection.balls:
            best_ball = max(message.detection.balls, key=lambda b: b.confidence)
            ball_id = self.read_object_from_message(best_ball, is_ball=True)
            recieved_objects_id.append(ball_id)
                
        self.delete_undetected_objects(recieved_objects_id)

        self.predict()