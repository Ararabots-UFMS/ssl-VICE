from vision.kalman_filter import KalmanFilterClass2D, KalmanFilterClass1D
from system_interfaces.msg import VisionMessage
from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

import numpy as np
from typing import Optional, List
from vision.objects import BallID, RobotID, BallObject, RobotObject

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
    def __init__(self, cam_id: int, max_frame_skipped: int):
        
        self.max_frame_skipped = max_frame_skipped
        self.objects_id = []
        self.objects = []
        self.last_time_stamp = 0
        self.cam_id = cam_id
        
    def update_object(self, object_: Object, x: float, y: float, confidence: float, orientation: Optional[float] = None) -> None:
        # Predict position and velocity.
        object_.prediction = object_.KF.update([[x], [y]])

        # Predict orientation if not ball.
        if not object_.id.is_ball:
            object_.orientantion = object_.orientation_KF.update(orientation)

        object_.confidence = confidence

        object_.skip_count = 0
        
    def add_object(self, id: ID, x: float, y: float, confidence: float, orientation: Optional[float] = None) -> None:
        self.objects_id.append(id)
        self.objects.append(Object([[x], [y]], id, confidence, orientation = orientation))
    
    def delete_undetected_objects(self, recieved_objects_id: List[ID]) -> None:
         for i in range(len(self.objects_id)):
            if self.objects_id[i] not in recieved_objects_id:
                if self.objects[i].skip_count > self.max_frame_skipped:
                    self.objects_id.remove(self.objects[i].id)
                    del self.objects[i]
                else:
                    self.objects[i].skip_count += 1
                    self.objects[i].confidence = 0
            else:
                self.objects[i].skip_count = 0
        
    def predict(self) -> None:
        for object_ in self.objects:
            object_.KF.predict(self.dt)
            if not object_.id.is_ball:
                object_.orientation_KF.predict(self.dt)
                
    def read_object_from_message(self, object_, is_ball, is_blue = None) -> ID:
        if is_ball:
            id = ID(id = 0, is_ball = is_ball)
            orientation = None
        else:
            id = ID(object_.robot_id, is_ball = is_ball, is_blue = is_blue)
            orientation = object_.orientation
            
        try:
            index = self.objects_id.index(id)
            self.update_object(self.objects[index], object_.x, object_.y, object_.confidence, orientation)
        except ValueError:
            self.add_object(id, object_.x, object_.y, object_.confidence, orientation)
        
        return id

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
        
        # Balls dont have ids, so will consider the first ball as the main ball and ignore the rest
        # TODO Implement a way to consider the ball with highest confidence to be the main ball.
        if message.detection.balls:
            ball_id = self.read_object_from_message(message.detection.balls[0], is_ball = True)
            recieved_objects_id.append(ball_id)
                
        self.delete_undetected_objects(recieved_objects_id)

        self.predict()