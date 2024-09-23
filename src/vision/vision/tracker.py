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
        
    def add_object(self, id: ID, x: float, y: float, confidence: float, orientation: Optional[float] = None) -> None:
        self.objects_id.append(id)
        self.objects.append(Object([[x], [y]], id, confidence, orientation = orientation))
        
    def update_objects(self, recieved_objects_id: List[ID], detections: List[List[float]], confidences: List[float], time_stamp: float, orientations: Optional[List[float]] = None) -> None:        
        # Updating parameters.
        for object_ in self.objects:
            # If the object is detected then update his kalman filter predictions, else update with predition
            if object_.id not in recieved_objects_id:
                object_.KF.predict(self.dt)
                if not object_.id.is_ball:
                    object_.orientation_KF.predict(self.dt)
            else:
                # Predict position and velocity.
                object_.KF.predict(self.dt)
                object_.prediction = object_.KF.update(detections[recieved_objects_id.index(object_.id)])

                # Predict orientation if not ball.
                if not object_.id.is_ball:
                    object_.orientation_KF.predict(self.dt)
                    object_.orientantion = object_.orientation_KF.update(orientations[recieved_objects_id.index(object_.id)])

                object_.confidence = confidences[recieved_objects_id.index(object_.id)]

                object_.skip_count = 0
        
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

    def update(self, message: SSL_WrapperPacket) -> VisionMessage:
        '''
        Updates the position and velocity of objects based on the detections.
        '''
        # TODO Implement a Hungarian algorithm to give the balls an id?
        detections, recieved_objects_id, confidences, orientations, time_stamp = [], [], [], [], message.detection.t_capture
        
        self.dt = time_stamp - self.last_time_stamp
        self.last_time_stamp = time_stamp

        if message.detection.robots_yellow:
            for yellow_robot in message.detection.robots_yellow:
                recieved_objects_id.append(ID(yellow_robot.robot_id, is_ball = False, is_blue = False))
                detections.append([[yellow_robot.x], [yellow_robot.y]])
                confidences.append(yellow_robot.confidence)
                orientations.append(yellow_robot.orientation)
                robot_id = ID(yellow_robot.robot_id, is_ball = False, is_blue = False)
                if robot_id not in self.objects_id:
                    self.add_object(robot_id, yellow_robot.x, yellow_robot.y, yellow_robot.confidence, yellow_robot.orientation)
        
        if message.detection.robots_blue:
            for blue_robot in message.detection.robots_blue:
                recieved_objects_id.append(ID(blue_robot.robot_id, is_ball = False, is_blue = True))
                detections.append([[blue_robot.x], [blue_robot.y]])
                confidences.append(blue_robot.confidence)
                orientations.append(blue_robot.orientation)
                robot_id = ID(blue_robot.robot_id, is_ball = False, is_blue = True)
                if robot_id not in self.objects_id:
                    self.add_object(robot_id, blue_robot.x, blue_robot.y, blue_robot.confidence, blue_robot.orientation)
        
        # Balls dont have ids, so will consider the first ball as the main ball and ignore the rest
        # TODO Implement a way to consider the ball with highest confidence to be the main ball.
        if message.detection.balls:
            recieved_objects_id.append(ID(id = 0, is_ball = True))
            detections.append([[(message.detection.balls[0]).x], [(message.detection.balls[0]).y]])
            confidences.append(message.detection.balls[0].confidence)
            orientations.append(None)
            ball_id = ID(id = 0, is_ball = True)
            if ball_id not in self.objects_id:
                ball = message.detection.balls[0]
                self.add_object(ball_id, ball.x, ball.y, ball.confidence)
                
        self.delete_undetected_objects(recieved_objects_id)

        self.update_objects(recieved_objects_id, detections, confidences, time_stamp, orientations)
        
