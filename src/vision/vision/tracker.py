from vision.kalman_filter import KalmanFilterClass2D
from system_interfaces.msg import VisionMessage, Robots, Balls, ObjectID
from vision.proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

import numpy as np
from typing import Optional

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

    def update(self, message: SSL_WrapperPacket) -> VisionMessage:
        '''
        Updates the position and velocity of objects based on the detections.
        '''
        # TODO Implement a Hungarian algorithm to give the balls an id?
        detections, objects_id, confidences, orientations, time_stamp = [], [], [], [], message.detection.t_capture
        
        self.dt = time_stamp - self.last_time_stamp

        if message.detection.robots_yellow:
            for yellow_robot in message.detection.robots_yellow:
                objects_id.append(ID(yellow_robot.robot_id, is_ball = False, is_blue = False))
                detections.append([[yellow_robot.x], [yellow_robot.y]])
                confidences.append(yellow_robot.confidence)
                orientations.append(yellow_robot.orientation)
        
        if message.detection.robots_blue:
            for blue_robot in message.detection.robots_blue:
                objects_id.append(ID(blue_robot.robot_id, is_ball = False, is_blue = True))
                detections.append([[blue_robot.x], [blue_robot.y]])
                confidences.append(blue_robot.confidence)
                orientations.append(blue_robot.orientation)
        
        # Balls dont have ids, so will consider the first ball as the main ball and ignore the rest
        # TODO Implement a way to consider the ball with highest confidence to be the main ball.
        if message.detection.balls:
            objects_id.append(ID(id = 0, is_ball = True))
            detections.append([[(message.detection.balls[0]).x], [(message.detection.balls[0]).y]])
            confidences.append(message.detection.balls[0].confidence)
            orientations.append(None)

        self._update(detections, objects_id, confidences, orientations)

        return self.wrap_message()
    # Detections == [x, y]
    def _update(self, detections, objects_id, confidences, orientations) -> None:
        # if empty list, assigning objects to the ObjectTracker.
        if not self.objects:
            for i, detection in enumerate(detections):
                self.objects.append(Object(detection, objects_id[i], confidences[i], orientation = orientations[i]))
                self.objects_id = objects_id
        
        # Check for objects not begin detected, and if skip_count surpass max_frame_skipped, delete it.
        for i in range(len(self.objects)):
            if not self.objects[i].id in self.objects_id:
                
                if self.objects[i].skip_count > self.max_frame_skipped:
                    del self.objects[i]
                    self.object_id.remove(self.objects[i].id)
                
                else:
                    self.objects[i].skip_count += 1
                    self.objects[i].confidence = 0

        # Check for new detections and adding them to objects.
        for i in range(len(detections)):
            if not objects_id[i] in self.objects_id:
                self.objects.append(Object(detections[i], objects_id[i], confidences[i], orientation = orientations[i]))
                self.objects_id.append(objects_id[i])

        # Updating parameters.
        for object_ in self.objects:
            # If the object is detected then update his kalman filter predictions, else update with predition
            if object_.id in objects_id:
                object_.KF.predict(self.dt)
                object_.prediction = object_.KF.update(detections[objects_id.index(object_.id)])
                object_.confidence = confidences[objects_id.index(object_.id)]

                object_.skip_count = 0
                self.last_time_stamp = time_stamp
            else:
                object_.KF.predict(self.dt)

    def wrap_message(self) -> VisionMessage:
        message = VisionMessage()

        for i, object_id in enumerate(self.objects_id):
            if object_id.is_ball:
                ball_msg = Balls()
                ball_id = ObjectID()

                ball_id.id = object_id.id
                ball_id.is_ball = object_id.is_ball
                ball_msg.id = ball_id

                # Kalman filter x attribute is a vector [x position, y position, x velocity, y velocity]
                ball_msg.position_x = float((self.objects[i]).KF.x[0][0])
                ball_msg.position_y = float((self.objects[i]).KF.x[1][0])

                ball_msg.velocity_x = float((self.objects[i]).KF.x[2][0])
                ball_msg.velocity_y = float((self.objects[i]).KF.x[3][0])

                message.balls.append(ball_msg)

            elif object_id.is_blue:
                robot_msg = Robots()
                robot_id = ObjectID()

                robot_id.id = object_id.id
                robot_id.is_ball = object_id.is_ball
                robot_id.is_blue = object_id.is_blue
                robot_msg.id = robot_id

                robot_msg.position_x = float((self.objects[i]).KF.x[0][0])
                robot_msg.position_y = float((self.objects[i]).KF.x[1][0])

                robot_msg.velocity_x = float((self.objects[i]).KF.x[2][0])
                robot_msg.velocity_y = float((self.objects[i]).KF.x[3][0])

                robot_msg.orientation = (self.objects[i]).orientation
                robot_msg.velocity_orientation = 0.0

                message.blue_robots.append(robot_msg)

            else:
                robot_msg = Robots()
                robot_id = ObjectID()

                robot_id.id = object_id.id
                robot_id.is_ball = object_id.is_ball
                robot_id.is_blue = object_id.is_blue
                robot_msg.id = robot_id

                robot_msg.position_x = float((self.objects[i]).KF.x[0][0])
                robot_msg.position_y = float((self.objects[i]).KF.x[1][0])

                robot_msg.velocity_x = float((self.objects[i]).KF.x[2][0])
                robot_msg.velocity_y = float((self.objects[i]).KF.x[3][0])

                robot_msg.orientation = (self.objects[i]).orientation
                robot_msg.velocity_orientation = 0.0

                message.yellow_robots.append(robot_msg)
            
        return message