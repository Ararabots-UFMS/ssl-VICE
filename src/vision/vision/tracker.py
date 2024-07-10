from vision import messages_robocup_ssl_wrapper_pb2
from kalman_filter import KalmanFilterClass2D
from system_interfaces.msg import VisionMessage

import numpy as np

class Object(object):
    '''
    Tracked object class, mainly robots, but ball also.
    '''

    def __init__(self, ID: int, detections):
        self. detections = detections
        self.id = ID
        self.KF = KalmanFilterClass2D()
        self.skip_count = 0

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
        self.objects_id = []
        self.objects = []

    def update(message: VisionMessage) -> VisionMessage:
        '''
        Updates the position and velocity of objects based on the detections.

        Note:
            All ids needs to be unique, so all blue robots will a off set of 100 and the main ball will have 200 as id.
        '''
        # Balls dont have ids, so will consider the first ball as the main ball and ignore the rest
        # TODO Implement a Hungarian algorithm to give the balls a id?
        detections, objects_id = [], []

        # Blue id offset to maintain all ids unique.
        blue_offset = 100
        
        if message.yellow_robots:
            for yellow_robot in message.yellow_robots:
                detections.append([yellow_robot.x, yellow_robot.y])
                object_id.append(yellow_robot.id)

        if message.blue_robots:
            for blue_robot in message.blue_robots:
                detections.append([blue_robot.x, blue_robot.y])
                object_id.append(blue_robot.id + blue_offset)

        if message.balls:
            detections.append([(balls[0]).x, (balls[0]).y])
            object_id.append(200)

        _update(detections, objects_id)

        return self.wrap_message()

    def _update(self, detections, objects_id) -> None:
        # if empty list, assing objects to the ObjectTracker.
        if not self.objects:
            for i, detection in enumerate(detections):
                self.objects.append(Object(detection, objects_id[i]))
                self.objects_id = objects_id
        
        # Check for objects not begin detected, and if skip_count surpass max_frame_skipped, delete it.
        for i in range(len(self.objects)):
            if not self.objects[i].ID in self.objects_id:
                
                if self.objects[i].skip_count > self.max_frame_skipped:
                    del self.objects[i]
                    self.object_id.remove(self.objects[i].ID)
                
                else:
                    self.objects[i].skip_count += 1

        # Check for new detections and adding them to objects.
        for i in range(len(detections)):
            if not objects_id[i] in self.objects_id:
                self.objects.append(Object(detections[i], objects_id[i]))
                self.objects_id.append(objects_id[i])

        # Updating parameters.
        for object_ in self.objects:
            # If the object is detected then update his kalman filter predictions, else update with [[0], [0]]
            if object_.ID in objects_id:
                object_.KF.predict()
                object_.prediction = object_.KF.update(detections[objects_id.index(object_.ID)])

                object_.skip_count = 0
            else:
                object_.prediction = object_.KF.update(np.array([[0], [0]]))

    def wrap_message(self) -> VisionMessage:
        # TODO Get objects and transform to vision message again.
        pass