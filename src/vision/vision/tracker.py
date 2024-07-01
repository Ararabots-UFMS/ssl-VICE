from vision import messages_robocup_ssl_wrapper_pb2
from kalman_filter import KalmanFilterClass2D
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
    '''
    def __init__(self, max_frame_skipped: int):
        
        self.max_frame_skipped = max_frame_skipped
        self.objects_id = objects_id
        self.objects = []

    def Update(self, detections, objects_id) -> None:
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
                object_.ID.KF.predict()
                object_.prediction = object_.KF.update(detections[objects_id.index(object_.ID)])

                object_.skip_count = 0
            else:
                object_.prediction = object_.KF.update(np.array([[0], [0]]))


def convert(msg: messages_robocup_ssl_wrapper_pb2):
    '''
    Protobuf message conversor to VisionMessage
    
    Args:
        msg: messages_robocup_ssl_wrapper_pb2 = Protobuf input message.

    Output:
        ball_position: float32[6] = X and Y position of the balls, 3 balls maximum.
        
        yellow_team_ids:          uint16[5]   =  IDs for the yellow team, 5 IDs maximum.
        yellow_team_pos:          float32[10] =  X and Y position of the yellow team robots.
        yellow_team_speed:        float32[10] =  X and Y velocity of the yellow team robots.
        yellow_team_orientation:  float32[5]  =  Robots orientation in radians.
        yellow_team_vorientation: float32[5]  =  Robots orientation velocity.
 
        blue_team_ids:            uint16[5]   =  IDs for the yellow team, 5 IDs maximum.
        blue_team_pos:            float32[10] =  X and Y position of the yellow team robots.
        blue_team_speed:          float32[10] =  X and Y velocity of the yellow team robots.
        blue_team_orientation:    float32[5]  =  Robots orientation in radians.
        blue_team_vorientation:   float32[5]  =  Robots orientation velocity.

        vision_fps:               uint16      =  Received frames per seconds
    '''