from vision.kalman_filter import KalmanFilterClass2D, KalmanFilterClass1D
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
        self.last_time_stamp = 0
        
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

        # >>> ARARABOTS_AJUSTE   (ligado/desligado por: ararabots.sh ajustes on|off)
        #
        # Efeito medido no desalinhamento do chutador (limite do grSim: 40 mm):
        #     sem o ajuste -> yy = 68, 78, 84, 108, 114, 143 mm
        #     com o ajuste -> yy = 29, 50, 71 mm
        #
        # Fica DESLIGADO por padrao: e codigo fora de src/strategy/ e a decisao
        # de adota-lo e do time. O script liga ao testar e desliga ao sair.
        self.dt = time_stamp - self.last_time_stamp   # ORIGINAL: dt cru, sem validacao
        #AJUSTE# dt = time_stamp - self.last_time_stamp   # intervalo entre DOIS quadros
        # dt == 0 NAO pode cair aqui - MEDIDO, e era o erro.
        #
        # O grSim emite um pacote por CAMERA. Medimos 181 pacotes/s vindos de 4
        # cameras para apenas 46 quadros distintos: 75% dos pacotes tem o MESMO
        # t_capture do anterior, ou seja dt == 0.
        #
        # O vision_node chama tracker.update() uma vez por PACOTE, e cada update
        # termina chamando predict() para TODOS os objetos. Entao predict roda
        # 181x/s enquanto cada objeto e corrigido so 46x/s - quatro predicoes
        # para cada correcao.
        #
        # Com dt == 0 isso e inofensivo: a matriz de transicao vira identidade e
        # o ruido de processo vira zero, entao a predicao e um no-op. Era assim
        # no codigo original.
        #
        # Trocando dt == 0 por 1/60, como esta guarda fazia, cada um desses 75%
        # virava uma predicao CHEIA: o filtro passava a avancar o modelo dele 4x
        # mais rapido que o tempo real, e a posicao estimada corria na frente da
        # realidade. E o mecanismo mais plausivel para a "trajetoria estranha".
        #
        # Entao: dt < 0 (grSim reiniciado), dt > 1 (pausa longa) e o primeiro
        # quadro continuam sendo substituidos. dt == 0 fica 0.
        #AJUSTE# if self.last_time_stamp == 0 or dt < 0 or dt > 1.0:
            #AJUSTE# dt = 1.0 / 60.0   # periodo tipico de um quadro, em vez de um valor que estoura a covariancia
        #AJUSTE# self.dt = dt   # so entao publica, ja validado
        # <<< ARARABOTS_AJUSTE

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