import numpy as np

from typing import Optional

class KalmanFilterClass2D(object):
    '''
    Kalman filter for 2D (position and velocity).
    
    All calculation and code are based on the reference.

    Reference: 
    - "https://github.com/NickNair/Multiple-Object-Tracking-using-Kalman-Filter"
    - "https://github.com/LdDl/kalman-filter"

    Complementary documentation:

    - "https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/"
    - "https://github.com/mabhisharma/Multi-Object-Tracking-with-Kalman-Filter/blob/master/kalmanFilter.py"
    - "https://cookierobotics.com/071/"
    '''
    # >>> ARARABOTS_AJUSTE   (ligado/desligado por: ararabots.sh ajustes on|off filtro)
    #
    # As unidades do estado sao MILIMETROS. Os padroes dizem que a camera acerta
    # a 0,1 mm e que o objeto acelera no maximo 1 mm/s2 - mas so a desaceleracao
    # da bola, medida no grSim, vai de 1330 a 8900 mm/s2. Com o ruido de processo
    # ~4 ordens de grandeza baixo demais, o filtro confia no proprio modelo de
    # velocidade constante e passa a ignorar a camera.
    #
    # Medido no voo da bola (verdade: sai a 6400 mm/s e para 1,44 m adiante):
    #                    atraso maximo   velocidade de pico   ultrapassa   assenta
    #   0,1 / 1 (atual)      644 mm         2883 mm/s          431 mm      1,93 s
    #   3,0 / 5000           108 mm         6242 mm/s            7 mm      1,03 s
    #
    # Confirmado em execucao real: com o grSim reportando a bola a 6196 mm/s, o
    # /visionTopic publicava 1607 mm/s - 26% da verdade. Numa execucao a bola
    # ficou parada no topico por 1,2 s DEPOIS do disparo e entao saltou 1546 mm.
    #
    # ATENCAO: o defeito e real e esta medido, mas LIGAR ISTO NAO MELHOROU O
    # RESULTADO - 6 execucoes contra 6 deram 0 gols e 0 disparos com, contra
    # 1 e 1 sem. Fica desligado ate que alguem meça um ganho.
    #AJUSTE# def __init__(self, x_sd: float = 3.0, y_sd: float = 3.0, u_x: float = 0.1, u_y: float = 0.1, sd_acceleration: float = 5000):
    def __init__(self, x_sd: float = 0.1, y_sd: float = 0.1, u_x: float = 0.1, u_y: float = 0.1, sd_acceleration: float = 1):
    # <<< ARARABOTS_AJUSTE
        self.sd_acceleration = sd_acceleration

        self.u = np.matrix([[u_x],[u_y]])

        #  State vector ; it's [ x position ;  y position ; x velocity ; y velocity ; ]
        self.x = np.matrix([[0], [0], [0], [0]])

        # The matrix that maps state vector to measurement 
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        # Measurement Covariance
        self.R = np.matrix([[x_sd**2,0],
                           [0, y_sd**2]])

        # The error covariance matrix that is Identity for now. It gets updated based on Q, A and R.
        self.P = np.eye(self.x.shape[0])
        
    def predict(self, dt):
        self.B = np.matrix([[(dt**2)/2, 0],
                            [0, (dt**2)/2],
                            [dt,0],
                            [0,dt]])

        # The state transition matrix 
        self.A = np.matrix([[1, 0, dt, 0],
                            [0, 1, 0, dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])

        # Processs Covariance that for our case depends solely on the acceleration
        self.Q = np.matrix([[(dt**4)/4, 0, (dt**3)/2, 0],
                            [0, (dt**4)/4, 0, (dt**3)/2],
                            [(dt**3)/2, 0, dt**2, 0],
                            [0, (dt**3)/2, 0, dt**2]]) * self.sd_acceleration ** 2

        # If we add the B.u it doesnt work... maybe its a dt problem
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        
        # Updation of the error covariance matrix 
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        return self.x

    def update(self, z: np.matrix):
        # z is the measurement taken, it should be a matrix [[x], [y]] measured.
        # Two sources use different operators between HPH^T and R. One uses + and other -
        # Innovation covariance matrix
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S)) 

        # Update State vector
        self.x = self.x + np.dot(K, (z - np.dot(self.H, self.x)))

        I = np.eye(self.H.shape[1])

        self.P = (I -(K*self.H))*self.P  
        
        return self.x
    
    def set_param(self, x_sd: Optional[float] = None,
                        y_sd: Optional[float] = None,
                        u_x:  Optional[float] = None,
                        u_y:  Optional[float] = None,
                        acceleration_sd_2d: Optional[float] = None):

        self.R[0] = [x_sd**2, 0] if x_sd else self.R[0]
        self.R[1] = [0, y_sd**2] if y_sd else self.R[1]
        
        self.u[0] = [u_x] if u_x else self.u[0]
        self.u[1] = [u_y] if u_y else self.u[1]

        self.sd_acceleration = acceleration_sd_2d if acceleration_sd_2d else self.sd_acceleration

import numpy as np
from typing import Optional

class KalmanFilterClass1D(object):
    '''
    Kalman filter for 1D (Angle and Angular velocity).

    Serves for orientation purposes, employing the information in path planning.
    This version is modified to handle the circular nature of angles in the [-pi, pi] range
    and uses a numerically stable covariance update to prevent divergence.
    '''
    # >>> ARARABOTS_AJUSTE   (ligado/desligado por: ararabots.sh ajustes on|off filtro)
    #
    # Mesmo defeito de unidade do filtro 2D, e aqui doi mais: a direcao do corpo
    # e o que decide se o chute dispara. O grSim exige a bola a menos de 40 mm do
    # eixo do corpo (robot.cpp:128) e a nossa trava e 25 mm.
    #
    # Medido, robo girando a 3 rad/s por 0,5 s (atraso convertido em mm de
    # desalinhamento lateral a 100 mm da placa):
    #                    tremor parado   girando        1,5 s DEPOIS de parar
    #   0,1 / 1 (atual)     0,04 graus   23 graus/42 mm    17 graus/31 mm
    #   0,02 / 200          0,03 graus    4,7 graus/8 mm    2,2 graus/4 mm
    #
    # Repare que 31 mm de erro PERSISTEM 1,5 s depois de o robo parar de girar -
    # em cima da nossa trava de 25 mm.
    #
    # Mesma ressalva do filtro 2D: medido, mas sem ganho de resultado ate agora.
    #AJUSTE# def __init__(self, a_sd: float = 0.02, u: float = 0.0, sd_acceleration: float = 200.0):
    def __init__(self, a_sd: float = 0.1, u: float = 0.0, sd_acceleration: float = 1.0):
    # <<< ARARABOTS_AJUSTE
        self.sd_acceleration = sd_acceleration
        self.u = u
        # State vector: [angle; angular velocity]
        self.x = np.matrix([[0], [0]])
        # Measurement mapping matrix
        self.H = np.matrix([[1, 0]])
        # Measurement Covariance
        self.R = np.matrix([[a_sd ** 2]])
        # State Covariance matrix
        self.P = np.eye(self.x.shape[0])
        
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wraps an angle to the [-pi, pi] range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def predict(self, dt: float):
        # Control input matrix
        self.B = np.matrix([[(dt**2)/2], [dt]]) 
        # State transition matrix
        self.A = np.matrix([[1, dt], [0, 1]])
        # Process Covariance matrix
        self.Q = np.matrix([[(dt**4)/4, (dt**3)/2],
                            [(dt**3)/2, dt**2]]) * self.sd_acceleration**2

        # Predict the next state
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        
        # Wrap the predicted angle to the [-pi, pi] range
        self.x[0, 0] = self._wrap_angle(self.x[0, 0])

        # Update the state covariance matrix
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        return self.x

    def update(self, z: np.matrix):
        # z is the measurement taken, should be a matrix [[angle]]
        
        # Calculate the residual (innovation) and wrap it
        predicted_measurement = np.dot(self.H, self.x)
        residual = z - predicted_measurement
        residual[0, 0] = self._wrap_angle(residual[0, 0])

        # Innovation covariance
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        # Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        # Update the state vector using the wrapped residual
        self.x = self.x + np.dot(K, residual)
        
        # Wrap the updated angle state to keep it in range
        self.x[0, 0] = self._wrap_angle(self.x[0, 0])

        # --- MODIFICATION START: STABLE COVARIANCE UPDATE ---
        # Update the state covariance using the numerically stable Joseph form
        I = np.eye(self.H.shape[1])
        I_KH = I - np.dot(K, self.H)
        self.P = np.dot(I_KH, self.P).dot(I_KH.T) + np.dot(K, self.R).dot(K.T)
        # --- MODIFICATION END ---
        
        return self.x

    def set_param(self, a_sd: Optional[float] = None,
                        u_a:  Optional[float] = None,
                        acceleration_sd_1d: Optional[float] = None):
        if a_sd is not None:
            self.R = np.matrix([[a_sd ** 2]])
        if u_a is not None:
            self.u = u_a
        if acceleration_sd_1d is not None:
            self.sd_acceleration = acceleration_sd_1d