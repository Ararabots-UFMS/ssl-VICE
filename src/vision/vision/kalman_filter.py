import numpy as np

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
    def __init__(self, x_sd: float = 0.01, y_sd: float = 0.01, u_x: float = 0.1, u_y: float = 0.1, sd_acceleration: float = 1):
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

class KalmanFilterClass1D(object):
    '''
    Kalman filter for 2D (Angle and Angular velocity).
    
    All calculation and code are based on the reference.

    Reference: 
    - "https://github.com/NickNair/Multiple-Object-Tracking-using-Kalman-Filter"
    - "https://github.com/LdDl/kalman-filter"

    Complementary documentation:

    - "https://machinelearningspace.com/2d-object-tracking-using-kalman-filter/"
    - "https://github.com/mabhisharma/Multi-Object-Tracking-with-Kalman-Filter/blob/master/kalmanFilter.py"
    - "https://cookierobotics.com/071/"
    '''
    def __init__(self, a_sd: float = 0.01, u: float = 0.1, sd_acceleration: float = 1):
        self.sd_acceleration = sd_acceleration

        #Acceleration
        self.u = u

        #  State vector ; it's [ angle ;  angular velocity]
        self.x = np.matrix([[0], [0]])

        # The matrix that maps state vector to measurement 
        self.H = np.matrix([[1, 0]])

        # Measurement Covariance
        self.R = a_sd ** 2

        # The error covariance matrix that is Identity for now. It gets updated based on Q, A and R.
        self.P = np.eye(self.x.shape[0])
        
    def predict(self, dt):
        self.B = np.matrix([[(dt**2)/2], [dt]]) 

        # The state transition matrix 
        self.A = np.matrix([[1, dt],
                            [0, 1]] )

        # Processs Covariance that for our case depends solely on the acceleration
        self.Q = np.matrix([[(dt**4)/4, (dt**3)/2],
                            [(dt**3)/2, dt**2]]) * self.sd_acceleration**2

        # If we add the B.u it doesnt work... maybe its a dt problem
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        
        # Updation of the error covariance matrix 
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q

        return self.x

    def update(self, z: np.matrix):
        # z is the measurement taken, it should be a matrix [[angle]] measured.
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

    def set_param(self, a_sd: Optional[float] = None,
                        u_a:  Optional[float] = None,
                        acceleration_sd_1d: Optional[float] = None):

        self.R = a_sd ** 2 if a_sd else self.R
        
        self.u = u_a if u_a else self.u

        self.sd_acceleration = acceleration_sd_1d if acceleration_sd_1d else self.sd_acceleration
        