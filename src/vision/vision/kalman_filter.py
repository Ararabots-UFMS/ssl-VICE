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
    def __init__(self, x_sd: float = 0.1, y_sd: float = 0.1, u_x: float = 0.1, u_y: float = 0.1, sd_acceleration: float = 1):
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

class ExtendedKalmanFilterClass2D(object):
    '''
    Extended Kalman Filter for 2D (position and velocity) with non-linear motion model.
    
    Assumes constant acceleration model with friction (exponential decay in velocity).
    '''
    def __init__(self, x_sd: float = 0.1, y_sd: float = 0.1, u_x: float = 0.1, u_y: float = 0.1, sd_acceleration: float = 1, friction: float = 0.1):
        self.sd_acceleration = sd_acceleration
        self.friction = friction  # Friction coefficient for velocity decay

        self.u = np.matrix([[u_x],[u_y]])

        # State vector: [x, y, vx, vy]
        self.x = np.matrix([[0], [0], [0], [0]])

        # Measurement matrix (linear: measures position)
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])

        # Measurement Covariance
        self.R = np.matrix([[x_sd**2, 0],
                           [0, y_sd**2]])

        # Error covariance matrix
        self.P = np.eye(self.x.shape[0])
        
    def _transition_function(self, x, dt):
        # Non-linear transition: constant acceleration with friction
        x_new = np.zeros_like(x)
        x_new[0] = x[0] + x[2] * dt + 0.5 * self.u[0] * dt**2  # x
        x_new[1] = x[1] + x[3] * dt + 0.5 * self.u[1] * dt**2  # y
        x_new[2] = x[2] * np.exp(-self.friction * dt) + self.u[0] * dt  # vx with decay
        x_new[3] = x[3] * np.exp(-self.friction * dt) + self.u[1] * dt  # vy with decay
        return x_new
    
    def _jacobian_F(self, x, dt):
        # Jacobian of transition function
        F = np.matrix([[1, 0, dt, 0],
                       [0, 1, 0, dt],
                       [0, 0, np.exp(-self.friction * dt), 0],
                       [0, 0, 0, np.exp(-self.friction * dt)]])
        return F
    
    def predict(self, dt):
        # Process noise covariance
        self.Q = np.matrix([[(dt**4)/4, 0, (dt**3)/2, 0],
                            [0, (dt**4)/4, 0, (dt**3)/2],
                            [(dt**3)/2, 0, dt**2, 0],
                            [0, (dt**3)/2, 0, dt**2]]) * self.sd_acceleration ** 2

        # Predict state
        self.x = self._transition_function(self.x, dt)
        
        # Predict covariance
        F = self._jacobian_F(self.x, dt)
        self.P = F @ self.P @ F.T + self.Q

        return self.x

    def update(self, z: np.matrix):
        # Measurement function (linear)
        h = self.H @ self.x
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman Gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        
        # Update covariance
        I = np.eye(self.x.shape[0])
        self.P = (I - K @ self.H) @ self.P
        
        return self.x
    
    def set_param(self, x_sd: Optional[float] = None,
                        y_sd: Optional[float] = None,
                        u_x:  Optional[float] = None,
                        u_y:  Optional[float] = None,
                        acceleration_sd_2d: Optional[float] = None,
                        friction: Optional[float] = None):

        if x_sd is not None:
            self.R[0, 0] = x_sd**2
        if y_sd is not None:
            self.R[1, 1] = y_sd**2
        
        if u_x is not None:
            self.u[0] = u_x
        if u_y is not None:
            self.u[1] = u_y

        if acceleration_sd_2d is not None:
            self.sd_acceleration = acceleration_sd_2d
        if friction is not None:
            self.friction = friction

import numpy as np
from typing import Optional

class KalmanFilterClass1D(object):
    '''
    Kalman filter for 1D (Angle and Angular velocity).

    Serves for orientation purposes, employing the information in path planning.
    This version is modified to handle the circular nature of angles in the [-pi, pi] range
    and uses a numerically stable covariance update to prevent divergence.
    '''
    def __init__(self, a_sd: float = 0.1, u: float = 0.0, sd_acceleration: float = 1.0):
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

class ExtendedKalmanFilterClass1D(object):
    '''
    Extended Kalman Filter for 1D (angle and angular velocity) with non-linear motion model.
    
    Assumes constant angular acceleration with friction.
    '''
    def __init__(self, a_sd: float = 0.1, u: float = 0.0, sd_acceleration: float = 1.0, friction: float = 0.1):
        self.sd_acceleration = sd_acceleration
        self.friction = friction
        self.u = u
        # State vector: [theta, omega]
        self.x = np.matrix([[0], [0]])
        # Measurement matrix (linear: measures angle)
        self.H = np.matrix([[1, 0]])
        # Measurement Covariance
        self.R = np.matrix([[a_sd ** 2]])
        # State Covariance matrix
        self.P = np.eye(self.x.shape[0])
        
    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wraps an angle to the [-pi, pi] range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _transition_function(self, x, dt):
        # Non-linear transition: constant angular acceleration with friction
        theta_new = x[0, 0] + x[1, 0] * dt + 0.5 * self.u * dt**2
        omega_new = x[1, 0] * np.exp(-self.friction * dt) + self.u * dt
        return np.matrix([[self._wrap_angle(theta_new)], [omega_new]])
    
    def _jacobian_F(self, x, dt):
        # Jacobian of transition function
        F = np.matrix([[1, dt],
                       [0, np.exp(-self.friction * dt)]])
        return F
    
    def predict(self, dt: float):
        # Process noise covariance
        self.Q = np.matrix([[(dt**4)/4, (dt**3)/2],
                            [(dt**3)/2, dt**2]]) * self.sd_acceleration**2

        # Predict state
        self.x = self._transition_function(self.x, dt)
        
        # Predict covariance
        F = self._jacobian_F(self.x, dt)
        self.P = F @ self.P @ F.T + self.Q

        return self.x

    def update(self, z: np.matrix):
        # Measurement function (linear)
        h = self.H @ self.x
        
        # Innovation (with angle wrapping)
        y = z - h
        y[0, 0] = self._wrap_angle(y[0, 0])
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman Gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.x = self.x + K @ y
        self.x[0, 0] = self._wrap_angle(self.x[0, 0])
        
        # Update covariance (Joseph form for stability)
        I = np.eye(self.x.shape[0])
        self.P = (I - K @ self.H) @ self.P @ (I - K @ self.H).T + K @ self.R @ K.T
        
        return self.x

    def set_param(self, a_sd: Optional[float] = None,
                        u_a:  Optional[float] = None,
                        acceleration_sd_1d: Optional[float] = None,
                        friction: Optional[float] = None):
        if a_sd is not None:
            self.R = np.matrix([[a_sd ** 2]])
        if u_a is not None:
            self.u = u_a
        if acceleration_sd_1d is not None:
            self.sd_acceleration = acceleration_sd_1d
        if friction is not None:
            self.friction = friction