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
    # Defaults are in millimetres, matched to SSL hardware at ~60 Hz:
    # - sd_acceleration is the process noise. Robots accelerate at 3000-5000 mm/s²;
    #   the old value of 1 mm/s² made the filter distrust motion and lag the true
    #   velocity by up to 900 mm/s during an acceleration ramp.
    # - x_sd / y_sd is the SSL-Vision measurement noise, realistically 5-20 mm,
    #   not the 0.1 mm the filter used to assume.
    # - u is the control input. The vision node has no access to the commanded
    #   acceleration, so it must be zero — a constant bias just drifts the estimate.
    def __init__(self, x_sd: float = 15.0, y_sd: float = 15.0, u_x: float = 0.0, u_y: float = 0.0, sd_acceleration: float = 3000.0):
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

    def initialize(self, x: float, y: float, velocity_sd: float = 3000.0):
        '''
        Seeds the filter with the first detection of an object.

        Without this the state starts at the origin with P = I, which claims a 1 mm
        confidence in a position the filter has never measured: the initial gain comes
        out at ~R^-1 and the estimate crawls to the real position over ~1 s, publishing
        metres of error in the meantime.

        Position starts at the measurement with the sensor's own variance. A single
        frame carries no velocity information, so velocity starts at zero with a
        variance wide enough to cover a robot at full speed.
        '''
        self.x = np.matrix([[x], [y], [0.0], [0.0]])
        self.P = np.matrix([[self.R[0, 0], 0.0, 0.0, 0.0],
                            [0.0, self.R[1, 1], 0.0, 0.0],
                            [0.0, 0.0, velocity_sd ** 2, 0.0],
                            [0.0, 0.0, 0.0, velocity_sd ** 2]])

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

        # Numerically stable Joseph form, same as the 1D filter.
        I = np.eye(self.H.shape[1])
        I_KH = I - np.dot(K, self.H)
        self.P = np.dot(I_KH, self.P).dot(I_KH.T) + np.dot(K, self.R).dot(K.T)

        return self.x

    def set_param(self, x_sd: Optional[float] = None,
                        y_sd: Optional[float] = None,
                        u_x:  Optional[float] = None,
                        u_y:  Optional[float] = None,
                        acceleration_sd_2d: Optional[float] = None):

        # Compared against None, not truthiness, so that a caller can set u to zero.
        if x_sd is not None:
            self.R[0] = [x_sd**2, 0]
        if y_sd is not None:
            self.R[1] = [0, y_sd**2]

        if u_x is not None:
            self.u[0] = [u_x]
        if u_y is not None:
            self.u[1] = [u_y]

        if acceleration_sd_2d is not None:
            self.sd_acceleration = acceleration_sd_2d

class ExtendedKalmanFilterClass2D(object):
    '''
    Extended Kalman Filter for 2D (position and velocity) with non-linear motion model.
    
    Assumes constant acceleration model with friction (exponential decay in velocity).
    '''
    # Defaults are in millimetres, matched to SSL hardware at ~60 Hz:
    # - sd_acceleration is the process noise. Robots accelerate at 3000-5000 mm/s²;
    #   a value of 1 mm/s² makes the filter distrust motion and lag the true
    #   velocity by up to 900 mm/s during an acceleration ramp.
    # - x_sd / y_sd is the SSL-Vision measurement noise, realistically 5-20 mm,
    #   not the 0.1 mm the filter used to assume.
    # - u is the control input. The vision node has no access to the commanded
    #   acceleration, so it must be zero — a constant bias just drifts the estimate.
    def __init__(self, x_sd: float = 15.0, y_sd: float = 15.0, u_x: float = 0.0, u_y: float = 0.0, sd_acceleration: float = 3000.0, friction: float = 0.1):
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

    def initialize(self, x: float, y: float, velocity_sd: float = 3000.0):
        '''
        Seeds the filter with the first detection of an object.

        Seeding the state alone is not enough: leaving P = I claims a 1 mm confidence
        in position and a 1 mm/s confidence in a velocity that has never been measured,
        so the filter ignores the next several frames of real motion before the
        estimate catches up.

        Position starts at the measurement with the sensor's own variance. A single
        frame carries no velocity information, so velocity starts at zero with a
        variance wide enough to cover a robot at full speed.
        '''
        self.x = np.matrix([[x], [y], [0.0], [0.0]])
        self.P = np.matrix([[self.R[0, 0], 0.0, 0.0, 0.0],
                            [0.0, self.R[1, 1], 0.0, 0.0],
                            [0.0, 0.0, velocity_sd ** 2, 0.0],
                            [0.0, 0.0, 0.0, velocity_sd ** 2]])

    def _transition_function(self, x, dt):
        # Non-linear transition: constant acceleration with friction
        x_new = np.zeros_like(x)
        x_new[0] = x[0] + x[2] * dt + 0.5 * self.u[0] * dt**2  # posição x
        x_new[1] = x[1] + x[3] * dt + 0.5 * self.u[1] * dt**2  # posição y
        # Velocity with exponential decay due to friction
        x_new[2] = x[2] * np.exp(-self.friction * dt) + self.u[0] * dt
        x_new[3] = x[3] * np.exp(-self.friction * dt) + self.u[1] * dt
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

class KalmanFilterClass1D(object):
    '''
    Kalman filter for 1D (Angle and Angular velocity).

    Serves for orientation purposes, employing the information in path planning.
    This version is modified to handle the circular nature of angles in the [-pi, pi] range
    and uses a numerically stable covariance update to prevent divergence.
    '''
    # a_sd is ssl-vision's orientation noise (~0.02 rad), sd_acceleration the angular
    # acceleration the robot can actually reach. At 0.1/1.0 the filter trusted a
    # constant-rate model over the measurements and lagged badly through every turn.
    def __init__(self, a_sd: float = 0.02, u: float = 0.0, sd_acceleration: float = 50.0):
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

    def initialize(self, angle: float, velocity_sd: float = 30.0):
        '''
        Seeds the filter with the first orientation measured for an object.
        Same reasoning as the 2D filter: one frame gives an angle but no angular rate.
        '''
        self.x = np.matrix([[self._wrap_angle(float(angle))], [0.0]])
        self.P = np.matrix([[self.R[0, 0], 0.0],
                            [0.0, velocity_sd ** 2]])

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wraps an angle to the [-pi, pi] range."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def predict(self, dt):
        # Process noise covariance
        self.Q = np.matrix([[(dt**4)/4, 0, (dt**3)/2, 0],
                            [0, (dt**4)/4, 0, (dt**3)/2],
                            [(dt**3)/2, 0, dt**2, 0],
                            [0, (dt**3)/2, 0, dt**2]]) * self.sd_acceleration ** 2
        
        # Cache prior state so the EKF linearization uses the pre-transition state
        prior_x = self.x.copy()

        # Predict covariance using the Jacobian evaluated at the prior state
        F = self._jacobian_F(prior_x, dt)

        # Predict state
        self.x = self._transition_function(prior_x, dt)
        
        # Predict covariance
        self.P = F @ self.P @ F.T + self.Q

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
    # a_sd is the orientation measurement noise in radians (~1 degree) and
    # sd_acceleration the angular process noise; a robot can spin up far faster than
    # the 1 rad/s² the original default assumed, which smeared heading through a turn.
    def __init__(self, a_sd: float = 0.02, u: float = 0.0, sd_acceleration: float = 50.0, friction: float = 0.1):
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

    def initialize(self, angle: float, velocity_sd: float = 30.0):
        '''
        Seeds the filter with the first orientation measured for an object.
        Same reasoning as the 2D filter: one frame gives an angle but no angular rate.
        '''
        self.x = np.matrix([[self._wrap_angle(float(angle))], [0.0]])
        self.P = np.matrix([[self.R[0, 0], 0.0],
                            [0.0, velocity_sd ** 2]])
        
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
        # Matriz de transição para 1D (posição e velocidade)
        F = np.array([[1, dt],
                    [0, 1]])

        # Ruído de processo compatível com 2x2
        Q = np.array([[dt**4/4, dt**3/2],
                    [dt**3/2, dt**2]]) * self.sd_acceleration**2

        # Predição do estado
        self.x = F @ self.x
        # Predição da covariância
        self.P = F @ self.P @ F.T + Q

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