from utils.math_utils import angle_between, distancePoints, unitVector, forward_min_diff, predict_speed
from utils.yaml_handler import YamlHandler

from control.PIDclient import PIDClientAsync
from univector.univec_field import UnivectorField
from utils.linalg import *

univector_list = YamlHandler().read("parameters/univector.yml")

# univector parameters
RADIUS = univector_list['RADIUS']
KR = univector_list['KR']
K0 = univector_list['K0']
DMIN = univector_list['DMIN']
LDELTA = univector_list['LDELTA']

RIGHT = 1
LEFT = 0

SOFTWARE = 0
HARDWARE = 1

FORWARD = True
BACKWARDS = False

class Movement():
    """Movement class return leftWheelSpeed(int), rightWheelSpeed(int), done(boolean)"""

    def __init__(self, PID_list, error=10, attack_goal=RIGHT, _pid_type=SOFTWARE, _debug_topic = None):
        self.pid = PIDClientAsync(kp=PID_list[0], ki=PID_list[1], kd=PID_list[2]) #PID(kp=PID_list[0], ki=PID_list[1], kd=PID_list[2])
        self.last_pos = Vec2D.origin()
        self.error_margin = error
        self.orientation = FORWARD
        self.attack_goal = attack_goal
        self.gamma_count = 0
        self.simulation = False
        if type(attack_goal) is int:
            self.univet_field = UnivectorField(attack_goal=self.attack_goal)
        else:
            self.univet_field = UnivectorField(attack_goal=attack_goal, _rotation=True)

        self.univet_field.update_constants(RADIUS, KR, K0, DMIN, LDELTA)
        self.pid_type = _pid_type
        self.debug_topic = _debug_topic

    def set_pid_type(self, _pid_type):
        """
            Define if the pid correction is done on SOFTWARE or HARDWARE (0 or 1 respectively)
        """
        self.pid_type = _pid_type

    def initialize_simulation(self):
        """
        Initialize flag for simulation
        :return:
        """
        self.simulation = True

    def update_pid(self, pid_list):
        """
        Update pid paramters
        :param pid_list: [float, float, float]
        :return:
        """
        self.pid.set_parameters(pid_list[0], pid_list[1], pid_list[2])

    def predict_univector(self, speed, number_of_predictions, robot_position, robot_vector, robot_speed, obstacle_position, obstacle_speed, ball_position, only_forward=False):
        """Recive players positions and speed and return the speed to follow univector
         :param speed : int
         :param robot_position : np.array([float, float])
         :param robot_vector : np.array([float, float])
         :param robot_speed : np.array([float, float])
         :param obstacle_position : np.array([float, float])
         :param obstacle_speed : np.array([float, float])
         :param ball_position : np.array([float, float])
         :param number_of_predictions: int
        :param only_forward: boolean

        :return: returns nothing
        """
        # TODO: Método nunca chamado...
        #TODO: Testar a predicao dos vetores
        self.univet_field.update_obstacles(obstacle_position, obstacle_speed)
        vec_result = Vec2D.origin()
        robot_position_aux = robot_position
        for i in range(number_of_predictions):
            vec = self.univet_field.get_vec(robot_position_aux, robot_speed, ball_position)
            vec_result += vec
            robot_position_aux += Vec2D(int(robot_speed[0]*0.016), int(robot_speed[1]*0.016))

        return self.follow_vector(speed, robot_vector, vec_result.versor())

    def do_univector(self, speed, robot_position, robot_vector, robot_speed, obstacle_position, obstacle_speed, ball_position, only_forward=False, speed_prediction=False):
        """Receive players positions and speed and return the speed to follow univector
         :param speed : int
         :param robot_position : np.array([float, float])
         :param robot_vector : np.array([float, float])
         :param robot_speed : np.array([float, float])
         :param obstacle_position : np.array([float, float])
         :param obstacle_speed : np.array([float, float])
         :param ball_position : np.array([float, float])
         :param only_forward : boolean
         :param speed_prediction : boolean

        :return: returns nothing
        """
        self.univet_field.update_obstacles(obstacle_position, obstacle_speed)
        vec = self.univet_field.get_vec_with_ball(robot_position, robot_speed, ball_position)

        if speed_prediction:
            # central area speed
#            raio = raio_vetores(robot_position, robot_vector, ball_position,
#                                np.array(self.univet_field.get_attack_goal() - ball_position), speed, 500, angle=0.07)
            raio = predict_speed(robot_position, robot_vector, ball_position, self.attack_goal)
            cte = 90
            speed = (raio * cte) ** 0.5 + 10

        return self.follow_vector(speed, robot_vector, vec, only_forward)

    def do_univector_ball(self, speed, robot_position, robot_vector, robot_speed, obstacle_position, obstacle_speed, ball_position):
        """Recive players positions and speed and return the speed to follow univector
         :param speed : int
         :param robot_position : np.array([float, float])
         :param robot_vector : np.array([float, float])
         :param robot_speed : np.array([float, float])
         :param obstacle_position : np.array([float, float])
         :param obstacle_speed : np.array([float, float])
         :param ball_position : np.array([float, float])

        :return: returns nothing
        """
        self.univet_field.update_obstacles(obstacle_position, obstacle_speed)
        vec = self.univet_field.get_vec_with_ball(robot_position, robot_speed, ball_position)
        return self.follow_vector(speed, robot_vector, vec)

    def in_goal_position(self, robot_position, goal_position):
        """Verify if the robot is in goal position and return a boolean of the result
         :param robot_position : np.array([float, float])
         :param goal_position : np.array([float, float])

        :return: returns : boolean
        """
        if distancePoints(robot_position, goal_position) <= self.error_margin:
            return True
        return False

    def in_goal_vector(self, robot_vector, goal_vector):
        """Verify if the robot is in goal vector and return a boolean of the result
         :param robot_vector : np.array([float, float])
         :param goal_vector : np.array([float, float])

         :return: returns : boolean
        """
        if abs(angle_between(robot_vector, goal_vector, absol=False)) <= 0.087266463: #5 degrees error
            return True
        return False

    def move_to_point(self, speed, robot_position, robot_vector, goal_position, only_forward=False):
        """Recives robot position, robot direction vector, goal position and a speed.
        Return the speed os the wheel to follow the vector (goal - robot)
         :param speed : int
         :param robot_position : np.array([float, float])
         :param robot_vector : np.array([float, float])
         :param goal_position : np.array([float, float])
         :param only_forward : boolean

        :return: returns int, int, boolean
        """
        if self.in_goal_position(robot_position, goal_position):
            return 0, 0, True
        direction_vector = unitVector(goal_position - robot_position)

        return self.follow_vector(speed, robot_vector, direction_vector, only_forward)

    def follow_vector(self, speed, robot_vector, goal_vector, only_forward=False, correct_pid = True):
        """Recives the robot vector, goal vector and a speed and return the speed
        of the wheels to follow the goal vector
         :param speed : int
         :param robot_vector : np.array([float, float])
         :param goal_vector : np.array([float, float])
         :param only_forward : boolean
         :param correct_pid: boolean

        :return: returns int, int, boolean
        """
        if self.debug_topic is not None:
            self.debug_topic.debug_publish(goal_vector.tolist())

        forward, diff_angle, self.gamma_count = forward_min_diff(self.gamma_count, self.orientation, robot_vector, goal_vector, only_forward)
        #forward, diff_angle = get_orientation_and_angle(self.orientation, robot_vector, goal_vector)
        self.orientation = forward

        # Return the speed and angle if the PID is in hardware, otherwise
        # returns both wheels speed and its correction
        if self.pid_type == HARDWARE:
            if forward:
                return diff_angle, speed, False
            return diff_angle, -speed, False

        correction = self.pid.request_update(diff_angle)

        if forward:
            return self.return_speed(speed, correction)
        return self.return_speed(-speed, correction)

    def spin(self, speed, ccw=True):
        """Recives a speed and a boolean counterclockwise and return the left wheel speed,
           right wheel speed and a boolean. Spin the robot
         :param speed : int
         :param ccw : boolean

         :return: returns int, int, boolean
        """
        if ccw:
            return int(-speed), int(speed), False
        return int(speed), int(-speed), False

    def head_to(self, robot_vector, goal_vector, multiplicator=1):
        """Receive robot direction vector, goal vector and a speed. Return the left wheels speed,
        right wheel speed and done. Robot vector and goal vector will be parallels vectors, mult will speed up
        the correction velocity.
        :param robot_vector : np.array([float, float])
        :param goal_vector : np.array([float, float])
        :param multiplicator : float

        :return: returns int, int, boolean
        """
        diff_angle = angle_between(robot_vector, goal_vector, absol=False)

        if self.in_goal_vector(robot_vector, goal_vector):
            return 0, 0, True

        if self.pid_type == HARDWARE:
            return diff_angle, 0, False

        correction = multiplicator * self.pid.request_update(diff_angle)
        return self.normalize(int(correction)), self.normalize(int(-correction)), False

    def return_speed(self, speed, correction):
        """Recives the robot speed and the PID correction, and return each wheel speed.
        :param speed : int
        :param correction : int

        :return: returns int, int, boolean
        """
        if not self.simulation:
            return self.normalize(int(speed + correction)), self.normalize(int(speed - correction)), False
        return self.normalize(int(speed - correction)), self.normalize(int(speed + correction)), False

    @staticmethod
    def normalize(speed):
        """Normalize robot speed
            :param speed: int
            :return: return int
        """
        if abs(speed) > 255:
            return 255*speed/abs(speed)
        return speed

if __name__ == "__main__":
    from utils.linalg import Vec2D
    from utils.math_utils import angle_between

    movement = Movement([1, 1, 1])
    robot_vector = Vec2D(1, 0)
    goal_vector = Vec2D(1, 0)
    print(movement.follow_vector(speed=10, robot_vector=robot_vector, goal_vector=goal_vector))