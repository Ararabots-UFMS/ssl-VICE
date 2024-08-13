from vision.tracker import ObjectTracker, Object, ID

from system_interfaces.msg import VisionMessage, Robots, Balls, ObjectID

from typing import List


def wrap_message(objects: List[Object]) -> VisionMessage:
    message = VisionMessage()

    for i, object_ in enumerate(objects):
        if object_.id.is_ball:
            ball_msg = Balls()
            ball_id = ObjectID()

            ball_id.id = object_.id.id
            ball_id.is_ball = object_.id.is_ball
            ball_msg.id = ball_id

            # Kalman filter x attribute is a vector [x position, y position, x velocity, y velocity]
            ball_msg.position_x = float(object_.KF.x[0][0])
            ball_msg.position_y = float(object_.KF.x[1][0])

            ball_msg.velocity_x = float(object_.KF.x[2][0])
            ball_msg.velocity_y = float(object_.KF.x[3][0])

            message.balls.append(ball_msg)

        else:
            robot_msg = Robots()
            robot_id = ObjectID()

            robot_id.id = object_.id.id
            robot_id.is_ball = object_.id.is_ball
            robot_id.is_blue = object_.id.is_blue
            robot_msg.id = robot_id

            robot_msg.position_x = float(object_.KF.x[0][0])
            robot_msg.position_y = float(object_.KF.x[1][0])

            robot_msg.velocity_x = float(object_.KF.x[2][0])
            robot_msg.velocity_y = float(object_.KF.x[3][0])

            robot_msg.orientation = float(object_.orientation_KF.x[0][0])
            robot_msg.velocity_orientation = float(object_.orientation_KF.x[1][0])

            if object_.id.is_blue:
                message.blue_robots.append(robot_msg)
            else:
                message.yellow_robots.append(robot_msg)

    return message