from vision.tracker import ObjectTracker, Object, ID

from system_interfaces.msg import (
    VisionMessage,
    VisionGeometry,
    Robots,
    Balls,
    FieldLineSegment,
    FieldArcsSegment,
    FieldLineType,
)

from vision.proto.messages_robocup_ssl_geometry_pb2 import SSL_GeometryData

from typing import List


def wrap_message(objects: List[Object]) -> VisionMessage:
    message = VisionMessage()

    for object_ in objects:
        if object_.id.is_ball:
            ball_msg = Balls()

            ball_msg.id = object_.id.id

            # Kalman filter x attribute is a vector [x position, y position, x velocity, y velocity]
            ball_msg.position_x = float(object_.KF.x[0][0])
            ball_msg.position_y = float(object_.KF.x[1][0])

            ball_msg.velocity_x = float(object_.KF.x[2][0])
            ball_msg.velocity_y = float(object_.KF.x[3][0])

            message.balls.append(ball_msg)

        else:
            robot_msg = Robots()

            robot_msg.id = object_.id.id

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


def wrap_geo_message(message: SSL_GeometryData):
    wraped_msg = VisionGeometry()

    wraped_msg.field_length = message.field.field_length
    wraped_msg.field_width = message.field.field_width
    wraped_msg.goal_width = message.field.goal_width
    wraped_msg.goal_depth = message.field.goal_depth
    wraped_msg.boundary_width = message.field.boundary_width

    wraped_msg.field_lines = []
    wraped_msg.field_arcs = []

    for message_line_segment in message.field.field_lines:
        wraped_msg_line = FieldLineSegment()

        wraped_msg_line.name = message_line_segment.name
        wraped_msg_line.x1 = message_line_segment.p1.x
        wraped_msg_line.y1 = message_line_segment.p1.y
        wraped_msg_line.x2 = message_line_segment.p2.x
        wraped_msg_line.y2 = message_line_segment.p2.y

        wraped_msg_line.thickness = message_line_segment.thickness

        tp = FieldLineType()
        tp.type = message_line_segment.type

        wraped_msg_line.type = tp

        wraped_msg.field_lines.append(wraped_msg_line)

    for message_arc_segment in message.field.field_arcs:
        wraped_msg_arc = FieldArcsSegment()

        wraped_msg_arc.name = message_arc_segment.name
        wraped_msg_arc.x = message_arc_segment.center.x
        wraped_msg_arc.y = message_arc_segment.center.y
        wraped_msg_arc.radius = message_arc_segment.radius
        wraped_msg_arc.starting_angle = message_arc_segment.a1
        wraped_msg_arc.end_angle = message_arc_segment.a2

        wraped_msg_arc.thickness = message_arc_segment.thickness

        tp = FieldLineType()
        tp.type = message_arc_segment.type

        wraped_msg_arc.type = tp

        wraped_msg.field_arcs.append(wraped_msg_arc)

    return wraped_msg
