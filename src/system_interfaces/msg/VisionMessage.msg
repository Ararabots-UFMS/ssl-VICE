Robots[] yellow_robots
Robots[] blue_robots
Balls[] balls
# Instant the estimates below refer to, in the ssl-vision camera clock (t_capture).
float64 capture_stamp
# The same instant expressed in the ROS clock, taken when the packet was received.
# Consumers use this one to age the data; capture_stamp is only comparable to itself.
float64 wall_stamp
