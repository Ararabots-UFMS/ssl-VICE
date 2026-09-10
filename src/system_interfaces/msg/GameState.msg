Robots[] ally_robots
Robots[] enemy_robots
Balls[] balls
GUIMessage gui
RefereeMessage referee
RefereeMessage referee_last_command
VisionGeometry geometry
bool can_i_start
float32 can_i_kick
# Stamps of the vision data the robot/ball estimates came from, carried through from
# VisionMessage. Zero until the first vision packet arrives.
float64 vision_capture_stamp
float64 vision_wall_stamp
