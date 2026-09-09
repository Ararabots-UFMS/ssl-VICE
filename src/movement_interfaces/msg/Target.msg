uint8 robot_id
Vector2D initial_pos
Vector2D initial_vel
Vector2D target_pos
Vector2D target_vel
PlanningOptions planning_options
# ROS-clock instant initial_pos/initial_vel refer to, so a plan built from them can
# be stamped with the age its t=0 already has.
float64 vision_stamp
