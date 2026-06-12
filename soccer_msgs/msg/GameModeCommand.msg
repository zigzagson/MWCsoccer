std_msgs/Header header

# IDLE, PENALTY_ATTACK, GOALKEEPER, or CALIBRATION.
string mode

# Only used by PENALTY_ATTACK. Left post is -1, right post is 1, range [-1, 1].
# For example, 0.3 targets the right side of center.
float32 goal_target

# Distance from the fixed navigation point to the ball.
float32 nav_ball_distance_m

bool reset_state
