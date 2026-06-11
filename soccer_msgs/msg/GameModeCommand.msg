std_msgs/Header header

# IDLE, PENALTY_ATTACK, GOALKEEPER, or CALIBRATION.
string mode

# Only used by PENALTY_ATTACK. Left is positive, right is negative, range [-1, 1].
float32 goal_target

# Distance from the fixed navigation point to the ball.
float32 nav_ball_distance_m

bool reset_state
