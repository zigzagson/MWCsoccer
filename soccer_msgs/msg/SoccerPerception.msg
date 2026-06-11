std_msgs/Header header

# Whether the current image frame contains a valid ball detection.
bool image_has_ball

# Whether the current image frame contains a valid goal detection.
bool image_has_goal

# Recommended frame_id: dummy. x is forward distance, y is lateral offset.
geometry_msgs/PointStamped ball
geometry_msgs/PointStamped goal_center
geometry_msgs/PointStamped left_post
geometry_msgs/PointStamped right_post

float32 ball_confidence
float32 goal_confidence
bool transform_valid

# Free-form status such as "ok", "stale_ball", "stale_goal", or transform errors.
string detail
