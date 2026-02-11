std_msgs/Header header
string source              # "yolo" or "aruco"
float32 confidence         # detection confidence (0-1)
geometry_msgs/PoseStamped target_pose   # estimated position in map frame
bool is_precise            # true if from ArUco (cm-level), false if from YOLO (m-level)
