std_msgs/Header header

uint8 TYPE_ARUCO = 0
uint8 TYPE_COLOR = 1

uint8 type
int32 aruco_id       # valid when type == TYPE_ARUCO
string label         # color name, or "aruco_<id>"
float32 u            # pixel centroid x
float32 v            # pixel centroid y
float32[4] bbox      # x, y, width, height
float32 confidence   # blob area fraction of bbox for color, 1.0 for aruco
