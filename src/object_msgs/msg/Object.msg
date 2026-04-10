std_msgs/Header header
geometry_msgs/Pose pose
string classification       # material type e.g. 'metal', 'plastic', 'fabric'
float64[3] dimensions       # [thin, mid, thick] in metres — measured by perception
uint8 thin_axis             # index of thinnest axis: 0=x, 1=y, 2=z
uint8 bin_index             # which bin this object maps to (see BIN_MAP)
