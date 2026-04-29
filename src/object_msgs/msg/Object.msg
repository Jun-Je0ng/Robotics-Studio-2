std_msgs/Header header
geometry_msgs/Pose pose
string classification       # material type e.g. 'metal', 'plastic', 'fabric'
float64[3] dimensions       # dimensions of the object aligned to the pose . i.e. this is what the bounding box definition would be

