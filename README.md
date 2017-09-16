# pose_estimation

The code subscribes to the topic /camera/image_raw and publishes the pose to my_pose topic. 
my_pose is of type geometry_msgs/pose. The pose contains two parameters:

1. Position: in centimeters 
2. Orientation: In quaternions. (X->Y->Z) euler notation is used.

The marker is an 'H', enclosed within two circles. The size of 'H' is 10.84 x 7.30 centimeters.

The x-axis is along the longer side of H. The y-axis is away from the shorter side of H. Right hand coordinate system notation is used.

The video relevant is can be found on: https://youtu.be/H3O-9_tb7K0

Crammed everything in 421 lines of code. Used inline functions, call by reference, global arrays, pre-increment operators, constructors and destructors to reduce runtime.

