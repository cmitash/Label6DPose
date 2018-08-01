# Label6DPose
label 6d pose of objects in images by manually aligning object models to the pointcloud obtained from depth and color image

# Setup
* cd src/
* catkin_init_workspace
* cd ../
* catkin_make

# Run example
* source devel/setup.sh
* rosrun label_poses label_poses /path/to/repository/scene-0001 crayola_24_ct
