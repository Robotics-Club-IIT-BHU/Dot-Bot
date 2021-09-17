cp -r dot_gazebo/world/ps1/my_ground_plane ~/.gazebo/models/my_ground_plane
cp -r dot_gazebo/world/ps2/ps2_ground_plane ~/.gazebo/models/ps2_ground_plane
cp -r dot_gazebo/world/ps1/Apriltag36/* ~/.gazebo/models/

git clone https://github.com/ros-planning/navigation.git ../navigation

echo "Done Copying models"


