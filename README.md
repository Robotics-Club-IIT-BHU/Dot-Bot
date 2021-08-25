# Swarm-Bot
Ros repository for the swarm bots

# Setup guidelines 

```bash
cd catkin_ws/src
git clone git@github.com:Robotics-Club-IIT-BHU/Swarm-Bot.git
./Swarm-Bot/setup.bash
cd ..
catkin build
```
# Localization Filter

Use the odom_ekf.launch file to launch the node of the fusion of imu+encoder data. Reference link: https://roverrobotics.com/blogs/guides/fusing-imu-encoders-with-ros-robot-localization
Explanation of code: https://github.com/cra-ros-pkg/robot_localization/blob/32896d6d1aaec5a92c3a65e5f16dfc0b859a7d26/params/ekf_template.yaml

# Teb & Marker Array

Reference Tut: http://wiki.ros.org/teb_local_planner/Tutorials/Incorporate%20dynamic%20obstacles
