# Swarm-Bot
This is the ROS repository for MultiPle omni drive based robots used in a constrained indoor setting for navigating to efficently reache their respective goals. 
We have designed a modular setting for a deploying both centralized and decentralized planning for traversing the terrain with other robts in the enviroment. The major locatization is done using Apriltags and used odometry for more accurate control. We have Also Implemented MultiAgent Reinforcement LEarning in a Decentralized as well as Centrallized Manner which can be found in the other repository.

For Hardware code visi This particular repo [Swarmbot-Hardware](https://github.com/Robotics-Club-IIT-BHU/Swarm-Bot-Hardware)

# Setup guidelines 

```bash
cd catkin_ws/src
git clone git@github.com:Robotics-Club-IIT-BHU/Swarm-Bot.git
cd Swarm-Bot
./setup.sh
cd ../..
catkin build #or catkin_make
```


# Launching 
```bash
roslaunch dot_gazebo gazebo.launch ps:=ps1
roslaunch dot_gazebo gazebo.launch ps:=ps2
```

# Structure


            .
            ├── apriltag_ros
            │   ├── CHANGELOG.rst
            │   ├── CMakeLists.txt
            │   ├── include
            │   │   └── apriltag_ros
            │   │       ├── common_functions.h
            │   │       ├── continuous_detector.h
            │   │       └── single_image_detector.h
            │   ├── launch
            │   │   ├── continuous_detection.launch
            │   │   ├── single_image_client.launch
            │   │   └── single_image_server.launch
            │   ├── msg
            │   │   ├── AprilTagDetectionArray.msg
            │   │   └── AprilTagDetection.msg
            │   ├── nodelet_plugins.xml
            │   ├── package.xml
            │   ├── scripts
            │   │   ├── analyze_image
            │   │   └── calibrate_bundle.m
            │   ├── src
            │   │   ├── apriltag_ros_continuous_node.cpp
            │   │   ├── apriltag_ros_single_image_client_node.cpp
            │   │   ├── apriltag_ros_single_image_server_node.cpp
            │   │   ├── common_functions.cpp
            │   │   ├── continuous_detector.cpp
            │   │   └── single_image_detector.cpp
            │   └── srv
            │       └── AnalyzeSingleImage.srv
            ├── dot_config
            │   ├── CMakeLists.txt
            │   ├── config
            │   │   └── control
            │   │       └── bot_control.yaml
            │   └── package.xml
            ├── dot_control
            │   ├── CMakeLists.txt
            │   ├── include
            │   │   └── dot_control
            │   │       └── omni_control.hpp
            │   ├── launch
            │   │   ├── bring_up.launch
            │   │   └── bring_up_multi.launch
            │   ├── package.xml
            │   ├── README.md
            │   ├── scripts
            │   │   └── tf_apriltag.py
            │   └── src
            │       ├── omnidrive.cpp
            │       ├── omnidrive.cpp.bak
            │       └── omnidrive_node.cpp
            ├── dot_description
            │   ├── CMakeLists.txt
            │   ├── mesh
            │   │   ├── base.stl
            │   │   ├── rim.stl
            │   │   └── roller.stl
            │   ├── package.xml
            │   ├── urdf
            │   │   └── bot.urdf.xacro
            │   └── xacro
            │       ├── main.urdf.xacro
            │       ├── rim.urdf.xacro
            │       └── roller.urdf.xacro
            ├── dot_gazebo
            │   ├── CMakeLists.txt
            │   ├── launch
            │   │   ├── gazebo.launch
            │   │   ├── multi_gazebo.launch
            │   │   ├── parameter.launch
            │   │   ├── robot_load_gazebo.launch
            │   │   ├── rviz.launch
            │   │   └── test.launch
            │   ├── odometry.cpp
            │   ├── package.xml
            │   ├── urdf
            │   │   └── cameras.urdf
            │   └── world
            │       ├── ps1
            │       │   ├── Apriltag36
            │       │   │   ├── Contains tag meshes
            │       │   ├── dummy_.world
            │       │   ├── final_ps.world
            │       │   ├── lol.world
            │       │   ├── my_ground_plane
            │       │   │   ├── materials
            │       │   │   │   ├── scripts
            │       │   │   │   │   └── my_ground_plane.material
            │       │   │   │   └── textures
            │       │   │   │       └── MyImage.png
            │       │   │   ├── model.config
            │       │   │   └── model.sdf
            │       │   ├── ps1_apriltag.world
            │       │   ├── ps1_edit.world
            │       │   ├── ps1_final.world
            │       │   └── ps1_nosignal.world
            │       └── ps2
            │           ├── final_ps2.world
            │           ├── ps2_ground_plane
            │           │   ├── materials
            │           │   │   ├── scripts
            │           │   │   │   └── ps2_ground_plane.material
            │           │   │   └── textures
            │           │   │       └── ps2.jpeg
            │           │   ├── model.config
            │           │   └── model.sdf
            │           └── ps2_nosignal.world
            ├── dot_map
            │   ├── CMakeLists.txt
            │   ├── config
            │   │   ├── my_map.yaml
            │   │   ├── ps1_map.jpeg
            │   │   └── ps1_map.pgm
            │   ├── include
            │   │   └── dot_map
            │   │       └── gazebo_map.hpp
            │   ├── package.xml
            │   └── src
            │       ├── gazebo_map.cpp
            │       └── gazebo_map_node.cpp
            ├── dot_navigation_goals
            │   ├── CMakeLists.txt
            │   ├── package.xml
            │   ├── scripts
            │   └── src
            │       ├── Relay_race.cpp
            │       └── simple_navigation_goals.cpp
            ├── dot_perception
            │   ├── CMakeLists.txt
            │   ├── config
            │   │   ├── settings.yaml
            │   │   └── tags.yaml
            │   ├── info
            │   │   ├── contains_camera_infos
            │   ├── launch
            │   │   ├── gazebo_camera_continous_detection.launch
            │   │   ├── hardware_continous_detection.launch
            │   │   ├── simulation_continous_detection.launch
            │   │   └── usb_continous_detection.launch
            │   ├── package.xml
            │   └── scripts
            │       ├── dummy_coor.py
            │       └── vel_estimate.py
            ├── dot_planner
            │   ├── CMakeLists.txt
            │   ├── config
            │   │   ├── base_local_planner_params.yaml
            │   │   ├── costmap_common_params.yaml
            │   │   ├── costmap_global_laser.yaml
            │   │   ├── costmap_global_static.yaml
            │   │   ├── ekf_localization.yaml
            │   │   ├── local_costmap_params.yaml
            │   │   └── ukf_template.yaml
            │   ├── debugging.odt
            │   ├── launch
            │   │   ├── costmap.launch
            │   │   ├── dot_planner.launch
            │   │   ├── localization.launch
            │   │   └── move_base.launch
            │   ├── navguide.pdf
            │   ├── package.xml
            │   ├── rviz
            │   │   ├── costmap.rviz
            │   │   ├── dot_planner.rviz
            │   │   ├── localization.rviz
            │   │   └── move_base.rviz
            │   └── scripts
            │       └── move.py
            ├── dot_teleop
            │   ├── CMakeLists.txt
            │   ├── package.xml
            │   └── src
            │       ├── dot_joy.cpp
            │       ├── dot_key.cpp
            │       └── dot_teleop.py
            ├── imu_tools
            │   ├── Dockerfile-melodic
            │   ├── Dockerfile-noetic
            │   ├── imu_complementary_filter
            │   │   ├── CHANGELOG.rst
            │   │   ├── CMakeLists.txt
            │   │   ├── include
            │   │   │   └── imu_complementary_filter
            │   │   │       ├── complementary_filter.h
            │   │   │       └── complementary_filter_ros.h
            │   │   ├── launch
            │   │   │   └── complementary_filter.launch
            │   │   ├── package.xml
            │   │   └── src
            │   │       ├── complementary_filter.cpp
            │   │       ├── complementary_filter_node.cpp
            │   │       └── complementary_filter_ros.cpp
            │   ├── imu_filter_madgwick
            │   │   ├── cfg
            │   │   │   └── ImuFilterMadgwick.cfg
            │   │   ├── CHANGELOG.rst
            │   │   ├── CMakeLists.txt
            │   │   ├── imu_filter_nodelet.xml
            │   │   ├── include
            │   │   │   └── imu_filter_madgwick
            │   │   │       ├── imu_filter.h
            │   │   │       ├── imu_filter_nodelet.h
            │   │   │       ├── imu_filter_ros.h
            │   │   │       ├── stateless_orientation.h
            │   │   │       └── world_frame.h
            │   │   ├── launch
            │   │   │   └── imu_filter_madgwick.launch
            │   │   ├── package.xml
            │   │   ├── sample
            │   │   │   ├── ardrone_imu.bag
            │   │   │   ├── phidgets_imu_upside_down.bag
            │   │   │   └── sparkfun_razor.bag
            │   │   ├── src
            │   │   │   ├── imu_filter.cpp
            │   │   │   ├── imu_filter_node.cpp
            │   │   │   ├── imu_filter_nodelet.cpp
            │   │   │   ├── imu_filter_ros.cpp
            │   │   │   └── stateless_orientation.cpp
            │   │   └── test
            │   │       ├── madgwick_test.cpp
            │   │       ├── stateless_orientation_test.cpp
            │   │       └── test_helpers.h
            │   ├── imu_tools
            │   │   ├── CHANGELOG.rst
            │   │   ├── CMakeLists.txt
            │   │   └── package.xml
            │   ├── LICENSE
            │   ├── LICENSE.bsd
            │   ├── LICENSE.gplv3
            │   ├── README.md
            │   └── rviz_imu_plugin
            │       ├── CHANGELOG.rst
            │       ├── CMakeLists.txt
            │       ├── package.xml
            │       ├── plugin_description.xml
            │       ├── rosdoc.yaml
            │       ├── rviz_imu_plugin.png
            │       └── src
            │           ├── imu_acc_visual.cpp
            │           ├── imu_acc_visual.h
            │           ├── imu_axes_visual.cpp
            │           ├── imu_axes_visual.h
            │           ├── imu_display.cpp
            │           ├── imu_display.h
            │           ├── imu_orientation_visual.cpp
            │           └── imu_orientation_visual.h
            ├── README.md
            ├── setup.sh
            └── struct.md

            202 directories, 317 files
