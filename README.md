# Swarm-Bot
Ros repository for the swarm bots Simulations

For Hardware code visit [Swarmbot-Hardware](https://github.com/Robotics-Club-IIT-BHU/Swarm-Bot-Hardware)

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


            ├── dot_config
            │   ├── CMakeLists.txt
            │   ├── config
            │   │   └── control
            │   │       └── bot_control.yaml            # pid gains
            │   └── package.xml
            ├── dot_control
            │   ├── CMakeLists.txt
            │   ├── include
            │   │   └── dot_control
            │   │       └── diff_control.h
            │   ├── launch
            │   │   └── bring_up.launch
            │   ├── package.xml
            │   └── src
            │       └── omnidrive.cpp                   ## [twist(desired_linear_velocity, desired_angular_velocity) -> topic(/cmd_vel) => joint_torque]
            ├── dot_description                         ## robot urdf
            │   ├── CMakeLists.txt
            │   ├── mesh
            │   │   ├── base.stl
            │   │   ├── rim.stl
            │   │   └── roller.stl
            │   ├── package.xml
            │   ├── urdf
            │   │   └── bot.urdf.xacro                  # constains bot urdf
            │   └── xacro
            │       ├── main.urdf.xacro
            │       ├── rim.urdf.xacro
            │       └── roller.urdf.xacro
            ├── dot_gazebo                              ## simulation
            │   ├── CMakeLists.txt
            │   ├── launch
            │   │   ├── gazebo.launch                   # physics simulation
            │   │   ├── parameter.launch
            │   │   ├── rviz.launch                     # visualization of robot
            │   │   └── test.launch
            │   ├── package.xml
            │   └── world                               # world elements
            │       ├── ps1
            │       │   ├── dummy_.world
            │       │   ├── final_ps.world
            │       │   ├── my_ground_plane
            │       │   │   ├── materials
            │       │   │   │   ├── scripts
            │       │   │   │   │   └── my_ground_plane.material
            │       │   │   │   └── textures
            │       │   │   │       └── MyImage.png
            │       │   │   ├── model.config
            │       │   │   └── model.sdf
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
            ├── dot_planner
            │   ├── include 
            │   │   └── dot_planner
            │   │        └── swarm_plannign.h               ## M*
            │   ├── scripts
            │   │   └── dot_planner
            │   │        └── M_star.py                      ## M*   
            │   ├── base_local_planner_params.yaml
            │   ├── CMakeLists.txt
            │   ├── costmap_common_params.yaml
            │   ├── global_costmap_params.yaml              ## A* Yash, Yatharth and Aryaman and dhruv
            │   ├── launch
            │   │   ├── dot_config.launch
            │   │   └── move_base.launch
            │   ├── local_costmap_params.yaml               ## teb_local_planner ## navigation lidar using overhead-camera ## Raghav, Karthik, Pranav and Varad
            │   └── package.xml
            ├── dot_teleop
            │   ├── CMakeLists.txt
            │   ├── package.xml
            │   └── src
            │       ├── dot_joy.cpp                         ## teleop using joystick
            │       ├── dot_key.cpp                         ## teleop using keyboard
            │       └── dot_teleop.py
            ├── README.md
            └── setup.sh

29 directories, 54 files

to be added 
|
---dot_hardware
    |
    src/
        |


