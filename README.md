# Swarm-Bot
Ros repository for the swarm bots

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

|
---dot_gazebo ## simulation
    |
    - launch 
        |
        -- gazebo.launch # physics simulation
        |
        -- rviz.launch # visualization of robot
    - world # world elements
|
---dot_description ## robot urdf
    |
    --urdf
        |
        -- bot.urdf.xacro # constains bot urdf
|
---dot_config
    |
    --config/control
        |
        bot_control.yaml # pid gains
|
---dot_control
    |
    src
    |
    ---diffdrive.cpp ## Vishwas and Prateek make it omni drive 
        [twist(desired_linear_velocity, desired_angular_velocity) -> topic(/cmd_vel) => joint_torque]
|
---dot_planner
    |
    config
        |
        -- costmap.yaml
        |
        -- local_planner.yaml ## teb_local_planner ## navigation lidar using overhead-camera ## Raghav, Karthik, Pranav and Varad
        |
        -- global_planner.yaml ## A* Yash, Yatharth and Aryaman and dhruv
    |
    include
        |
        dot_planner
            |
            swarm_planning.h ## M*
    |
    scripts
        |
        dot_planner
            |
            M_star.py
    
|
---dot_hardware
    |
    src/
        |


