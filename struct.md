.
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
│   │       └── diff_control.h
│   ├── launch
│   │   └── bring_up.launch
│   ├── package.xml
│   └── src
│       └── omnidrive.cpp
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
│   │   ├── parameter.launch
│   │   ├── rviz.launch
│   │   └── test.launch
│   ├── package.xml
│   └── world
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
│   ├── base_local_planner_params.yaml
│   ├── CMakeLists.txt
│   ├── costmap_common_params.yaml
│   ├── global_costmap_params.yaml
│   ├── launch
│   │   ├── dot_config.launch
│   │   └── move_base.launch
│   ├── local_costmap_params.yaml
│   └── package.xml
├── dot_teleop
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src
│       ├── dot_joy.cpp
│       ├── dot_key.cpp
│       └── dot_teleop.py
├── README.md
├── setup.sh
└── struct.md

29 directories, 54 files
