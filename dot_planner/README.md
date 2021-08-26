I created a package relaxed_astar for the A* algorithm

I made a file bot.launch which launches only the bot and saved it in dot_gazebo/launch/

In turtlebot_navigation/launch/includes/move_base.launch.xml , I edited param value in line 16 in order to use relaxed_astar

To launch our bot instead of turtlebot, I changed the launch file in line 47 in turtlebot_stage/launch/turtlebot_in_stage.launch

Now, just use "roslaunch turtlebot_stage turtlebot_in_stage.launch"

Uploaded files:
               turtlebot_stage
               turtlebot_navigation
               turtlebot_bringup
               relaxed_astar
               bot.launch
