# cmu_16833_SLAM_final_project
## How to launch the simulation
Note: Please copy the src files to local pc, and do not build or make within this git repo
1. First git clone the repo to your local pc.
2. Build Gazebo turtlebot3 simulation: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
3. copy and paste "turtlebot3_custom_world.launch" to /../turtlebot3_gazebo/launch/
4. copy and paste "turtlebot3_custom_world.world" to /../turtlebot3_gazebo/world/
5. Build box animation
6. copy and paste "box_animation" to your local pc.
  ```
  $ cd to /..../box_animation/
  $ mkdir build
  $ cd build
  $ cmake ../
 ```
7. in /../build/ ,  copy /THE/PATH/TO/libanimated_box.so to line 427 in "turtlebot3_custom_world.world". (plugin name= ..... filename = "/THE/PATH/TO/libanimated_box.so"/)
