# move_box_gazebo_ros

1. Edit bashrc
```
gedit ~/.bashrc
```
```
export GAZEBO_PLUGIN_PATH=$HOME/catkin_ws/build/move_box_gazebo_ros:$GAZEBO_PLUGIN_PATH
```
2. Launch test world
```
roslaunch move_box_gazebo_ros move_box_gazebo_ros.launch
```

3. rostopic pub
```
rostopic pub /box_weebee/cmd std_msgs/Float32MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [0,0,0,0,0,0]"  -r 10
```

4. Source bashrc
```
source ~/.bashrc
```

- gedit move_box.world
```
<plugin name="move_box" filename="libmove_box_gazebo_ros.so">
  <bodyName>box</bodyName>
  <xyzVelLimit>5 5 5</xyzVelLimit>
  <debug>false</debug>
  <!-- Set to false if not using ROS -->
  <topicNs>box_weebee</topicNs> <!-- Float32MultiArray, xyzrpy -->
</plugin>
```
