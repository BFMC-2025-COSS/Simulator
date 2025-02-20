# How to Run
## purepursuit_ws
### node_to_path
```bash
cd ~/BFMC/Simulator/purepursuit_ws
source devel/setup.bash

roslaunch control node_to_path.launch
```

### pure_pursuit
```bash
cd ~/BFMC/Simulator/purepursuit_ws
source devel/setup.bash

rosrun control pure_pursuit.py
```
you can visualize current position, look ahead point, look ahead line in `Rviz`
### if you want to change global path
Edit `config/global_path_key_nodes.yaml`  
you must write nodes below  
1. start node
2. nodes that you want to visit
3. end node

## test_env
### Gazebo
```bash
cd ~/BFMC/Simulator/test_env
source devel/setup.bash

roslaunch sim_pkg pure_pursuit_mode.launch
```

### if you want to GPS noise
Edit `plugins_pkgs/gps_plugin/src/gps_plugin.cpp`
```cpp
this->m_gps_pose.posA   = this->m_model->RelativePose().Pos().X();// + (rand() / (float)RAND_MAX * 0.2) - 0.1;
this->m_gps_pose.posB   = abs(this->m_model->RelativePose().Pos().Y());// + (rand() / (float)RAND_MAX * 0.2) - 0.1;
```
and `catkin_make`