# How to Run: Simulation
## control_ws
```bash
cd ~/BFMC/Simulator/control_ws
source devel/setup.bash

roslaunch control control.launch
```

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
# TODO: Real car
- You can edit topic names in `control.launch`
- You can edit `look ahead distance` and `wheel_base` in `control.launch`
