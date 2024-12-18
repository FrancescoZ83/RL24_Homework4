# Mobile Robot navigation

## :hammer: Setup

To install the packages run from the ros2 workspace:
```sh
colcon build
```
You will need 4 terminals open, be sure to run this on all of them:
```sh
source install/setup.sh
```

## 🧭 Autonomous navigation task

Launching Gazebo simulation
```sh
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

Launching exploration (SLAM and Nav2)
```sh
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

Running the follow_waypoints.py script (wait a few seconds after launching explore)
```sh
ros2 run rl_fra2mo_description follow_waypoints.py
```

## 🗺️ Mapping the environment

Launching Gazebo simulation
```sh
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

Launching exploration (SLAM and Nav2)
```sh
ros2 launch rl_fra2mo_description fra2mo_explore.launch.py
```

Running the follow_map_points.py script (wait a few seconds after launching explore)
```sh
ros2 run rl_fra2mo_description follow_map_points.py
```

Launching Rviz in the correct configuration to see the mapping process
```sh
ros2 launch rl_fra2mo_description display_fra2mo.launch.py
```

## 👁️ Vision-based task

Launching Gazebo simulation
```sh
ros2 launch rl_fra2mo_description gazebo_fra2mo.launch.py
```

Launching exploration (SLAM and Nav2) and the aruco detector
```sh
ros2 launch rl_fra2mo_description fra2mo_vision.launch.py
```

Running the node that publishes the aruco pose once detected
```sh
ros2 run rl_fra2mo_description fra2mo_pose_xy_node
```

Display /tf_static
```sh
ros2 topic echo /tf_static
```
