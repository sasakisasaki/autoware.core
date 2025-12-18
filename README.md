# autoware_core

- An [Autoware](https://github.com/autowarefoundation/autoware) repository that contains a basic set of high-quality, stable ROS packages for autonomous driving.

- Although this repository is currently empty, porting of code from Universe to Core will begin once the interfaces for Autoware Core/Universe have been finalized, as per ongoing [Autoware Architecture WG](https://github.com/autowarefoundation/autoware/discussions?discussions_q=label%3Aarchitecture_wg) discussions.
- A more detailed explanation about Autoware Core can be found on the [Autoware concepts documentation page](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-concepts/#the-core-module).

- For researchers and developers who want to extend the functionality of Autoware Core with experimental, cutting-edge ROS packages, see [Autoware Universe](https://github.com/autowarefoundation/autoware_universe).

## Planning Simulation

We can run [the planning simulation](https://autowarefoundation.github.io/autoware-documentation/main/demos/planning-sim/) based on the `autoware_core`.

### Build Dependencies

```
# If you have not cloned `autoware_core` yet
$ git clone https://github.com/autowarefoundation/autoware_core.git

$ cd autoware_core
$ mkdir src && vcs import src < autoware_core.repos    # TO BE UPDATED
$ rosdep install -y --from-paths ./src --ignore-src --rosdistro $ROS_DISTRO
$ colcon build --base-paths ./src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
$ git clone https://github.com/autowarefoundation/autoware_universe.git
$ colcon build --base-paths ./ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to autoware_simple_planning_simulator autoware_raw_vehicle_cmd_converter autoware_dummy_perception_publisher
```

### Launch Planning Simulation
```
$ source install/setup.bash
$ ros2 launch autoware_core autoware_core.launch.xml \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit \
    map_path:=PATH_TO_YOUR_MAP \
    launch_sensing:=false \
    launch_sensing_driver:=false \
    launch_perception:=false \
    is_planning_simulation:=true
```
