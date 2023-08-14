# app-ras-group1
GitHub repository for group 1 of the study field APP-RAS at the Technical University of Berlin.
<img src="https://github.com/ChazzKemal/app-ras-bear-car/assets/71472091/70af0a26-2298-4574-98ce-860a337c2e8e" width="300" alt="Image">



## Bearcar Simulation

### Requirements
To run the simulation following packages are needed:

```
sudo apt install ros-foxy-robot-localization
sudo apt install ros-foxy-navigation2
sudo apt install ros-foxy-nav2-bringup
```

Also, please make sure that your ros distro (foxy) is sourced.

`source /opt/ros/foxy/setup.bash`

On top of that you should pull all the submodules:

`git submodule update --init --recursive`

### Building
For building, execute the following script:

`./build_packages_simulation.sh`

Then, source your environment again:

`source install/setup.sh`

### Starting the Simulation with Object detection and behavioral planner

Start the simulation with slam enabled:

`ros2 launch bearcar_simulation bearcar_sim_v2.py`

Then watch the behavioral planner transition through its states:

`ros2 topic echo bearbrain`

### Starting the Simulation with Gazebo UI and without behavioral planner

In this mode the waypoints wont be set automatically instead one can select the goal in RVIZ with the "Navigation2 goal" button. 

`ros2 launch bearcar_simulation bearcar_sim_v1.py`

Starting the Same with a preloaded mapping:

`ros2 launch bearcar_simulation bearcar_sim_v1.py slam:=False`

Using the preloaded map and without SLAM the robot initial position has to be set first before it can start navigating. The initial position can be set with the "2D Pose Estimate" Button in RVIZ.





## Bearcar (Hardware)
Everything needed for running the software on the real Bearcar 
should be included in this repo. However, we were not able yet to properly integrate
everything, so there are no step-by-step instructions on how to set up everything yet.

## Package Description
| package                 | description                                      | standard component |
|-------------------------|--------------------------------------------------|--------------------|
| bearcar_simulation      | bearcar simulation                               |         ❎          |
| bearbrain               | behavioral planner                               |         ❎          |
| bearcar_urdf            | urdf model of the car                            |         ❎          |
| breye                   | BeaR-EYE image processing node                   |         ❎          |
| breye_interface         | BeaR-EYE message format definitions              |         ❎          |
| diff_to_ack             | mapping between cmd velocity and ackermann drive |        ✅           |
| realsense_ros           | hardware camera driver                           |        ✅           |
| realsense_gazebo_plugin | virtual camera driver for gazebo                 |        ✅           |
| slam_toolbox            | standard slam package                            |        ✅           |
| sllidar_ros2            | hardware lidar driver                            |        ✅           |
| vesc                    | hardware motor controller driver                 |        ✅           |
| navigation2             | standard navigation package                      |        ✅           |


