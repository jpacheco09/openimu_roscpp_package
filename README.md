ROS package used to publish a tf/Imu message acquired from the (serial) OpenIMU335RI VG_AHRS Data Output Message Packet.
This version was tested with a ROS noetic distribution, using an RS232 to USB wire, thus, using serial communication.

## Requirements
The package relies on the ubuntu [_libserial 1.0_](https://github.com/crayzeewulf/libserial) package, that can be acquired via apt:
```bash
sudo apt install libserial-dev
```
for older ubuntu versions it's recommended to manually build using CMake following the [developer instructions](https://github.com/crayzeewulf/libserial?tab=readme-ov-file#building-using-cmake) and run the 
```bash
sudo ldconfig
```
before building the package (see (#159)[https://github.com/crayzeewulf/libserial/issues/159]. The project was built using the [catkin tools](https://catkin-tools.readthedocs.io/en/latest/index.html) toolchain. 

## Usage
It's recommended to clone the repo into your local git folder
```bash
git clone git@github.com:jpacheco09/openimu_roscpp_package.git
```
and then symlink to your catkin_ws/src folder
```bash
ln -s ${GIT_PATH}/openimu_roscpp_package ${catkin_ws}/src/
```
or directly clone the repo on the catkin_ws/src.

Build the project using catkin tools,
```bash
cd ${CATKIN_WS}
catkin build openimu_roscpp_package -DCMAKE_EXPORT_COMPILE_COMMANDS=1
source devel/setup.sh
```
for vscode development, it's recommended to include the `CMAKE_EXPORT_COMPILE_COMMANDS` flag.
Finally, run the project using the following roslaunch instruction
```bash
roslaunch openimu_roscpp_package openimu_attitude_publisher.launch
```
a Rviz window will open. For some reason, the Rviz Imu visualization is not enabled by default. 

### Bonus track:
The node will print the RPY attitude in the command line, however, the Imu message contains only quaternion information, nonetheless one can use the ROS `topic_tools` to extract the attitude data
as a separate topic
```bash
rosrun topic_tools transform /open_imu/data.orientation /euler geometry_msgs/Vector3 'tf.transformations.euler_from_quaternion([m.x, m.y, m.z, m.w])' --import tf
```
and then plot it using `rqt_plot` or simply print the attitude topic in bash using `rostopic echo /euler`

## TODO:
- [ ] Wrap the OpenIMU reading as a separate class:
    - [ ] add support for ping message
    - [ ] add support for sensor configuration 
- [ ] Migrate from `libserial` to standard `serial`
- [ ] Fix `Rviz` topic
- [ ] Add parameter to launch `Rviz` as an option.
- [ ] Fix linting files symlink
- [ ] Add debug build option to provide auxiliar serial readings prints.
