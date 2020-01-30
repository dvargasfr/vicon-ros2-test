# vicon-ros2-test

This repo is created as a fast prototype for testing a Vicon-ROS2 system. It is being used while the development of the [MOCAP4ROS2](https://github.com/IntelligentRoboticsLabs/MOCAP4ROS2) project.

Its main purpose is to analize several QoS features working on diferent OS and with different DDS.

## Fast_RTPS

- Linux: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- Windows: `set RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

## OpenSplice

- Linux: `export RMW_IMPLEMENTATION=rmw_opensplice_cpp`
- Windows: `set RMW_IMPLEMENTATION=rmw_opensplice_cpp`

## Cyclone

- Linux: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Windows: `set RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Cyclone DDS allows you to configure some features on a .xml file.

Check the [cyclonedds.xml](https://github.com/dvargasfr/vicon-ros2-test/blob/master/linux/config/cyclonedds.xml) file. See more [here](https://github.com/eclipse-cyclonedds/cyclonedds/blob/master/cdds.md).

You need to set the `CYCLONEDDS_URI` environment variable.

- Linux: `export CYCLONEDDS_URI=file://{path/to/xml/file}`
- Windows: `set CYCLONEDDS_URI=file://{path/to/xml/file}`

# Get some ROS2 topics metrics

To get some metrics of a given topic, e.g.: `/my_custom_topic` :
- Bandwidth: `ros2 topic bw /my_custom_topic`
- Rate: `ros2 topic hz /my_custom_topic`
- Delay: `ros2 topic delay /my_custom_topic`
