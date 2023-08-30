Certainly! Let's update the README to accurately describe the publishers your package is using.

---

# README for ODrive_CAN ROS2 Package

## Overview

The `odrive_can` package is a ROS2 package designed to facilitate communication and control with the ODrive motor controller via the CAN protocol. This package provides a lifecycle-managed node to handle these communications. It exposes various ROS2 services for control actions and utilizes ROS2 publishers for real-time data reporting and monitoring.

## Features

- ROS2 Lifecycle management for robust initialization, configuration, and shutdown.
- Communication with ODrive using the CAN protocol.
- ROS2 Services for various control actions like rebooting, clearing errors, setting gains, and setting the axis state.
- ROS2 Publishers for real-time reporting of sensor data like bus voltage, temperature, encoder values, torque, and current (IQ).

## Prerequisites

1. ROS2 (Tested on Humble Hawksbill)
2. ODrive Motor Controller with CAN protocol support

## Installation

Clone this repository into your ROS2 workspace `src/` directory and build using colcon:

```bash
cd ~/ros2_ws/src/
git clone [repository_url] odrive_can
cd ..
colcon build --packages-select odrive_can
```

Source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## How to Use

To launch the ODrive node, execute:

```bash
ros2 run odrive_can odrive_can_node
```

or set up your own composable container look at :

```bash
odrive_can/launch/odrive_composable.launch.py
```


# Topics and Services

## Topics In (Subscribers)

| Topic Name                        | Message Type                        | Description                      | Callback Method                |
|-----------------------------------|-------------------------------------|----------------------------------|--------------------------------|
| `/from_can_bus`                   | `can_msgs::msg::Frame`              | Receives CAN bus frames          | `odrive_node::can_callback`    |
| `frame_id_ + "/target"`           | `odrive_interfaces::msg::Target`    | Receives target position, velocity, and torque | `odrive_node::target_callback`|

## Topics Out (Publishers)

| Topic Name                           | Message Type                               | Description                              |
|--------------------------------------|--------------------------------------------|------------------------------------------|
| `frame_id_ + "/bus_voltage"`         | `odrive_interfaces::msg::BusVoltage`       | Publishes bus voltage data               |
| `/to_can_bus`                        | `can_msgs::msg::Frame`                     | Publishes CAN bus frames                 |
| `frame_id_ + "/temperature"`         | `odrive_interfaces::msg::Temperature`      | Publishes temperature data               |
| `frame_id_ + "/sensor"`              | `odrive_interfaces::msg::OdriveSensor`     | Publishes sensor data including encoder, iq, and torque |


## Services

| Service Name                        | Service Type                          | Description                             |
|-------------------------------------|---------------------------------------|-----------------------------------------|
| `frame_id_ + "/reboot"`             | `odrive_interfaces::srv::Reboot`      | Reboots the ODrive system               |
| `frame_id_ + "/clear_errors"`       | `odrive_interfaces::srv::ClearErrors` | Clears ODrive errors                    |
| `frame_id_ + "/set_pos_gain"`       | `odrive_interfaces::srv::SetPosGain`  | Sets position gains for control loop    |
| `frame_id_ + "/set_vel_gains"`      | `odrive_interfaces::srv::SetVelGains` | Sets velocity gains for control loop    |
| `frame_id_ + "/set_axis_state"`     | `odrive_interfaces::srv::SetAxisState`| Sets the state for a specific axis     |


## Parameters

| Parameter Name          | Default Value | Description                                                         |
|-------------------------|--------------|---------------------------------------------------------------------|
| `node_id`               | `0`          | Node identifier. Used to generate `frame_id` as "odrive"+`node_id`   |
| `control_mode`          | `0x2`        | Mode of control                                                     |
| `input_mode`            | `0x2`        | Mode of input                                                       |
| `velocity_limit`        | `25.0`       | Velocity limit                                                      |
| `current_limit`         | `10.0`       | Current limit                                                       |
| `traj_vel_limit`        | `25.0`       | Trajectory velocity limit                                           |
| `traj_accel_limit`      | `25.0`       | Trajectory acceleration limit                                       |
| `traj_deaccel_limit`    | `25.0`       | Trajectory deacceleration limit                                     |
| `traj_inertia`          | `25.0`       | Trajectory inertia                                                  |
| `gear_ratio`            | `20.0`       | Gear ratio                                                          |


## Extended Message Types Information Table

| Message Type                         | Direction  | Description                                                  | Fields                                                                 |
|--------------------------------------|------------|--------------------------------------------------------------|------------------------------------------------------------------------|
| `odrive_interfaces::msg::BusVoltage` | Outgoing   | Represents the bus voltage and current information           | `std_msgs/Header header`, `std_msgs/Float32 bus_volt`, `std_msgs/Float32 bus_current`                   |
| `odrive_interfaces::msg::Temperature`| Outgoing   | Represents the temperature data                              | `std_msgs/Header header`, `std_msgs/Float32 fet_temp`, `std_msgs/Float32 motor_temp` |
| `odrive_interfaces::msg::OdriveSensor`| Outgoing  | Includes encoder, iq, and torque data                        | `odrive_interfaces/Torque torque`, `odrive_interfaces/Encoder encoder`, `odrive_interfaces/Iq iq`|
| `can_msgs::msg::Frame`               | Incoming/Outgoing | Represents a CAN bus frame                              | Standard CAN fields                                                      |
| `odrive_interfaces::msg::Target`     | Incoming   | Represents target control instructions                      | `std_msgs/Header header`, `std_msgs/Float32 pos_des`, `std_msgs/Float32 vel_des`, `std_msgs/Float32 torque_des`|

### Sub-messages:

- `odrive_interfaces::msg::Encoder`
  - `std_msgs/Header header`
  - `std_msgs/Float32 pos`
  - `std_msgs/Float32 vel`

- `odrive_interfaces::msg::Iq`
  - `std_msgs/Header header`
  - `std_msgs/Float32 iq_des`
  - `std_msgs/Float32 iq_est`

- `odrive_interfaces::msg::Torque`
  - `std_msgs/Header header`
  - `std_msgs/Float32 torque_des`
  - `std_msgs/Float32 torque_est`

### Note:
- "Outgoing" means the message is published by this node.
- "Incoming" means the message is subscribed to by this node.

## Contributing

Contributions are welcome through pull requests. If you encounter any issues or have suggestions, please open an issue on the repository.

## License

This package is distributed under [LICENSE_NAME].

---

Feel free to edit this README to better fit your package's specific functionalities.
