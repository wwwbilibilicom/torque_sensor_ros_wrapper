# Torque Sensor ROS 2 Driver

A ROS 2 driver package for reading data from a serial torque sensor. This wrapper integrates the underlying C++ serial driver into the ROS 2 ecosystem, publishing standard wrench messages with robust connection handling.

![ROS 2](https://img.shields.io/badge/ROS2-Humble%2B-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Build](https://img.shields.io/badge/Build-Colcon-orange)

## Features

- **Standard Interface**: Publishes `geometry_msgs/msg/WrenchStamped` messages.
- **Auto-Reconnection**: Automatically attempts to reconnect if the serial device is disconnected or the port becomes unavailable.
- **Configurable**: Fully parameterizable (Baudrate, Port, Sensor Range, Voltage Mapping) via launch files or CLI.
- **High Performance**: Optimized polling loop to ensure low latency data acquisition.

## Prerequisites

- **OS**: Linux (Ubuntu 22.04 / 24.04 recommended)
- **ROS 2**: Humble / Iron / Jazzy
- **Hardware**: A supported serial torque sensor (connected via USB-Serial adapter).

## Installation

1.  **Clone the repository** (ensure you get submodules):
    ```bash
    cd ~/ros2_ws/src
    git clone --recursive <your-repo-url>
    ```
    *If you already cloned without submodules:*
    ```bash
    git submodule update --init --recursive
    ```

2.  **Install dependencies**:
    ```bash
    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y
    ```

3.  **Build**:
    ```bash
    colcon build --packages-select torque_sensor_ros
    ```

4.  **Source the workspace**:
    ```bash
    source install/setup.bash
    ```

## Usage

### 🚀 Using the Launch File (Recommended)

The easiest way to run the driver is using the provided launch file, which allows you to configure arguments easily.

```bash
ros2 launch torque_sensor_ros torque_sensor.launch.py port_name:=/dev/ttyUSB0 baudrate:=256000
```

**Common Arguments:**

| Argument | Default | Description |
| :--- | :--- | :--- |
| `port_name` | `/dev/ttyUSB0` | Path to the serial device. |
| `baudrate` | `256000` | Serial communication speed. |
| `sensor_type` | `RANGE_30NM` | Sensor range type (`RANGE_30NM` or `RANGE_100NM`). |
| `publish_rate` | `100.0` | Frequency (Hz) to publish data. |


### 🐢 Running the Node Directly

You can also run the node directly using `ros2 run`:

```bash
ros2 run torque_sensor_ros torque_sensor_ros_node --ros-args -p port_name:=/dev/ttyUSB0 -p sensor_type:=RANGE_100NM
```

## Nodes & Topics

### Node: `torque_sensor_node`

#### Published Topics

| Topic | Type | Description |
| :--- | :--- | :--- |
| `/torque` | `geometry_msgs/msg/WrenchStamped` | The torque data. Value is in `wrench.torque.z`. |

#### Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- |
| `port_name` | string | `/dev/ttyUSB0` | Serial port device path. |
| `baudrate` | int | `256000` | Baud rate for connection. |
| `frame_id` | string | `torque_sensor_link` | Frame ID for the stamped message header. |
| `publish_rate` | double | `100.0` | Main loop rate in Hz. |
| `sensor_type` | string | `RANGE_30NM` | `RANGE_30NM` or `RANGE_100NM`. |
| `zero_voltage` | float | `5.0` | Voltage reading corresponding to 0 Nm. |
| `max_voltage` | float | `10.0` | Voltage reading corresponding to max range. |


## Data Interpretation

The driver maps analog voltage readings to torque based on the configured parameters.
The equation used is:

$$ 
\text{Torque} = (\text{CurrentVoltage} - \text{ZeroVoltage}) \times \frac{\text{MaxTorque}}{\text{MaxVoltage} - \text{ZeroVoltage}} 
$$

Ensure your `zero_voltage` and `max_voltage` parameters match your sensor's calibration.

## Troubleshooting

- **Permission Denied**: Ensure your user is in the `dialout` group to access serial ports:
  ```bash
  sudo usermod -a -G dialout $USER
  # Log out and log back in
  ```
- **Connection Failed**: Verify the port path using `ls -l /dev/serial/by-id/` to find the persistent name of your USB adapter.

## License

MIT License. See `LICENSE` file for details.
