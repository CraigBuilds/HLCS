# HLCS
High Level Control System

A ROS2 package that integrates OPC UA communication with a Qt/QML-based GUI.

## Components

- **sim**: OPC UA simulator server (not a ROS2 node, standalone application)
- **driver**: ROS2 node that bridges OPC UA server to ROS2 topics and services
- **gui**: ROS2 node with PySide6/QML interface for visualization and control

## Dependencies

- ROS2 (Humble or later recommended)
- Python 3.8+
- asyncua
- PySide6

Install Python dependencies:
```bash
pip install -r requirements.txt
```

## Building

```bash
colcon build --packages-select hlcs
source install/setup.bash
```

## Usage

1. Start the OPC UA simulator:
```bash
ros2 run hlcs sim
```

2. In another terminal, start the driver node:
```bash
ros2 run hlcs driver
```

3. In another terminal, start the GUI:
```bash
ros2 run hlcs gui
```

## Topics

- `hlcs/data` (std_msgs/Float64): Live data from OPC UA server
- `hlcs/counter` (std_msgs/Int32): Counter value from OPC UA server

## Services

- `hlcs/increment_counter` (std_srvs/Trigger): Increment the counter
- `hlcs/reset_counter` (std_srvs/Trigger): Reset the counter to 0

## Testing

### Running Tests Locally

Prerequisites:
- Source ROS2 environment: `source /opt/ros/humble/setup.bash`
- Build package: `colcon build --packages-select hlcs`
- Source install: `source install/setup.bash`

Install dependencies and run tests:
```bash
pip install -r requirements.txt
python -m pytest test/ -v
```

### Continuous Integration

Tests run automatically on push/pull requests using GitHub Actions with a ROS2 Humble container.
