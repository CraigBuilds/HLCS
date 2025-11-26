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

The project includes unit tests and smoke tests for all three components (sim, driver, gui).

**Prerequisites:**
- ROS2 environment must be sourced: `. /opt/ros/humble/setup.bash`
- Package must be built: `colcon build --packages-select hlcs`
- Package must be sourced: `. install/setup.bash`

1. Install test dependencies:
```bash
pip install pytest pytest-asyncio pytest-mock pytest-timeout psutil
```

2. Run all tests:
```bash
python -m pytest test/ -v
```

3. Run specific test files:
```bash
# Smoke tests only (includes ROS2 node startup tests)
python -m pytest test/test_smoke.py -v

# Tests for simulator
python -m pytest test/test_sim.py -v

# Tests for driver node
python -m pytest test/test_driver.py -v

# Tests for GUI node
python -m pytest test/test_gui.py -v
```

### Continuous Integration

Tests are automatically run on every push and pull request via GitHub Actions using a ROS2 Humble container. The workflow builds the package with colcon and runs the full test suite. See the workflow status in the Actions tab of the repository.
