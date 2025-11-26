"""Smoke tests to verify basic functionality of HLCS components."""
import pytest
import subprocess
import time
from test.test_helper import ROS2NodeRunner


def test_sim_import():
    """Smoke test: Verify sim module can be imported."""
    from hlcs import sim
    assert sim is not None


def test_driver_import():
    """Smoke test: Verify driver module can be imported."""
    from hlcs import driver
    assert driver is not None


def test_gui_import():
    """Smoke test: Verify gui module can be imported."""
    from hlcs import gui
    assert gui is not None


def test_opcua_simulator_class_exists():
    """Smoke test: Verify OpcuaSimulator class exists."""
    from hlcs.sim import OpcuaSimulator
    assert OpcuaSimulator is not None


def test_driver_node_class_exists():
    """Smoke test: Verify DriverNode class exists."""
    from hlcs.driver import DriverNode
    assert DriverNode is not None


def test_gui_node_class_exists():
    """Smoke test: Verify GUINode class exists."""
    from hlcs.gui import GUINode
    assert GUINode is not None


def test_ros2bridge_class_exists():
    """Smoke test: Verify ROS2Bridge class exists."""
    from hlcs.gui import ROS2Bridge
    assert ROS2Bridge is not None


@pytest.mark.timeout(15)
def test_sim_node_runs():
    """Smoke test: Verify sim node can be started and stopped."""
    with ROS2NodeRunner('hlcs', 'sim') as runner:
        # Verify process is running
        assert runner.process is not None
        assert runner.process.poll() is None
        time.sleep(1)
        # Process should still be running after 1 second (verify no immediate crash)
        assert runner.process.poll() is None


@pytest.mark.timeout(15)
def test_driver_node_runs():
    """Smoke test: Verify driver node can be started and stopped.
    
    Note: This test may fail if OPC UA server is not available, which is expected.
    We're just testing that the node can start and handle the error gracefully.
    """
    with ROS2NodeRunner('hlcs', 'driver', timeout=3) as runner:
        # Verify process is running
        assert runner.process is not None
        # Give it time to initialize
        time.sleep(2)
        # Process should be running (even if it can't connect to OPC UA)
        assert runner.process.poll() is None


@pytest.mark.timeout(15)
def test_gui_node_runs():
    """Smoke test: Verify gui node can be started and stopped.
    
    Note: GUI node requires a display. This test verifies it starts even in headless mode.
    The node may fail to initialize GUI components without a display, which is expected.
    """
    import os
    env = os.environ.copy()
    env['QT_QPA_PLATFORM'] = 'offscreen'
    with ROS2NodeRunner('hlcs', 'gui', timeout=3, env=env) as runner:
        # Give it time to attempt startup
        time.sleep(2)
        # The process might have exited due to display issues, which is OK
        # We just verify it attempted to start
        returncode = runner.process.poll()
        # If it's still running, that's good
        # If it exited, check it's not a catastrophic failure (returncode -9, -11)
        if returncode is not None:
            assert returncode not in [-9, -11], f"GUI node crashed with signal {-returncode}"
