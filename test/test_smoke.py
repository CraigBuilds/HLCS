"""Smoke tests to verify basic functionality of HLCS components."""
import pytest


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
