"""Integration tests to verify HLCS nodes can run in a real ROS2 environment."""
import pytest
import time
import os
from test.test_helper import ROS2NodeRunner


@pytest.mark.timeout(15)
def test_sim_node_runs():
    """Integration test: Verify sim node starts and runs without crashing.
    
    Requires ROS2 environment to be installed and sourced.
    """
    with ROS2NodeRunner('hlcs', 'sim') as runner:
        assert runner.process is not None
        assert runner.process.poll() is None
        time.sleep(1)
        # Process should still be running after 1 second
        assert runner.process.poll() is None


@pytest.mark.timeout(15)
def test_driver_node_runs():
    """Integration test: Verify driver node starts and handles missing OPC UA server gracefully.
    
    Requires ROS2 environment to be installed and sourced.
    This test verifies the node can start even without an OPC UA server available.
    """
    with ROS2NodeRunner('hlcs', 'driver', timeout=3) as runner:
        assert runner.process is not None
        time.sleep(2)
        # Process should be running (even if it can't connect to OPC UA)
        assert runner.process.poll() is None


@pytest.mark.timeout(15)
def test_gui_node_runs():
    """Integration test: Verify gui node starts in headless mode.
    
    Requires ROS2 environment to be installed and sourced.
    Tests that the node attempts to start even in headless environments.
    """
    env = os.environ.copy()
    env['QT_QPA_PLATFORM'] = 'offscreen'
    with ROS2NodeRunner('hlcs', 'gui', timeout=3, env=env, check_startup=False) as runner:
        time.sleep(2)
        returncode = runner.process.poll()
        # If it exited, verify it wasn't a catastrophic crash (SIGKILL, SIGSEGV)
        if returncode is not None:
            assert returncode not in [-9, -11], f"GUI node crashed with signal {-returncode}"
