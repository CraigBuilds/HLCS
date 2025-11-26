"""Tests for ROS2NodeRunner helper class."""
import pytest
import os
import time
from test.test_helper import ROS2NodeRunner


def test_node_runner_init():
    """Test ROS2NodeRunner initialization."""
    runner = ROS2NodeRunner('test_pkg', 'test_node', timeout=10)
    assert runner.package_name == 'test_pkg'
    assert runner.node_name == 'test_node'
    assert runner.timeout == 10
    assert runner.process is None
    assert runner.env is not None
    assert isinstance(runner.env, dict)


def test_node_runner_init_with_env():
    """Test ROS2NodeRunner initialization with custom environment."""
    custom_env = os.environ.copy()
    custom_env['TEST_VAR'] = 'test_value'
    runner = ROS2NodeRunner('test_pkg', 'test_node', env=custom_env)
    assert runner.env == custom_env
    assert 'TEST_VAR' in runner.env
    assert runner.env['TEST_VAR'] == 'test_value'


def test_is_running_before_start():
    """Test is_running returns False before starting."""
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    assert runner.is_running() is False


def test_get_return_code_before_start():
    """Test get_return_code returns None before starting."""
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    assert runner.get_return_code() is None


def test_get_output_before_start():
    """Test get_output returns empty strings before starting."""
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    stdout, stderr = runner.get_output()
    assert stdout == ''
    assert stderr == ''


def test_wait_for_output_invalid_stream():
    """Test wait_for_output raises ValueError for invalid stream."""
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    with pytest.raises(ValueError, match="stream must be"):
        runner.wait_for_output('pattern', stream='invalid')


def test_node_runner_context_manager():
    """Test ROS2NodeRunner can be used as context manager."""
    # This test doesn't actually start a node (ROS2 not available)
    # but verifies the context manager structure is correct
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    assert hasattr(runner, '__enter__')
    assert hasattr(runner, '__exit__')


# Integration tests with actual processes
def test_is_running_with_real_process():
    """Test is_running with a real process (using 'sleep' command)."""
    import subprocess
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    # Start a simple sleep process instead of a ROS2 node
    runner.process = subprocess.Popen(
        ['sleep', '10'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    # Start reader threads manually since we bypassed start()
    import threading
    runner._stdout_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stdout, runner._stdout_queue),
        daemon=True
    )
    runner._stderr_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stderr, runner._stderr_queue),
        daemon=True
    )
    runner._stdout_thread.start()
    runner._stderr_thread.start()
    
    try:
        assert runner.is_running() is True
        runner.process.terminate()
        runner.process.wait(timeout=2)
        assert runner.is_running() is False
    finally:
        try:
            runner.process.kill()
            runner.process.wait(timeout=1)
        except Exception:
            pass


def test_get_return_code_with_real_process():
    """Test get_return_code with a real process."""
    import subprocess
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    # Start a process that exits immediately with code 0
    runner.process = subprocess.Popen(
        ['echo', 'test'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    # Start reader threads manually since we bypassed start()
    import threading
    runner._stdout_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stdout, runner._stdout_queue),
        daemon=True
    )
    runner._stderr_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stderr, runner._stderr_queue),
        daemon=True
    )
    runner._stdout_thread.start()
    runner._stderr_thread.start()
    
    runner.process.wait(timeout=2)
    assert runner.get_return_code() == 0


def test_get_output_with_real_process():
    """Test get_output captures stdout and stderr."""
    import subprocess
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    # Start a process that outputs to stdout
    runner.process = subprocess.Popen(
        ['echo', 'hello world'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    # Start reader threads manually since we bypassed start()
    import threading
    runner._stdout_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stdout, runner._stdout_queue),
        daemon=True
    )
    runner._stderr_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stderr, runner._stderr_queue),
        daemon=True
    )
    runner._stdout_thread.start()
    runner._stderr_thread.start()
    
    runner.process.wait(timeout=2)
    time.sleep(0.2)  # Give time for output buffering and thread to read
    stdout, stderr = runner.get_output(timeout=0.1)
    assert 'hello world' in stdout


def test_wait_for_output_with_real_process():
    """Test wait_for_output finds pattern in output."""
    import subprocess
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    # Start a process that outputs to stdout
    runner.process = subprocess.Popen(
        ['bash', '-c', 'echo "starting"; sleep 0.5; echo "ready"; sleep 5'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    # Start reader threads manually since we bypassed start()
    import threading
    runner._stdout_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stdout, runner._stdout_queue),
        daemon=True
    )
    runner._stderr_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stderr, runner._stderr_queue),
        daemon=True
    )
    runner._stdout_thread.start()
    runner._stderr_thread.start()
    
    try:
        # Wait for "ready" to appear in stdout
        found = runner.wait_for_output('ready', timeout=3, stream='stdout')
        assert found is True
    finally:
        try:
            runner.process.kill()
            runner.process.wait(timeout=1)
        except Exception:
            pass


def test_wait_for_output_timeout():
    """Test wait_for_output returns False on timeout."""
    import subprocess
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    # Start a process that doesn't output the expected pattern
    runner.process = subprocess.Popen(
        ['sleep', '10'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    # Start reader threads manually since we bypassed start()
    import threading
    runner._stdout_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stdout, runner._stdout_queue),
        daemon=True
    )
    runner._stderr_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stderr, runner._stderr_queue),
        daemon=True
    )
    runner._stdout_thread.start()
    runner._stderr_thread.start()
    
    try:
        # Wait for a pattern that won't appear
        found = runner.wait_for_output('nonexistent', timeout=1, stream='stdout')
        assert found is False
    finally:
        try:
            runner.process.kill()
            runner.process.wait(timeout=1)
        except Exception:
            pass


def test_stop_collects_output():
    """Test that stop method collects remaining output."""
    import subprocess
    runner = ROS2NodeRunner('test_pkg', 'test_node')
    # Start a process that outputs data
    runner.process = subprocess.Popen(
        ['echo', 'final output'],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )
    # Start reader threads manually since we bypassed start()
    import threading
    runner._stdout_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stdout, runner._stdout_queue),
        daemon=True
    )
    runner._stderr_thread = threading.Thread(
        target=runner._read_stream,
        args=(runner.process.stderr, runner._stderr_queue),
        daemon=True
    )
    runner._stdout_thread.start()
    runner._stderr_thread.start()
    
    time.sleep(0.3)  # Let process complete and threads read output
    runner.stop()
    
    stdout, stderr = runner.get_output()
    assert 'final output' in stdout
