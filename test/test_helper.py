"""Helper utilities for testing HLCS ROS2 nodes."""
import subprocess
import signal
import time
import psutil


class ROS2NodeRunner:
    """Helper class to run and manage ROS2 nodes in tests."""
    
    def __init__(self, package_name, node_name, timeout=5):
        """Initialize the node runner.
        
        Args:
            package_name: Name of the ROS2 package
            node_name: Name of the node to run
            timeout: Timeout in seconds for node startup
        """
        self.package_name = package_name
        self.node_name = node_name
        self.timeout = timeout
        self.process = None
    
    def start(self):
        """Start the ROS2 node."""
        cmd = ['ros2', 'run', self.package_name, self.node_name]
        # preexec_fn ignores SIGINT in child to allow clean shutdown via parent
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
        )
        
        # Give the node time to start
        time.sleep(2)
        
        # Check if process is still running
        if self.process.poll() is not None:
            stdout, stderr = self.process.communicate()
            raise RuntimeError(
                f"Node {self.node_name} failed to start.\n"
                f"stdout: {stdout}\nstderr: {stderr}"
            )
        
        return self.process
    
    def stop(self):
        """Stop the ROS2 node gracefully."""
        if self.process is None:
            return
        
        try:
            # Try graceful shutdown first
            parent = psutil.Process(self.process.pid)
            children = parent.children(recursive=True)
            
            # Send SIGINT (Ctrl+C) for graceful shutdown
            parent.send_signal(signal.SIGINT)
            
            # Wait for process to terminate
            try:
                self.process.wait(timeout=self.timeout)
            except subprocess.TimeoutExpired:
                # If graceful shutdown fails, force kill
                parent.kill()
                for child in children:
                    try:
                        child.kill()
                    except psutil.NoSuchProcess:
                        # Child process already exited; nothing to do.
                        pass
                self.process.wait()
        except psutil.NoSuchProcess:
            # Process already terminated
            pass
        except (subprocess.SubprocessError, OSError):
            # Handle other process-related errors
            try:
                if self.process.poll() is None:
                    self.process.terminate()
                    self.process.wait()
            except Exception:
                pass  # Best effort cleanup
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False
