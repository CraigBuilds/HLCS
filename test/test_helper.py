"""Helper utilities for testing HLCS ROS2 nodes."""
import subprocess
import signal
import time
import psutil
import os
import threading
import queue


class ROS2NodeRunner:
    """Helper class to run and manage ROS2 nodes in tests."""
    
    def __init__(self, package_name, node_name, timeout=5, env=None):
        """Initialize the node runner.
        
        Args:
            package_name: Name of the ROS2 package
            node_name: Name of the node to run
            timeout: Timeout in seconds for node startup
            env: Optional dictionary of environment variables to set for the process
        """
        self.package_name = package_name
        self.node_name = node_name
        self.timeout = timeout
        self.env = env if env is not None else os.environ.copy()
        self.process = None
        self._stdout_data = []
        self._stderr_data = []
        self._stdout_queue = queue.Queue()
        self._stderr_queue = queue.Queue()
        self._stdout_thread = None
        self._stderr_thread = None
    
    def _read_stream(self, stream, output_queue):
        """Read from a stream and put lines into a queue.
        
        Args:
            stream: The stream to read from (stdout or stderr)
            output_queue: Queue to put the lines into
        """
        try:
            for line in iter(stream.readline, ''):
                if line:
                    output_queue.put(line)
        except Exception:
            # Stream closed or other error
            pass
    
    def start(self):
        """Start the ROS2 node."""
        cmd = ['ros2', 'run', self.package_name, self.node_name]
        # preexec_fn ignores SIGINT in child to allow clean shutdown via parent
        self.process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            env=self.env,
            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
        )
        
        # Start threads to read stdout and stderr
        self._stdout_thread = threading.Thread(
            target=self._read_stream,
            args=(self.process.stdout, self._stdout_queue),
            daemon=True
        )
        self._stderr_thread = threading.Thread(
            target=self._read_stream,
            args=(self.process.stderr, self._stderr_queue),
            daemon=True
        )
        self._stdout_thread.start()
        self._stderr_thread.start()
        
        # Give the node time to start
        time.sleep(2)
        
        # Check if process is still running
        if self.process.poll() is not None:
            # Process terminated, collect any output
            self._collect_output()
            stdout = ''.join(self._stdout_data)
            stderr = ''.join(self._stderr_data)
            raise RuntimeError(
                f"Node {self.node_name} failed to start.\n"
                f"stdout: {stdout}\nstderr: {stderr}"
            )
        
        return self.process
    
    def _collect_output(self):
        """Collect all available output from queues."""
        while not self._stdout_queue.empty():
            try:
                self._stdout_data.append(self._stdout_queue.get_nowait())
            except queue.Empty:
                break
        while not self._stderr_queue.empty():
            try:
                self._stderr_data.append(self._stderr_queue.get_nowait())
            except queue.Empty:
                break
    
    def stop(self):
        """Stop the ROS2 node gracefully."""
        if self.process is None:
            return
        
        try:
            # Collect any remaining output before stopping
            self._collect_output()
            
            if self.process.poll() is not None:
                # Process already terminated
                self._collect_output()
                return
                
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
            
            # Collect final output
            time.sleep(0.1)  # Give threads time to read final output
            self._collect_output()
            
        except psutil.NoSuchProcess:
            # Process already terminated
            self._collect_output()
        except (subprocess.SubprocessError, OSError):
            # Handle other process-related errors
            try:
                if self.process.poll() is None:
                    self.process.terminate()
                    self.process.wait(timeout=1)
            except Exception:
                pass  # Best effort cleanup
            finally:
                self._collect_output()
    
    def is_running(self):
        """Check if the node process is currently running.
        
        Returns:
            bool: True if the process is running, False otherwise
        """
        if self.process is None:
            return False
        return self.process.poll() is None
    
    def get_return_code(self):
        """Get the return code of the process.
        
        Returns:
            int or None: The return code if process has terminated, None if still running
        """
        if self.process is None:
            return None
        return self.process.poll()
    
    def get_output(self, timeout=0.1):
        """Get accumulated stdout and stderr from the process.
        
        Args:
            timeout: Timeout in seconds to wait for new output (default: 0.1)
        
        Returns:
            tuple: (stdout, stderr) strings with all accumulated output
        """
        # Collect any new output from queues
        time.sleep(timeout)
        self._collect_output()
        
        return (''.join(self._stdout_data), ''.join(self._stderr_data))
    
    def wait_for_output(self, pattern, timeout=5, stream='stdout'):
        """Wait for a specific pattern to appear in the output.
        
        Args:
            pattern: String pattern to search for in the output
            timeout: Maximum time in seconds to wait for the pattern
            stream: Which stream to check - 'stdout', 'stderr', or 'both' (default: 'stdout')
        
        Returns:
            bool: True if pattern was found, False if timeout occurred
        
        Raises:
            ValueError: If stream parameter is invalid
        """
        if stream not in ['stdout', 'stderr', 'both']:
            raise ValueError("stream must be 'stdout', 'stderr', or 'both'")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            stdout, stderr = self.get_output(timeout=0.1)
            
            if stream == 'stdout' and pattern in stdout:
                return True
            elif stream == 'stderr' and pattern in stderr:
                return True
            elif stream == 'both' and (pattern in stdout or pattern in stderr):
                return True
            
            # Check if process has terminated
            if not self.is_running():
                # Process ended, do one final check
                stdout, stderr = self.get_output(timeout=0)
                if stream == 'stdout' and pattern in stdout:
                    return True
                elif stream == 'stderr' and pattern in stderr:
                    return True
                elif stream == 'both' and (pattern in stdout or pattern in stderr):
                    return True
                return False
            
            time.sleep(0.1)
        
        return False
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
        return False
