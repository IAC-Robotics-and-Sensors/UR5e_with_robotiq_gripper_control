import rtde_control
import rtde_receive
import rtde_io
import time
import logging
import threading
import socket
from robotiq_gripper import RobotiqGripper
import numpy as np

logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(levelname)s - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)


class GripperNotActivatedError(Exception):
    """Exception raised when gripper commands are attempted on an inactive gripper."""
    pass


class URControl:
    def __init__(self, robot_ip):
        """
        Initialize the URControl class.

        Args:
            robot_ip (str): IP address of the UR robot.
        """
        self.robot_ip = robot_ip
        self.rtde_ctrl = None
        self.rtde_rec = None
        self.rtde_inout = None
        self.gripper = None

        # connection monitor
        self._monitor_thread = None
        self._monitor_stop_event = threading.Event()
        self._monitor_interval = 1.0  # seconds between health checks

        # Define home position
        self.home_position = (-1.571, -1.396, -2.531, 0.785, 1.571, 4.712)

        # Define waypoints (can be customized)
        self.waypoints = {
            "waypoint_1": (-1.396, -2.217, -0.785, -1.658, 1.571, 3.316),
            "waypoint_2": (-3.153, -3.123, -1.074, -1.990, -1.571, 4.719),
            "waypoint_3": (-3.153, -2.628, -1.340, 0.801, 1.686, 4.633),
        }

    def disconnect_rtde(self):
        """
        Disconnect and clean up RTDE interfaces.
        """
        try:
            if self.rtde_ctrl is not None:
                self.rtde_ctrl.disconnect()
                self.rtde_ctrl = None
        except Exception as e:
            logging.warning(f"Error disconnecting rtde_ctrl: {e}")
            self.rtde_ctrl = None
        
        try:
            if self.rtde_rec is not None:
                self.rtde_rec.disconnect()
                self.rtde_rec = None
        except Exception as e:
            logging.warning(f"Error disconnecting rtde_rec: {e}")
            self.rtde_rec = None
        
        try:
            if self.rtde_inout is not None:
                self.rtde_inout.disconnect()
                self.rtde_inout = None
        except Exception as e:
            logging.warning(f"Error disconnecting rtde_inout: {e}")
            self.rtde_inout = None

    def disconnect_gripper(self):
        """
        Disconnect and clean up gripper connection.
        """
        try:
            if self.gripper is not None:
                self.gripper.disconnect()
                self.gripper = None
        except Exception as e:
            logging.warning(f"Error disconnecting gripper: {e}")
            self.gripper = None

    def connect(self):
        """
        Connect to the robot with retry logic.
        Tries up to 10 times with a short delay between attempts.

        Raises:
            Exception: If connection cannot be established after all retries.
        """
        # First, disconnect any existing connections
        self.disconnect_rtde()
        
        max_retries = 10
        retry_delay = 0.5  # seconds

        for attempt in range(1, max_retries + 1):
            try:
                self.rtde_ctrl = rtde_control.RTDEControlInterface(self.robot_ip, frequency=100)
                self.rtde_rec = rtde_receive.RTDEReceiveInterface(self.robot_ip, frequency=100)
                self.rtde_inout = rtde_io.RTDEIOInterface(self.robot_ip)
                logging.info(f"Connected to robot: {self.robot_ip} on attempt {attempt}")

                # start monitor thread once connected
                if self._monitor_thread is None or not self._monitor_thread.is_alive():
                    self._monitor_stop_event.clear()
                    self._monitor_thread = threading.Thread(target=self._connection_monitor_loop, daemon=True)
                    self._monitor_thread.start()

                return
            except Exception as e:
                logging.error(f"Attempt {attempt} failed: {e}")
                if attempt < max_retries:
                    time.sleep(retry_delay)
                else:
                    logging.error("Max retries reached. Unable to connect to the robot.")
                    raise

    def _connection_monitor_loop(self):
        """Background loop that checks RTDE health and reconnects if needed."""
        while not self._monitor_stop_event.is_set():
            try:
                self.ensure_rtde_connected()
                self.ensure_gripper_connected()
            except Exception as e:
                logging.error(f"Error while ensuring connections: {e}")
            self._monitor_stop_event.wait(self._monitor_interval)

    def connect_gripper(self):
        """
        Connect to the Robotiq gripper.
        """
        # First, disconnect any existing gripper connection
        self.disconnect_gripper()
        
        try:
            self.gripper = RobotiqGripper()
            self.gripper.connect(self.robot_ip, 63352)
            logging.info(f"Connected to gripper at {self.robot_ip}:63352")
        except Exception as e:
            logging.error(f"Failed to connect to gripper: {e}")
            raise

    def ensure_rtde_connected(self):
        """
        Ensure RTDE interfaces are alive; reconnect if needed.
        """
        need_reconnect = False
        if self.rtde_rec is None or self.rtde_ctrl is None or self.rtde_inout is None:
            need_reconnect = True
        else:
            try:
                # Try reading something simple
                _ = self.rtde_rec.getActualQ()
            except Exception as e:
                logging.warning(f"RTDE read failed: {e}. Reconnecting...")
                need_reconnect = True

        if need_reconnect:
            try:
                # Properly disconnect before reconnecting
                logging.info("Attempting RTDE reconnection...")
                self.connect()
                logging.info("RTDE reconnection successful")
            except Exception as e:
                logging.error(f"RTDE reconnection failed: {e}")

    def ensure_gripper_connected(self):
        """
        Ensure gripper is connected; reconnect if needed.
        """
        need_reconnect = False
        if self.gripper is None:
            need_reconnect = True
        else:
            try:
                # Try a simple socket probe
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(0.5)
                result = sock.connect_ex((self.robot_ip, 63352))
                sock.close()
                if result != 0:
                    logging.warning("Gripper connection check failed. Reconnecting...")
                    need_reconnect = True
            except Exception as e:
                logging.warning(f"Gripper health check failed: {e}. Reconnecting...")
                need_reconnect = True

        if need_reconnect:
            try:
                logging.info("Attempting gripper reconnection...")
                self.connect_gripper()
                logging.info("Gripper reconnection successful")
            except Exception as e:
                logging.error(f"Gripper reconnection failed: {e}")

    def stop_robot_control(self):
        """
        Stop the script controlling the robot and stop background monitor.
        """
        # stop monitor
        self._monitor_stop_event.set()
        if self._monitor_thread is not None:
            self._monitor_thread.join(timeout=2.0)

        try:
            if self.rtde_ctrl:
                self.rtde_ctrl.stopScript()
                logging.info("Stopped robot script")
        except Exception as e:
            logging.error(f"Error stopping robot script: {e}")

        # Disconnect all connections
        self.disconnect_rtde()
        self.disconnect_gripper()

    def get_joint_pos(self):
        """Return current joint positions in radians."""
        try:
            return self.rtde_rec.getActualQ()
        except Exception as e:
            logging.error(f"Failed to get joint positions: {e}")
            return None

    def get_tcp_pos(self):
        """Return current TCP position [x, y, z, rx, ry, rz]."""
        try:
            return self.rtde_rec.getActualTCPPose()
        except Exception as e:
            logging.error(f"Failed to get TCP position: {e}")
            return None

    def print_joint_pos(self):
        """Print current joint positions."""
        joint_pos = self.get_joint_pos()
        if joint_pos:
            logging.info(f"Joint positions: {joint_pos}")
        else:
            logging.error("Could not retrieve joint positions")

    def print_tcp_pos(self):
        """Print current TCP position."""
        tcp_pos = self.get_tcp_pos()
        if tcp_pos:
            logging.info(f"TCP position: {tcp_pos}")
        else:
            logging.error("Could not retrieve TCP position")

    def move_j(self, joint_positions, speed=1.0, acceleration=1.0):
        """
        Move to joint positions.
        
        Args:
            joint_positions: List of 6 joint angles in radians
            speed: Joint speed (rad/s)
            acceleration: Joint acceleration (rad/s^2)
        """
        try:
            self.ensure_rtde_connected()
            self.rtde_ctrl.moveJ(joint_positions, speed, acceleration)
            logging.info(f"Moved to joint position: {joint_positions}")
        except Exception as e:
            logging.error(f"Failed to move to joint position: {e}")

    def move_l(self, tcp_pose, speed=0.25, acceleration=1.0):
        """
        Move linearly to TCP pose.
        
        Args:
            tcp_pose: [x, y, z, rx, ry, rz]
            speed: TCP speed (m/s)
            acceleration: TCP acceleration (m/s^2)
        """
        try:
            self.ensure_rtde_connected()
            self.rtde_ctrl.moveL(tcp_pose, speed, acceleration)
            logging.info(f"Moved linearly to: {tcp_pose}")
        except Exception as e:
            logging.error(f"Failed to move linearly: {e}")

    def move_add_j(self, joint_deltas):
        """
        Add joint deltas to current position.
        
        Args:
            joint_deltas: List of 6 joint angle changes in radians
        """
        try:
            current_joints = self.get_joint_pos()
            if current_joints:
                new_joints = [c + d for c, d in zip(current_joints, joint_deltas)]
                self.rtde_ctrl.moveJ(new_joints, 1.0, 1.0, asynchronous=True)
        except Exception as e:
            logging.error(f"Failed to add joint deltas: {e}")

    # --- Pose transformation helpers ---

    def rodrigues_to_rotation_matrix(self, r):
        """
        Convert a Rodrigues vector to a rotation matrix.

        Args:
            r (np.array): Rodrigues rotation vector.

        Returns:
            np.array: 3x3 rotation matrix.
        """
        theta = np.linalg.norm(r)
        if theta < 1e-6:  # No rotation
            return np.eye(3)
        k = r / theta
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

    def pose_to_matrix(self, pose):
        """
        Convert a 6D pose into a 4x4 transformation matrix.

        Args:
            pose (list[float]): [x, y, z, rx, ry, rz].

        Returns:
            np.array: 4x4 transformation matrix.
        """
        R = self.rodrigues_to_rotation_matrix(pose[3:])
        t = np.array(pose[:3])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def matrix_to_pose(self, matrix):
        """
        Convert a 4x4 transformation matrix back into a 6D pose.

        Args:
            matrix (np.array): 4x4 transformation matrix.

        Returns:
            np.array: [x, y, z, rx, ry, rz].
        """
        R = matrix[:3, :3]
        t = matrix[:3, 3]
        theta = np.arccos((np.trace(R) - 1) / 2)
        if theta < 1e-6:
            r = np.zeros(3)
        else:
            r = theta / (2 * np.sin(theta)) * np.array([
                R[2, 1] - R[1, 2],
                R[0, 2] - R[2, 0],
                R[1, 0] - R[0, 1]
            ])
        return np.concatenate((t, r))

    def pose_trans(self, pose1, pose2):
        """
        Combine two poses using matrix multiplication.

        Args:
            pose1 (list[float]): Base pose.
            pose2 (list[float]): Relative pose.

        Returns:
            np.array: Resulting 6D pose.
        """
        T1 = self.pose_to_matrix(pose1)
        T2 = self.pose_to_matrix(pose2)
        T_result = np.dot(T1, T2)
        return self.matrix_to_pose(T_result)

    def relative_world_move(self, pose_delta, speed=0.1, acceleration=0.5):
        """
        Move relative to world frame.
        
        Args:
            pose_delta: [dx, dy, dz, drx, dry, drz]
            speed: TCP speed (m/s)
            acceleration: TCP acceleration (m/s^2)
        """
        try:
            self.ensure_rtde_connected()
            current = self.get_tcp_pos()
            if current:
                new_pose = [c + d for c, d in zip(current, pose_delta)]
                self.rtde_ctrl.moveL(new_pose, speed, acceleration, asynchronous=True)
        except Exception as e:
            logging.error(f"Failed relative world move: {e}")

    def relative_tool_move(self, pose_delta, speed=0.1, acceleration=0.5):
        """
        Move relative to tool frame.
        
        Args:
            pose_delta: [dx, dy, dz, drx, dry, drz]
            speed: TCP speed (m/s)
            acceleration: TCP acceleration (m/s^2)
        """
        try:
            self.ensure_rtde_connected()
            current = self.get_tcp_pos()
            if current:
                # Transform pose_delta from tool frame to world frame
                new_pose = self.pose_trans(current, pose_delta)
                self.rtde_ctrl.moveL(new_pose, speed, acceleration, asynchronous=True)
        except Exception as e:
            logging.error(f"Failed relative tool move: {e}")

    def move_home(self):
        """Move robot to home position."""
        try:
            logging.info("Moving to home position")
            self.move_j(self.home_position)
            logging.info("Reached home position")
        except Exception as e:
            logging.error(f"Cannot move to home: {e}")

    def is_at_home(self, tolerance=0.01):
        """
        Check if robot is at home position.
        
        Args:
            tolerance: Tolerance in radians for each joint
            
        Returns:
            bool: True if at home position
        """
        try:
            current = self.get_joint_pos()
            if current is None:
                return False
            
            for i, (curr, home) in enumerate(zip(current, self.home_position)):
                if abs(curr - home) > tolerance:
                    return False
            return True
        except Exception as e:
            logging.error(f"Error checking home position: {e}")
            return False

    def move_to_waypoint(self, waypoint_name):
        """
        Move to a predefined waypoint.
        
        Args:
            waypoint_name: Name of the waypoint to move to
        """
        if waypoint_name in self.waypoints:
            try:
                logging.info(f"Moving to waypoint: {waypoint_name}")
                self.move_j(self.waypoints[waypoint_name])
                logging.info(f"Reached waypoint: {waypoint_name}")
            except Exception as e:
                logging.error(f"Failed to move to waypoint {waypoint_name}: {e}")
        else:
            logging.error(f"Waypoint '{waypoint_name}' not found")

    # ========== Gripper Control Methods ==========

    def gripper_activate(self):
        """
        Activate the gripper.
        """
        try:
            if self.gripper:
                self.gripper.activate()
                logging.info("Gripper activated")
            else:
                logging.error("Gripper not connected")
        except Exception as e:
            logging.error(f"Failed to activate gripper: {e}")

    def gripper_open(self):
        """
        Open the gripper. Checks if gripper is activated first.
        
        Raises:
            GripperNotActivatedError: If gripper is not activated
        """
        try:
            if self.gripper:
                if not self.gripper.is_active():
                    raise GripperNotActivatedError("Gripper is not activated. Please activate the gripper first.")
                self.gripper.move_and_wait_for_pos(255, 255, 255)
                logging.info("Gripper opened")
            else:
                logging.error("Gripper not connected")
        except GripperNotActivatedError:
            raise
        except Exception as e:
            logging.error(f"Failed to open gripper: {e}")

    def gripper_close(self):
        """
        Close the gripper. Checks if gripper is activated first.
        
        Raises:
            GripperNotActivatedError: If gripper is not activated
        """
        try:
            if self.gripper:
                if not self.gripper.is_active():
                    raise GripperNotActivatedError("Gripper is not activated. Please activate the gripper first.")
                self.gripper.move_and_wait_for_pos(0, 255, 255)
                logging.info("Gripper closed")
            else:
                logging.error("Gripper not connected")
        except GripperNotActivatedError:
            raise
        except Exception as e:
            logging.error(f"Failed to close gripper: {e}")

    def gripper_move(self, position, speed=255, force=255):
        """
        Move gripper to specific position. Checks if gripper is activated first.
        
        Args:
            position: Gripper position (0-255, 0=closed, 255=open)
            speed: Gripper speed (0-255)
            force: Gripper force (0-255)
            
        Raises:
            GripperNotActivatedError: If gripper is not activated
        """
        try:
            if self.gripper:
                if not self.gripper.is_active():
                    raise GripperNotActivatedError("Gripper is not activated. Please activate the gripper first.")
                self.gripper.move_and_wait_for_pos(position, speed, force)
                logging.info(f"Gripper moved to position {position}")
            else:
                logging.error("Gripper not connected")
        except GripperNotActivatedError:
            raise
        except Exception as e:
            logging.error(f"Failed to move gripper: {e}")
