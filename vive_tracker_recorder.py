"""
VIVE Tracker Data Collection and 6DoF Visualization Script.

This script reads tracking data from VIVE trackers via OpenVR and provides
real-time visualization and CSV logging capabilities.
"""

import argparse
import csv
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import matplotlib.pyplot as plt
import numpy as np
import openvr
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from win_precise_time import sleep

# Default configuration constants
DEFAULT_SAMPLING_RATE = 120  # Hz
DEFAULT_OUTPUT_FILE = "ultimate_tracker_data.csv"


@dataclass
class VisualizationConfig:
    """Configuration for 6DoF visualization."""
    tracker_size: tuple = (0.08, 0.05, 0.03)  # (length, width, height) in meters
    axis_length: float = 0.12
    max_history: int = 100
    ghost_interval: int = 10
    ghost_alpha: float = 0.3
    trajectory_colormap: str = 'viridis'
    ground_plane_size: float = 2.0
    ground_plane_alpha: float = 0.2
    figsize: tuple = (16, 10)

# Default visualization config instance
_DEFAULT_VIS = VisualizationConfig()


def parse_arguments() -> argparse.Namespace:
    """Parse command-line arguments for the tracker data collection script."""
    parser = argparse.ArgumentParser(
        description='VIVE Tracker Data Collection and 6DoF Visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python ReadOutTracker.py                           # Default (6DoF visualization)
  python ReadOutTracker.py --no-plot --no-print      # High-performance logging only
  python ReadOutTracker.py -r 240 -o my_data.csv     # Custom rate and output file
  python ReadOutTracker.py --plot-6dof --plot-3d     # Multiple visualizations
        """
    )
    
    # Basic settings
    basic = parser.add_argument_group('Basic Settings')
    basic.add_argument('-r', '--rate', type=int, default=DEFAULT_SAMPLING_RATE,
                       metavar='HZ', help=f'Sampling rate in Hz (default: {DEFAULT_SAMPLING_RATE})')
    basic.add_argument('-o', '--output', type=str, default=DEFAULT_OUTPUT_FILE,
                       metavar='FILE', help=f'Output CSV file (default: {DEFAULT_OUTPUT_FILE})')
    
    # Visualization toggles
    viz = parser.add_argument_group('Visualization Options')
    viz.add_argument('--plot-6dof', action='store_true', default=True, dest='plot_6dof',
                     help='Enable 6DoF visualization (default: enabled)')
    viz.add_argument('--no-plot-6dof', action='store_false', dest='plot_6dof',
                     help='Disable 6DoF visualization')
    viz.add_argument('--plot-3d', action='store_true', default=False, dest='plot_3d',
                     help='Enable legacy 3D plot (position only)')
    viz.add_argument('--plot-xyz', action='store_true', default=False, dest='plot_xyz',
                     help='Enable X/Y/Z position time series plots')
    viz.add_argument('--no-plot', action='store_true', default=False,
                     help='Disable all visualizations')
    
    # Data output options
    output = parser.add_argument_group('Data Output Options')
    output.add_argument('--log', action='store_true', default=True, dest='log_data',
                        help='Enable CSV logging (default: enabled)')
    output.add_argument('--no-log', action='store_false', dest='log_data',
                        help='Disable CSV logging')
    output.add_argument('--print', action='store_true', default=True, dest='print_data',
                        help='Print tracker data to console (default: enabled)')
    output.add_argument('--no-print', '-q', action='store_false', dest='print_data',
                        help='Suppress console output')
    
    # Advanced visualization settings
    adv = parser.add_argument_group('Advanced Visualization Settings')
    adv.add_argument('--max-history', type=int, default=_DEFAULT_VIS.max_history,
                     metavar='N', help=f'Max trajectory history points (default: {_DEFAULT_VIS.max_history})')
    adv.add_argument('--ghost-interval', type=int, default=_DEFAULT_VIS.ghost_interval,
                     metavar='N', help=f'Orientation ghost interval (default: {_DEFAULT_VIS.ghost_interval})')
    adv.add_argument('--axis-length', type=float, default=_DEFAULT_VIS.axis_length,
                     metavar='M', help=f'Orientation axis length in meters (default: {_DEFAULT_VIS.axis_length})')
    adv.add_argument('--colormap', type=str, default=_DEFAULT_VIS.trajectory_colormap,
                     metavar='NAME', help=f'Trajectory colormap (default: {_DEFAULT_VIS.trajectory_colormap})')
    adv.add_argument('--ground-plane-size', type=float, default=_DEFAULT_VIS.ground_plane_size,
                     metavar='M', help=f'Ground plane size in meters (default: {_DEFAULT_VIS.ground_plane_size})')
    adv.add_argument('--figsize', type=float, nargs=2, default=list(_DEFAULT_VIS.figsize),
                     metavar=('W', 'H'), help=f'Figure size in inches (default: {_DEFAULT_VIS.figsize})')
    adv.add_argument('--list-devices', action='store_true', default=False,
                     help='List discovered VR devices and exit')
    
    args = parser.parse_args()
    
    if args.no_plot:
        args.plot_6dof = args.plot_3d = args.plot_xyz = False
    
    return args


def create_vis_config(args: argparse.Namespace) -> VisualizationConfig:
    """Create a VisualizationConfig from parsed arguments."""
    return VisualizationConfig(
        max_history=args.max_history,
        ghost_interval=args.ghost_interval,
        axis_length=args.axis_length,
        trajectory_colormap=args.colormap,
        ground_plane_size=args.ground_plane_size,
        figsize=tuple(args.figsize),
    )
def precise_wait(duration: float) -> None:
    """Wait with high precision. Uses sleep for >= 1ms, otherwise busy-waits."""
    end = time.time() + duration
    if duration >= 0.001:
        sleep(duration)
    while time.time() < end:
        pass

class VRSystemManager:
    """Manages OpenVR system initialization and tracker data retrieval."""
    
    def __init__(self):
        self.vr_system: Optional[openvr.IVRSystem] = None

    def initialize(self) -> bool:
        """Initialize the VR system. Returns True on success."""
        try:
            openvr.init(openvr.VRApplication_Other)
            self.vr_system = openvr.VRSystem()
            print("VR system initialized. Starting capture...")
            return True
        except Exception as e:
            print(f"Failed to initialize VR system: {e}")
            return False

    def get_poses(self):
        """Retrieve all device poses from the VR system."""
        return self.vr_system.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount
        )

    def get_device_class(self, index: int) -> int:
        """Get the device class for a given device index."""
        return self.vr_system.getTrackedDeviceClass(index)

    def list_devices(self) -> None:
        """Print information about all discovered VR devices."""
        print("\nDiscovered VR Devices:")
        print("-" * 50)
        for i in range(openvr.k_unMaxTrackedDeviceCount):
            device_class = self.vr_system.getTrackedDeviceClass(i)
            if device_class != openvr.TrackedDeviceClass_Invalid:
                serial = self.vr_system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_SerialNumber_String
                )
                model = self.vr_system.getStringTrackedDeviceProperty(
                    i, openvr.Prop_ModelNumber_String
                )
                print(f"Device {i}: {serial} ({model})")

    def shutdown(self) -> None:
        """Shutdown the VR system."""
        if self.vr_system:
            openvr.shutdown()
            self.vr_system = None

class CSVLogger:
    """Handles CSV file logging for tracker data."""
    
    HEADER = ['TrackerIndex', 'Time', 'PositionX', 'PositionY', 'PositionZ',
              'RotationW', 'RotationX', 'RotationY', 'RotationZ']
    
    def __init__(self):
        self._file = None
        self._writer = None

    def open(self, filename: str) -> bool:
        """Open CSV file for writing. Returns True on success."""
        try:
            self._file = open(filename, 'w', newline='')
            self._writer = csv.writer(self._file)
            self._writer.writerow(self.HEADER)
            return True
        except Exception as e:
            print(f"Failed to initialize CSV file: {e}")
            return False

    def write(self, index: int, timestamp: float, position: list) -> None:
        """Write a single data row to the CSV file."""
        if self._writer:
            try:
                self._writer.writerow([index, timestamp, *position])
            except Exception as e:
                print(f"Failed to write data to CSV file: {e}")

    def close(self) -> None:
        """Close the CSV file."""
        if self._file:
            self._file.close()
            self._file = None
            self._writer = None

class PoseConverter:
    """Utility class for converting pose data between different representations."""
    
    @staticmethod
    def pose_matrix_to_pose(pose_mat) -> list:
        """Convert a 3x4 pose matrix to [x, y, z, qw, qx, qy, qz]."""
        # Extract position
        x, y, z = pose_mat[0][3], pose_mat[1][3], pose_mat[2][3]
        
        # Convert rotation matrix to quaternion
        trace = pose_mat[0][0] + pose_mat[1][1] + pose_mat[2][2]
        qw = max(0.0001, math.sqrt(abs(1 + trace)) / 2)
        qx = (pose_mat[2][1] - pose_mat[1][2]) / (4 * qw)
        qy = (pose_mat[0][2] - pose_mat[2][0]) / (4 * qw)
        qz = (pose_mat[1][0] - pose_mat[0][1]) / (4 * qw)
        
        return [x, y, z, qw, qx, qy, qz]

    @staticmethod
    def quaternion_to_euler(w: float, x: float, y: float, z: float) -> tuple:
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    @staticmethod
    def quaternion_to_rotation_matrix(w: float, x: float, y: float, z: float) -> np.ndarray:
        """Convert quaternion to 3x3 rotation matrix."""
        # Normalize quaternion
        norm = math.sqrt(w*w + x*x + y*y + z*z)
        if norm == 0:
            return np.eye(3)
        w, x, y, z = w/norm, x/norm, y/norm, z/norm

        return np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
        ])

class LivePlotter:
    """Handles real-time visualization of tracker data."""
    
    def __init__(self, config: Optional[VisualizationConfig] = None):
        self.config = config or VisualizationConfig()
        self.start_time = time.time()
        
        # XYZ plot state
        self._xyz_initialized = False
        self._first_pos = None
        
        # 3D plot state
        self._3d_initialized = False
        
        # 6DoF plot state
        self._6dof_initialized = False

    def init_xyz_plot(self) -> None:
        """Initialize XYZ position time series plots."""
        self.fig_xyz, (self.ax_x, self.ax_y, self.ax_z) = plt.subplots(3, 1)
        for ax, title, color in [(self.ax_x, 'X Position', 'r'),
                                   (self.ax_y, 'Y Position', 'g'),
                                   (self.ax_z, 'Z Position', 'b')]:
            ax.set_title(title)
            ax.grid(True, alpha=0.3)
        
        self.xyz_time = deque()
        self.xyz_data = {axis: deque() for axis in 'xyz'}
        self.xyz_lines = {
            'x': self.ax_x.plot([], [], 'r-')[0],
            'y': self.ax_y.plot([], [], 'g-')[0],
            'z': self.ax_z.plot([], [], 'b-')[0],
        }
        self._xyz_initialized = True
        plt.ion()
        plt.show()

    def update_xyz_plot(self, position: tuple) -> None:
        """Update XYZ position plots with new data."""
        if not self._xyz_initialized:
            return
        
        x, y, z = position
        elapsed = time.time() - self.start_time
        
        self.xyz_time.append(elapsed)
        self.xyz_data['x'].append(x)
        self.xyz_data['y'].append(y)
        self.xyz_data['z'].append(z)
        
        for axis in 'xyz':
            self.xyz_lines[axis].set_data(self.xyz_time, self.xyz_data[axis])
        
        if self._first_pos is None:
            self._first_pos = (x, y, z)
        
        # Update axis limits
        t_range = (self.xyz_time[0], self.xyz_time[-1])
        for ax, center in [(self.ax_x, self._first_pos[0]),
                           (self.ax_y, self._first_pos[1]),
                           (self.ax_z, self._first_pos[2])]:
            ax.set_xlim(*t_range)
            ax.set_ylim(center - 1.5, center + 1.5)
        
        self.fig_xyz.canvas.draw()
        self.fig_xyz.canvas.flush_events()

    def init_3d_plot(self) -> None:
        """Initialize legacy 3D position plot."""
        self.fig_3d = plt.figure()
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        self.ax_3d.view_init(elev=25, azim=-135, vertical_axis='z')
        self.ax_3d.set_xlabel('X (+Left/-Right)')
        self.ax_3d.set_ylabel('Z (Front/Back)')
        self.ax_3d.set_zlabel('Y (Up/Down)')
        self.ax_3d.set_title('3D Tracker Position')
        # Invert X axis so larger values appear on the left
        self.ax_3d.invert_xaxis()
        
        self.pos_3d = {axis: deque(maxlen=50) for axis in 'xyz'}
        self.line_3d, = self.ax_3d.plot([], [], [], 'r-')
        self._3d_initialized = True
        plt.ion()
        plt.show()

    def update_3d_plot(self, position: tuple) -> None:
        """Update 3D position plot with new data.
        
        Swaps Y and Z for matplotlib: our (X,Y,Z) -> matplotlib (x,z,y)
        """
        if not self._3d_initialized:
            return
        
        x, y, z = position
        self.pos_3d['x'].append(x)
        self.pos_3d['y'].append(y)
        self.pos_3d['z'].append(z)
        
        # Plot with swapped axes: x, z (front/back), y (up/down)
        self.line_3d.set_data(self.pos_3d['x'], self.pos_3d['z'])
        self.line_3d.set_3d_properties(self.pos_3d['y'])
        
        if len(self.pos_3d['x']) > 1:
            # Map: our X -> x, our Z -> y, our Y -> z
            self.ax_3d.set_xlim(min(self.pos_3d['x']), max(self.pos_3d['x']))
            self.ax_3d.set_ylim(min(self.pos_3d['z']), max(self.pos_3d['z']))
            self.ax_3d.set_zlim(min(self.pos_3d['y']), max(self.pos_3d['y']))
        
        self.fig_3d.canvas.draw()
        self.fig_3d.canvas.flush_events()

    def _get_tracker_faces(self, vertices: np.ndarray) -> list:
        """Get the 6 faces of the tracker box from vertices."""
        face_indices = [
            [0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4],
            [2, 3, 7, 6], [0, 3, 7, 4], [1, 2, 6, 5],
        ]
        return [[vertices[i] for i in face] for face in face_indices]

    def _create_tracker_vertices(self, size: tuple) -> np.ndarray:
        """Create 8 vertices for a 3D box tracker model."""
        l, w, h = size[0]/2, size[1]/2, size[2]/2
        return np.array([
            [-l, -w, -h], [l, -w, -h], [l, w, -h], [-l, w, -h],
            [-l, -w, h], [l, -w, h], [l, w, h], [-l, w, h]
        ])

    def _draw_ground_plane(self, ax, size: float, alpha: float) -> None:
        """Draw a ground plane grid at Y=0 (X-Z plane).
        
        Coordinate system: X=left/right, Y=up/down, Z=front/back
        Ground plane is drawn in matplotlib's (x, y, z) where we map:
        - matplotlib x -> our X (left/right)
        - matplotlib y -> our Z (front/back)  
        - matplotlib z -> our Y (up/down)
        """
        grid_spacing = 0.25
        half = size / 2
        
        # Draw grid on X-Z plane (at Y=0, which is matplotlib's z=0)
        for x in np.arange(-half, half + grid_spacing, grid_spacing):
            ax.plot([x, x], [-half, half], [0, 0], 'gray', alpha=alpha, linewidth=0.5)
        for z in np.arange(-half, half + grid_spacing, grid_spacing):
            ax.plot([-half, half], [z, z], [0, 0], 'gray', alpha=alpha, linewidth=0.5)
        
        corners_x = [-half, half, half, -half, -half]
        corners_z = [-half, -half, half, half, -half]
        ax.plot(corners_x, corners_z, [0]*5, 'k-', alpha=alpha*2, linewidth=1)
        
        # Axis labels: X=left/right (inverted: +X=Left), Y=up/down, Z=front/back
        ax.text(half + 0.1, 0, 0, '+X (Left)', fontsize=8, color='red', alpha=0.7)
        ax.text(-half - 0.1, 0, 0, '-X (Right)', fontsize=8, color='red', alpha=0.5)
        ax.text(0, half + 0.1, 0, '+Z (Front)', fontsize=8, color='blue', alpha=0.7)
        ax.text(0, 0, half + 0.1, '+Y (Up)', fontsize=8, color='green', alpha=0.7)

    def init_6dof_plot(self) -> None:
        """Initialize enhanced 6DoF visualization with position, orientation, and trajectory."""
        cfg = self.config
        self.fig_6dof = plt.figure(figsize=cfg.figsize)
        
        # Main 3D view (left half)
        # Coordinate system: X=left/right (inverted: +X=Left), Y=up/down, Z=front/back
        self.ax_6dof_3d = self.fig_6dof.add_subplot(1, 2, 1, projection='3d')
        self.ax_6dof_3d.set_xlabel('X (m) [+Left/-Right]', fontsize=10)
        self.ax_6dof_3d.set_ylabel('Z (m) [Front/Back]', fontsize=10)
        self.ax_6dof_3d.set_zlabel('Y (m) [Up/Down]', fontsize=10)
        self.ax_6dof_3d.set_title('6DoF Tracker Pose\n(Position + Orientation)', fontsize=12)
        self.ax_6dof_3d.view_init(elev=25, azim=-135)
        
        # Initialize trajectory data with config
        maxlen = cfg.max_history
        self.pos_6dof = {axis: deque(maxlen=maxlen) for axis in 'xyz'}
        self.quat_history = deque(maxlen=maxlen)
        self.time_6dof = deque(maxlen=maxlen)
        self.euler_6dof = {angle: deque(maxlen=maxlen) for angle in ('roll', 'pitch', 'yaw')}
        
        # Create tracker model vertices
        self.base_tracker_vertices = self._create_tracker_vertices(cfg.tracker_size)
        
        # Create ground plane
        self._draw_ground_plane(self.ax_6dof_3d, cfg.ground_plane_size, cfg.ground_plane_alpha)
        
        # Plot objects (will be updated)
        self.trajectory_scatter = None
        self.tracker_model = None
        self.orientation_quivers = None
        self.ghost_models = []
        self.ghost_quivers = []
        
        # Add legend
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='purple', markersize=8, label='Trajectory'),
            Patch(facecolor='steelblue', edgecolor='darkblue', alpha=0.7, label='Tracker'),
            Line2D([0], [0], color='red', linewidth=2, label='X (+Left/-Right)'),
            Line2D([0], [0], color='green', linewidth=2, label='Y (Up/Down)'),
            Line2D([0], [0], color='blue', linewidth=2, label='Z (Front/Back)'),
        ]
        self.ax_6dof_3d.legend(handles=legend_elements, loc='upper left', fontsize=8)
        
        # Invert X axis so larger values appear on the left
        self.ax_6dof_3d.invert_xaxis()
        
        # Position time series (top right)
        self.ax_pos_time = self.fig_6dof.add_subplot(2, 2, 2)
        self.ax_pos_time.set_title('Position over Time', fontsize=11)
        self.ax_pos_time.set_xlabel('Time (s)')
        self.ax_pos_time.set_ylabel('Position (m)')
        self.ax_pos_time.grid(True, alpha=0.3)
        self.pos_lines = {
            'x': self.ax_pos_time.plot([], [], 'r-', label='X (+L/-R)', linewidth=1.5)[0],
            'y': self.ax_pos_time.plot([], [], 'g-', label='Y (U/D)', linewidth=1.5)[0],
            'z': self.ax_pos_time.plot([], [], 'b-', label='Z (F/B)', linewidth=1.5)[0],
        }
        self.ax_pos_time.legend(loc='upper right')
        
        # Euler angles time series (bottom right)
        self.ax_euler_time = self.fig_6dof.add_subplot(2, 2, 4)
        self.ax_euler_time.set_title('Orientation (Euler Angles) over Time', fontsize=11)
        self.ax_euler_time.set_xlabel('Time (s)')
        self.ax_euler_time.set_ylabel('Angle (degrees)')
        self.ax_euler_time.grid(True, alpha=0.3)
        self.euler_lines = {
            'roll': self.ax_euler_time.plot([], [], 'r-', label='Roll', linewidth=1.5)[0],
            'pitch': self.ax_euler_time.plot([], [], 'g-', label='Pitch', linewidth=1.5)[0],
            'yaw': self.ax_euler_time.plot([], [], 'b-', label='Yaw', linewidth=1.5)[0],
        }
        self.ax_euler_time.legend(loc='upper right')
        
        self.start_time_6dof = time.time()
        self._6dof_initialized = True
        
        plt.tight_layout()
        plt.ion()
        plt.show()

    def _draw_tracker_model(self, position, rot_matrix, alpha=0.7, color='steelblue', edgecolor='darkblue'):
        """Draw the 3D tracker model at a given position and orientation.
        
        Args:
            position: (x, y, z) tuple in matplotlib coordinates (already swapped)
            rot_matrix: 3x3 rotation matrix
            alpha: Transparency (0-1)
            color: Face color
            edgecolor: Edge color
        
        Returns:
            Poly3DCollection object
            
        Note: Vertices are swapped Y<->Z to match matplotlib coordinates.
        """
        # Rotate vertices in our coordinate system
        rotated = (rot_matrix @ self.base_tracker_vertices.T).T
        
        # Swap Y<->Z columns and add position (already in matplotlib coords)
        # Our (x, y, z) -> matplotlib (x, z, y)
        swapped_vertices = np.column_stack([rotated[:, 0], rotated[:, 2], rotated[:, 1]])
        translated_vertices = swapped_vertices + np.array(position)
        
        # Get faces
        faces = self._get_tracker_faces(translated_vertices)
        
        # Different colors for different faces to show orientation better
        face_colors = [
            (0.8, 0.2, 0.2, alpha),  # Bottom - red tint
            (0.2, 0.8, 0.2, alpha),  # Top - green tint (shows "up" direction)
            color,                    # Front
            color,                    # Back  
            color,                    # Left
            (0.2, 0.2, 0.8, alpha),  # Right - blue tint (shows "forward")
        ]
        
        model = Poly3DCollection(faces, alpha=alpha, linewidths=1, edgecolors=edgecolor)
        model.set_facecolors(face_colors)
        self.ax_6dof_3d.add_collection3d(model)
        
        return model
    
    def _draw_orientation_axes(self, position, rot_matrix, length, alpha=1.0, linewidth=2):
        """Draw orientation axes (quivers) at a given position.
        
        Args:
            position: (x, y, z) tuple in matplotlib coordinates (already swapped)
            rot_matrix: 3x3 rotation matrix
            length: Length of the axes
            alpha: Transparency
            linewidth: Line width
        
        Returns:
            Tuple of (quiver_x, quiver_y, quiver_z)
        
        Note: Axis directions are swapped Y<->Z to match matplotlib coordinates.
        """
        x, y, z = position  # Already in matplotlib coords: x, z_data, y_data
        
        # Calculate axis directions in our coordinate system
        x_axis = rot_matrix @ np.array([length, 0, 0])
        y_axis = rot_matrix @ np.array([0, length, 0])
        z_axis = rot_matrix @ np.array([0, 0, length])
        
        # Swap Y<->Z components for matplotlib: (ax, ay, az) -> (ax, az, ay)
        # Create quivers with swapped axis components
        qx = self.ax_6dof_3d.quiver(x, y, z, x_axis[0], x_axis[2], x_axis[1],
                                     color='red', arrow_length_ratio=0.15, linewidth=linewidth, alpha=alpha)
        qy = self.ax_6dof_3d.quiver(x, y, z, y_axis[0], y_axis[2], y_axis[1],
                                     color='green', arrow_length_ratio=0.15, linewidth=linewidth, alpha=alpha)
        qz = self.ax_6dof_3d.quiver(x, y, z, z_axis[0], z_axis[2], z_axis[1],
                                     color='blue', arrow_length_ratio=0.15, linewidth=linewidth, alpha=alpha)
        
        return (qx, qy, qz)

    def update_6dof_plot(self, position_data: list) -> None:
        """Update 6DoF visualization with new tracker data.
        
        Coordinate mapping: Our (X,Y,Z) -> Matplotlib (x,y,z)
        - X (left/right) -> x
        - Y (up/down) -> z (matplotlib vertical)
        - Z (front/back) -> y
        """
        if not self._6dof_initialized:
            return
        
        x, y, z, qw, qx, qy, qz = position_data
        cfg = self.config
        elapsed = time.time() - self.start_time_6dof
        
        # Update trajectory data
        self.pos_6dof['x'].append(x)
        self.pos_6dof['y'].append(y)
        self.pos_6dof['z'].append(z)
        self.time_6dof.append(elapsed)
        self.quat_history.append((qw, qx, qy, qz))
        
        # Calculate Euler angles
        roll, pitch, yaw = PoseConverter.quaternion_to_euler(qw, qx, qy, qz)
        self.euler_6dof['roll'].append(roll)
        self.euler_6dof['pitch'].append(pitch)
        self.euler_6dof['yaw'].append(yaw)
        
        rot_matrix = PoseConverter.quaternion_to_rotation_matrix(qw, qx, qy, qz)
        
        # Update trajectory scatter (swap Y<->Z for matplotlib)
        if self.trajectory_scatter is not None:
            self.trajectory_scatter.remove()
        
        if len(self.pos_6dof['x']) > 1:
            time_normalized = np.linspace(0, 1, len(self.pos_6dof['x']))
            # Plot with swapped axes: x, z (front/back), y (up/down)
            self.trajectory_scatter = self.ax_6dof_3d.scatter(
                list(self.pos_6dof['x']), list(self.pos_6dof['z']), list(self.pos_6dof['y']),
                c=time_normalized, cmap=cfg.trajectory_colormap, s=15, alpha=0.7
            )
        
        # Update tracker model (swap Y<->Z)
        if self.tracker_model is not None:
            self.tracker_model.remove()
        self.tracker_model = self._draw_tracker_model((x, z, y), rot_matrix)
        
        # Update orientation quivers (swap Y<->Z)
        if self.orientation_quivers is not None:
            for q in self.orientation_quivers:
                q.remove()
        self.orientation_quivers = self._draw_orientation_axes((x, z, y), rot_matrix, cfg.axis_length)
        
        # Update ghost models
        for ghost in self.ghost_models:
            ghost.remove()
        for gq in self.ghost_quivers:
            for q in gq:
                q.remove()
        self.ghost_models = []
        self.ghost_quivers = []
        
        # Draw new ghosts at intervals
        if len(self.pos_6dof['x']) > cfg.ghost_interval:
            pos_lists = {axis: list(self.pos_6dof[axis]) for axis in 'xyz'}
            quat_list = list(self.quat_history)
            
            for i in range(0, len(pos_lists['x']) - 1, cfg.ghost_interval):
                # Swap Y<->Z for matplotlib plotting
                ghost_pos = (pos_lists['x'][i], pos_lists['z'][i], pos_lists['y'][i])
                ghost_rot = PoseConverter.quaternion_to_rotation_matrix(*quat_list[i])
                
                ghost_model = self._draw_tracker_model(
                    ghost_pos, ghost_rot, alpha=cfg.ghost_alpha * 0.5,
                    color='lightgray', edgecolor='gray'
                )
                self.ghost_models.append(ghost_model)
                
                ghost_axes = self._draw_orientation_axes(
                    ghost_pos, ghost_rot, cfg.axis_length * 0.7,
                    alpha=cfg.ghost_alpha, linewidth=1
                )
                self.ghost_quivers.append(ghost_axes)
        
        # Update 3D plot bounds
        self._update_3d_bounds()
        
        # Update time series plots
        self._update_time_series_plots()
        
        self.fig_6dof.canvas.draw()
        self.fig_6dof.canvas.flush_events()

    def _update_3d_bounds(self) -> None:
        """Update 3D axis bounds based on trajectory data.
        
        Maps our axes to matplotlib: X->x, Z->y, Y->z
        """
        if len(self.pos_6dof['x']) <= 1:
            return
        
        ranges = {axis: (min(self.pos_6dof[axis]), max(self.pos_6dof[axis])) for axis in 'xyz'}
        padding = max(0.3, self.config.axis_length * 2.5)
        max_range = max(r[1] - r[0] for r in ranges.values())
        max_range = max(max_range, padding) / 2
        
        # Map: our X -> matplotlib x, our Z -> matplotlib y, our Y -> matplotlib z
        for our_axis, mpl_axis in [('x', 'x'), ('z', 'y'), ('y', 'z')]:
            min_val, max_val = ranges[our_axis]
            center = (max_val + min_val) / 2
            setter = getattr(self.ax_6dof_3d, f'set_{mpl_axis}lim')
            setter(center - max_range, center + max_range)

    def _update_time_series_plots(self) -> None:
        """Update position and Euler angle time series plots."""
        time_list = list(self.time_6dof)
        if len(time_list) <= 1:
            return
        
        # Update position lines
        for axis in 'xyz':
            self.pos_lines[axis].set_data(time_list, list(self.pos_6dof[axis]))
        
        all_pos = sum((list(self.pos_6dof[a]) for a in 'xyz'), [])
        pos_min, pos_max = min(all_pos), max(all_pos)
        pos_padding = max(0.1, (pos_max - pos_min) * 0.1)
        self.ax_pos_time.set_xlim(time_list[0], time_list[-1])
        self.ax_pos_time.set_ylim(pos_min - pos_padding, pos_max + pos_padding)
        
        # Update Euler angle lines
        for angle in ('roll', 'pitch', 'yaw'):
            self.euler_lines[angle].set_data(time_list, list(self.euler_6dof[angle]))
        
        all_angles = sum((list(self.euler_6dof[a]) for a in ('roll', 'pitch', 'yaw')), [])
        angle_min, angle_max = min(all_angles), max(all_angles)
        angle_padding = max(5, (angle_max - angle_min) * 0.1)
        self.ax_euler_time.set_xlim(time_list[0], time_list[-1])
        self.ax_euler_time.set_ylim(angle_min - angle_padding, angle_max + angle_padding)


def main() -> None:
    """Main entry point for VIVE tracker data collection."""
    args = parse_arguments()
    vis_config = create_vis_config(args)
    
    vr_manager = VRSystemManager()
    csv_logger = CSVLogger()
    plotter = LivePlotter(vis_config)
    
    if not vr_manager.initialize():
        return
    
    if args.list_devices:
        vr_manager.list_devices()
        vr_manager.shutdown()
        return
    
    if args.log_data and not csv_logger.open(args.output):
        vr_manager.shutdown()
        return
    
    # Print configuration summary
    print(f"\n{'='*50}")
    print("VIVE Tracker Data Collection")
    print(f"{'='*50}")
    print(f"Sampling Rate: {args.rate} Hz")
    print(f"Output File: {args.output if args.log_data else 'Disabled'}")
    print(f"Visualizations:")
    print(f"  - 6DoF Plot: {'Enabled' if args.plot_6dof else 'Disabled'}")
    print(f"  - 3D Plot: {'Enabled' if args.plot_3d else 'Disabled'}")
    print(f"  - XYZ Plot: {'Enabled' if args.plot_xyz else 'Disabled'}")
    print(f"Console Output: {'Enabled' if args.print_data else 'Disabled'}")
    print(f"{'='*50}\n")
    
    # Initialize plots
    if args.plot_xyz:
        plotter.init_xyz_plot()
    if args.plot_3d:
        plotter.init_3d_plot()
    if args.plot_6dof:
        plotter.init_6dof_plot()
    
    try:
        while True:
            poses = vr_manager.get_poses()
            for i in range(openvr.k_unMaxTrackedDeviceCount):
                if not poses[i].bPoseIsValid:
                    continue
                if vr_manager.get_device_class(i) != openvr.TrackedDeviceClass_GenericTracker:
                    continue
                
                timestamp = time.time()
                pose = PoseConverter.pose_matrix_to_pose(poses[i].mDeviceToAbsoluteTracking)
                
                if args.plot_xyz:
                    plotter.update_xyz_plot(pose[:3])
                if args.plot_3d:
                    plotter.update_3d_plot(pose[:3])
                if args.plot_6dof:
                    plotter.update_6dof_plot(pose)
                if args.log_data:
                    csv_logger.write(i - 1, timestamp, pose)
                if args.print_data:
                    print(f"Tracker {i - 1}: {pose}")
            
            precise_wait(1 / args.rate)
    except KeyboardInterrupt:
        print("\nStopping data collection...")
    finally:
        vr_manager.shutdown()
        csv_logger.close()
        if args.log_data:
            print(f"Data saved to: {args.output}")


if __name__ == "__main__":
    main()
