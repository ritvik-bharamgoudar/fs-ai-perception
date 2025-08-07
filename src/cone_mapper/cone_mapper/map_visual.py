#!/usr/bin/env python3
"""
Real-time 2D visualisation of mapped cones, vehicle position and track edges
Supports both cone mapping and track generation visualization
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import threading
import time

class ConeMapVisualiser(Node):
    """Cone map visualiser with track generation support"""
    
    def __init__(self):
        super().__init__('cone_map_visualiser')
        
        # Data storage
        self.local_cones = {'blue': [], 'yellow': [], 'orange': []}
        self.global_cones = {'blue': [], 'yellow': [], 'orange': []}
        self.vehicle_position = []
        self.vehicle_heading = 0.0
        self.vehicle_path = deque(maxlen=100)  # Longer path for better visualization
        
        # Track generation data
        self.outer_boundary = []
        self.inner_boundary = []
        self.centerline = []
        self.start_line = []
        
        # Track generation status
        self.track_generator_active = False
        self.last_track_update = 0.0
        
        # Thread lock for data safety
        self.data_lock = threading.Lock()
        
        # Visualization parameters
        self.zoom_level = 15.0
        self.auto_zoom = True
        self.show_confidence = True
        self.show_vehicle_path = True
        
        # Subscriptions - flexible topic names
        self.setup_subscriptions()
        
        # Statistics and performance tracking
        self.stats = {
            'local_cone_count': 0,
            'global_cone_count': 0,
            'track_points': 0,
            'last_cone_update': 0.0,
            'last_track_update': 0.0,
            'fps': 0.0
        }
        
        # Performance tracking
        self.last_update_time = time.time()
        self.frame_times = deque(maxlen=10)
        
        self.get_logger().info('Cone Map Visualiser initialised')
    
    def setup_subscriptions(self):
        """Setup all ROS subscriptions with flexible topic names"""
        # Cone mapping subscriptions
        self.local_sub = self.create_subscription(
            String, '/cone_map/local', self.local_cone_callback, 10)
        self.global_sub = self.create_subscription(
            String, '/cone_map/global', self.global_cone_callback, 10)
        
        # Vehicle pose - try multiple common topics
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/slam', self.pose_callback, 10)
        
        # Track generation subscriptions (new separate boundaries)
        self.outer_boundary_sub = self.create_subscription(
            Path, '/track/outer_boundary', self.outer_boundary_callback, 10)
        self.inner_boundary_sub = self.create_subscription(
            Path, '/track/inner_boundary', self.inner_boundary_callback, 10)
        self.centerline_sub = self.create_subscription(
            Path, '/track/centerline', self.centerline_callback, 10)
        self.start_line_sub = self.create_subscription(
            Path, '/track/start_line', self.start_line_callback, 10)
        
        # Legacy boundary support (for backwards compatibility)
        self.boundary_sub = self.create_subscription(
            Path, '/track/boundaries', self.legacy_boundary_callback, 10)
    
    def local_cone_callback(self, msg):
        """Handle local cone map updates"""
        with self.data_lock:
            self.local_cones = {'blue': [], 'yellow': [], 'orange': []}
            
            if not msg.data.strip():
                return
                
            lines = msg.data.strip().split('\n')
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    try:
                        x, y, z, color = float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])
                        confidence = float(parts[4]) if len(parts) > 4 else 1.0
                        
                        color_name = {0: 'blue', 1: 'yellow', 2: 'orange'}.get(color, 'blue')
                        self.local_cones[color_name].append({
                            'x': x, 'y': y, 'z': z, 'confidence': confidence
                        })
                    except (ValueError, IndexError):
                        continue
            
            self.stats['local_cone_count'] = sum(len(cones) for cones in self.local_cones.values())
            self.stats['last_cone_update'] = time.time()
    
    def global_cone_callback(self, msg):
        """Handle global cone map updates"""
        with self.data_lock:
            self.global_cones = {'blue': [], 'yellow': [], 'orange': []}
            
            if not msg.data.strip():
                return
                
            lines = msg.data.strip().split('\n')
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) >= 4:
                    try:
                        x, y, z, color = float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])
                        
                        color_name = {0: 'blue', 1: 'yellow', 2: 'orange'}.get(color, 'blue')
                        self.global_cones[color_name].append({'x': x, 'y': y, 'z': z})
                    except (ValueError, IndexError):
                        continue
            
            self.stats['global_cone_count'] = sum(len(cones) for cones in self.global_cones.values())
    
    def pose_callback(self, msg):
        """Handle vehicle pose updates"""
        with self.data_lock:
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            
            self.vehicle_position = [pos.x, pos.y]
            
            # Calculate heading from quaternion (yaw only)
            siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
            self.vehicle_heading = np.arctan2(siny_cosp, cosy_cosp)
            
            # Add to path with time-based decimation
            current_time = time.time()
            if (not self.vehicle_path or 
                current_time - self.last_update_time > 0.1):  # 10Hz path updates
                self.vehicle_path.append([pos.x, pos.y])
                self.last_update_time = current_time
    
    def outer_boundary_callback(self, msg):
        """Handle outer boundary updates"""
        with self.data_lock:
            self.outer_boundary = []
            for pose in msg.poses:
                self.outer_boundary.append([pose.pose.position.x, pose.pose.position.y])
            self.track_generator_active = True
            self.stats['last_track_update'] = time.time()
    
    def inner_boundary_callback(self, msg):
        """Handle inner boundary updates"""
        with self.data_lock:
            self.inner_boundary = []
            for pose in msg.poses:
                self.inner_boundary.append([pose.pose.position.x, pose.pose.position.y])
            self.track_generator_active = True
            self.stats['last_track_update'] = time.time()
    
    def centerline_callback(self, msg):
        """Handle centerline updates"""
        with self.data_lock:
            self.centerline = []
            for pose in msg.poses:
                self.centerline.append([pose.pose.position.x, pose.pose.position.y])
            self.track_generator_active = True
    
    def start_line_callback(self, msg):
        """Handle start line updates"""
        with self.data_lock:
            self.start_line = []
            for pose in msg.poses:
                self.start_line.append([pose.pose.position.x, pose.pose.position.y])
    
    def legacy_boundary_callback(self, msg):
        """Handle legacy combined boundary updates"""
        with self.data_lock:
            # Split combined boundary into left/right (basic implementation)
            all_points = []
            for pose in msg.poses:
                all_points.append([pose.pose.position.x, pose.pose.position.y])
            
            if len(all_points) > 4:
                # Simple split - first half = outer, second half = inner
                mid = len(all_points) // 2
                self.outer_boundary = all_points[:mid]
                self.inner_boundary = all_points[mid:]
                self.track_generator_active = True
    
    def get_plot_data(self):
        """Thread-safe data retrieval for plotting"""
        with self.data_lock:
            return {
                'local_cones': {k: v.copy() for k, v in self.local_cones.items()},
                'global_cones': {k: v.copy() for k, v in self.global_cones.items()},
                'vehicle_position': self.vehicle_position.copy(),
                'vehicle_heading': self.vehicle_heading,
                'vehicle_path': list(self.vehicle_path),
                'outer_boundary': self.outer_boundary.copy(),
                'inner_boundary': self.inner_boundary.copy(),
                'centerline': self.centerline.copy(),
                'start_line': self.start_line.copy(),
                'track_generator_active': self.track_generator_active,
                'stats': self.stats.copy()
            }

def create_realtime_plot():
    """Create real-time plotting with track generation support"""
    
    # Initialise ROS2
    rclpy.init()
    visualiser = ConeMapVisualiser()
    
    # Start ROS2 in separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(visualiser))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Set up matplotlib 
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(16, 12))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3, linewidth=0.5)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    
    # Performance tracking
    frame_times = deque(maxlen=20)
    last_frame_time = time.time()
    
    def update_plot(frame):
        nonlocal last_frame_time
        
        # Performance tracking
        current_time = time.time()
        frame_times.append(current_time - last_frame_time)
        last_frame_time = current_time
        fps = 1.0 / np.mean(frame_times) if frame_times else 0.0
        
        # Get latest data
        data = visualiser.get_plot_data()
        
        # Clear previous plots
        ax.clear()
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3, linewidth=0.5)
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        
        # Dynamic title with more information
        stats = data['stats']
        track_status = "Track Gen: ON" if data['track_generator_active'] else "Track Gen: OFF"
        title = (f'Formula Student cone map - Local: {stats["local_cone_count"]} | '
                f'Global: {stats["global_cone_count"]} | {track_status} | '
                f'FPS: {fps:.1f} | {time.strftime("%H:%M:%S")}')
        ax.set_title(title, fontsize=14, fontweight='bold')
        
        # Get vehicle position for centering
        vx, vy = data['vehicle_position'] if data['vehicle_position'] else [0, 0]
        
        # Plot track generation elements first (background)
        legend_elements = []
        
        # Plot outer boundary (blue cones boundary)
        if data['outer_boundary']:
            boundary_array = np.array(data['outer_boundary'])
            ax.plot(boundary_array[:, 0], boundary_array[:, 1], 'b-', 
                   linewidth=3, label='Outer Boundary (Blue)', alpha=0.8)
            legend_elements.append(plt.Line2D([0], [0], color='blue', linewidth=3, 
                                            label='Outer Boundary'))
        
        # Plot inner boundary (yellow cones boundary)
        if data['inner_boundary']:
            boundary_array = np.array(data['inner_boundary'])
            ax.plot(boundary_array[:, 0], boundary_array[:, 1], 'y-', 
                   linewidth=3, label='Inner Boundary (Yellow)', alpha=0.8)
            legend_elements.append(plt.Line2D([0], [0], color='yellow', linewidth=3, 
                                            label='Inner Boundary'))
        
        # Plot centerline
        if data['centerline']:
            centerline_array = np.array(data['centerline'])
            ax.plot(centerline_array[:, 0], centerline_array[:, 1], 'g-', 
                   linewidth=4, label='Centerline', alpha=0.9, linestyle='--')
            legend_elements.append(plt.Line2D([0], [0], color='green', linewidth=4, 
                                            linestyle='--', label='Centerline'))
        
        # Plot start line (orange cones)
        if data['start_line']:
            start_array = np.array(data['start_line'])
            ax.plot(start_array[:, 0], start_array[:, 1], 'orange', 
                   linewidth=6, label='Start Line', alpha=1.0)
            legend_elements.append(plt.Line2D([0], [0], color='orange', linewidth=6, 
                                            label='Start Line'))
        
        # Plot global cones (background layer)
        for color, cones in data['global_cones'].items():
            if cones:
                x_coords = [cone['x'] for cone in cones]
                y_coords = [cone['y'] for cone in cones]
                
                marker_color = {'blue': 'blue', 'yellow': 'gold', 'orange': 'orange'}[color]
                ax.scatter(x_coords, y_coords, c=marker_color, s=100, alpha=0.7, 
                          marker='o', label=f'Global {color}', 
                          edgecolors='white', linewidth=1.5)
        
        # Plot local cones (foreground layer)
        local_cone_count = 0
        for color, cones in data['local_cones'].items():
            if cones:
                x_coords = [cone['x'] for cone in cones]
                y_coords = [cone['y'] for cone in cones]
                confidences = [cone.get('confidence', 1.0) for cone in cones]
                
                # Create size array based on confidence
                sizes = [50 + conf * 100 for conf in confidences]
                marker_color = {'blue': 'blue', 'yellow': 'gold', 'orange': 'orange'}[color]
                
                # Plot with varying size and alpha based on confidence
                scatter = ax.scatter(x_coords, y_coords, c=marker_color, s=sizes, 
                                   alpha=0.9, marker='^', 
                                   label=f'Local {color}', 
                                   edgecolors='darkgray', linewidth=2)
                
                # Add confidence text for high-confidence cones
                if visualiser.show_confidence:
                    for x, y, conf in zip(x_coords, y_coords, confidences):
                        if conf > 0.6:  # Show confidence for medium+ confidence
                            ax.annotate(f'{conf:.1f}', (x, y), 
                                      xytext=(8, 8), textcoords='offset points',
                                      fontsize=9, alpha=0.8, color='white',
                                      bbox=dict(boxstyle='round,pad=0.2', 
                                              facecolor='black', alpha=0.6))
                
                local_cone_count += len(cones)
        
        # Plot vehicle position and heading
        if data['vehicle_position']:
            # Vehicle position 
            ax.scatter(vx, vy, c='red', s=400, marker='s', 
                      label='Vehicle', edgecolors='white', linewidth=3, zorder=10)
            
            # Vehicle heading arrow 
            heading = data['vehicle_heading']
            arrow_length = 4.0
            dx = arrow_length * np.cos(heading)
            dy = arrow_length * np.sin(heading)
            ax.arrow(vx, vy, dx, dy, head_width=1.0, head_length=0.8, 
                    fc='red', ec='white', linewidth=3, zorder=10, alpha=0.9)
            
            # Add coordinate and heading text
            ax.text(vx, vy - 3, f'({vx:.1f}, {vy:.1f})\n{np.degrees(heading):.0f}°', 
                   ha='center', va='top', fontsize=10, 
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='red', alpha=0.8),
                   color='white', fontweight='bold')
        
        # Plot vehicle path
        if visualiser.show_vehicle_path and len(data['vehicle_path']) > 1:
            path_array = np.array(data['vehicle_path'])
            ax.plot(path_array[:, 0], path_array[:, 1], 'r--', alpha=0.6, 
                   linewidth=2, label='Vehicle Path')
            legend_elements.append(plt.Line2D([0], [0], color='red', linestyle='--', 
                                            linewidth=2, label='Vehicle Path'))
        
        # Smart zoom control
        if data['vehicle_position']:
            # Calculate zoom based on cone distribution and track boundaries
            all_points = []
            
            # Add cone positions
            for color_cones in data['local_cones'].values():
                for cone in color_cones:
                    all_points.append([cone['x'], cone['y']])
            
            # Add track boundary points
            if data['outer_boundary']:
                all_points.extend(data['outer_boundary'])
            if data['inner_boundary']:
                all_points.extend(data['inner_boundary'])
            
            if all_points:
                points_array = np.array(all_points)
                # Calculate distances from vehicle
                distances = np.sqrt((points_array[:, 0] - vx)**2 + (points_array[:, 1] - vy)**2)
                max_distance = np.max(distances) if len(distances) > 0 else 10
                
                # Adaptive zoom: show all relevant points plus margin
                zoom_margin = min(25, max(10, max_distance * 1.2))
            else:
                zoom_margin = 15  # Default zoom when no data
            
            # Set limits centered on vehicle
            ax.set_xlim(vx - zoom_margin, vx + zoom_margin)
            ax.set_ylim(vy - zoom_margin, vy + zoom_margin)
            
            # Add compass rose
            compass_x = vx + zoom_margin * 0.85
            compass_y = vy + zoom_margin * 0.85
            ax.arrow(compass_x, compass_y, 0, 2, head_width=0.8, head_length=0.5, 
                    fc='cyan', ec='cyan', alpha=0.8)
            ax.text(compass_x, compass_y + 3, 'N', ha='center', fontsize=12, 
                   color='cyan', fontweight='bold')
        else:
            # Default view when no vehicle position
            ax.set_xlim(-15, 15)
            ax.set_ylim(-15, 15)
        
        # legend
        cone_legend_elements = []
        if any(len(cones) > 0 for cones in data['global_cones'].values()):
            for color in ['blue', 'yellow', 'orange']:
                if len(data['global_cones'][color]) > 0:
                    marker_color = {'blue': 'blue', 'yellow': 'gold', 'orange': 'orange'}[color]
                    cone_legend_elements.append(
                        plt.Line2D([0], [0], marker='o', color='w', 
                                 markerfacecolor=marker_color, markersize=8, 
                                 markeredgecolor='white', markeredgewidth=1.5,
                                 label=f'Global {color}', linestyle='None'))
        
        if any(len(cones) > 0 for cones in data['local_cones'].values()):
            for color in ['blue', 'yellow', 'orange']:
                if len(data['local_cones'][color]) > 0:
                    marker_color = {'blue': 'blue', 'yellow': 'gold', 'orange': 'orange'}[color]
                    cone_legend_elements.append(
                        plt.Line2D([0], [0], marker='^', color='w', 
                                 markerfacecolor=marker_color, markersize=10, 
                                 markeredgecolor='darkgray',
                                 label=f'Local {color}', linestyle='None'))
        
        if data['vehicle_position']:
            cone_legend_elements.append(
                plt.Line2D([0], [0], marker='s', color='w', 
                          markerfacecolor='red', markersize=12,
                          markeredgecolor='white', markeredgewidth=2,
                          label='Vehicle', linestyle='None'))
        
        # Combine all legend elements
        all_legend_elements = legend_elements + cone_legend_elements
        
        if all_legend_elements:
            ax.legend(handles=all_legend_elements, loc='upper left', 
                     bbox_to_anchor=(0.02, 0.98), fontsize=9, framealpha=0.9,
                     ncol=2 if len(all_legend_elements) > 8 else 1)
        
        # information panel
        info_text = []
        if data['vehicle_position']:
            info_text.append(f'Vehicle: ({vx:.1f}, {vy:.1f})')
            info_text.append(f'Heading: {np.degrees(data["vehicle_heading"]):.1f}°')
        
        info_text.append(f'Local cones: {local_cone_count}')
        info_text.append(f'Global cones: {sum(len(cones) for cones in data["global_cones"].values())}')
        
        if data['track_generator_active']:
            track_points = len(data['outer_boundary']) + len(data['inner_boundary']) + len(data['centerline'])
            info_text.append(f'Track points: {track_points}')
            
            # Time since last track update
            time_since_track = current_time - stats.get('last_track_update', current_time)
            info_text.append(f'Track age: {time_since_track:.1f}s')
        
        info_text.append(f'FPS: {fps:.1f}')
        
        # Position info panel
        ax.text(0.98, 0.02, '\n'.join(info_text), transform=ax.transAxes, 
               verticalalignment='bottom', horizontalalignment='right',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='black', alpha=0.8),
               fontsize=9, color='white')
    
    def on_key(event):
        """Handle keyboard events for interactive control"""
        if event.key == '+' or event.key == '=':
            visualiser.zoom_level *= 0.8  # Zoom in
        elif event.key == '-':
            visualiser.zoom_level *= 1.25  # Zoom out
        elif event.key == 'r':
            visualiser.auto_zoom = not visualiser.auto_zoom
            print(f"Auto zoom: {'ON' if visualiser.auto_zoom else 'OFF'}")
        elif event.key == 'c':
            visualiser.show_confidence = not visualiser.show_confidence
            print(f"Show confidence: {'ON' if visualiser.show_confidence else 'OFF'}")
        elif event.key == 'p':
            visualiser.show_vehicle_path = not visualiser.show_vehicle_path
            print(f"Show vehicle path: {'ON' if visualiser.show_vehicle_path else 'OFF'}")
    
    # Connect keyboard events
    fig.canvas.mpl_connect('key_press_event', on_key)
    
    # Add instructions
    instructions = [
        'Controls: +/- zoom, R auto-zoom, C confidence, P path',
        'Blue=Outer, Yellow=Inner, Orange=Start, Green=Centerline'
    ]
    fig.text(0.02, 0.02, ' | '.join(instructions), fontsize=9, alpha=0.7)
    
    # Create animation with adaptive update rate
    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualiser.destroy_node()
        rclpy.shutdown()

def main(args=None):
    """Main entry point for ROS2 console script"""
    create_realtime_plot()

if __name__ == '__main__':
    main()