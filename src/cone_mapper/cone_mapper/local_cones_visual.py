#!/usr/bin/env python3
"""
Simple Local Cone Map Visualizer
Shows real-time local cone detections with confidence
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import threading
import time

class LocalConeVisualizer(Node):
    """Simple visualizer for local cone detections"""
    
    def __init__(self):
        super().__init__('local_cone_visualizer')
        
        # Data storage
        self.local_cones = {'blue': [], 'yellow': [], 'orange': []}
        self.vehicle_position = [0.0, 0.0]
        self.vehicle_heading = 0.0
        
        # Thread lock for data safety
        self.data_lock = threading.Lock()
        
        # Subscriptions
        self.local_sub = self.create_subscription(
            String, '/cone_map/local', self.local_cone_callback, 10)
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/slam', self.pose_callback, 10)
        
        self.get_logger().info('Local Cone Visualizer initialized')
    
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
                            'x': x, 'y': y, 'confidence': confidence
                        })
                    except (ValueError, IndexError):
                        continue
    
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
    
    def get_plot_data(self):
        """Thread-safe data retrieval for plotting"""
        with self.data_lock:
            return {
                'local_cones': {k: v.copy() for k, v in self.local_cones.items()},
                'vehicle_position': self.vehicle_position.copy(),
                'vehicle_heading': self.vehicle_heading
            }

def create_local_plot():
    """Create and run local cone visualization"""
    
    # Initialize ROS2
    rclpy.init()
    visualizer = LocalConeVisualizer()
    
    # Start ROS2 in separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(visualizer))
    ros_thread.daemon = True
    ros_thread.start()
    
    # Set up matplotlib
    fig, ax = plt.subplots(figsize=(12, 8))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('Local Cone Map (Real-time Detections)', fontsize=14, fontweight='bold')
    
    def update_plot(frame):
        # Get latest data
        data = visualizer.get_plot_data()
        
        # Clear previous plots
        ax.clear()
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Local Cone Map (Real-time Detections)', fontsize=14, fontweight='bold')
        
        # Get vehicle position for centering
        vx, vy = data['vehicle_position'] if data['vehicle_position'] else [0, 0]
        
        # Plot local cones
        total_cones = 0
        for color, cones in data['local_cones'].items():
            if cones:
                x_coords = [cone['x'] for cone in cones]
                y_coords = [cone['y'] for cone in cones]
                confidences = [cone['confidence'] for cone in cones]
                
                # Size based on confidence (larger = higher confidence)
                sizes = [50 + conf * 100 for conf in confidences]
                
                # Plot cones
                ax.scatter(x_coords, y_coords, c=color, s=sizes, 
                          alpha=0.8, marker='o', label=f'{color.title()} cones',
                          edgecolors='black', linewidth=1)
                
                total_cones += len(cones)
        
        # Plot vehicle position and heading
        if data['vehicle_position']:
            # Vehicle position
            ax.scatter(vx, vy, c='red', s=200, marker='s', 
                      label='Vehicle', edgecolors='white', linewidth=2, zorder=10)
            
            # Vehicle heading arrow
            heading = data['vehicle_heading']
            arrow_length = 2.0
            dx = arrow_length * np.cos(heading)
            dy = arrow_length * np.sin(heading)
            ax.arrow(vx, vy, dx, dy, head_width=0.5, head_length=0.3, 
                    fc='red', ec='red', linewidth=2, zorder=10)
        
        # Set zoom around vehicle
        zoom = 15.0  # meters
        ax.set_xlim(vx - zoom, vx + zoom)
        ax.set_ylim(vy - zoom, vy + zoom)
        
        # Add legend and info
        ax.legend(loc='upper right')
        
        # Add cone count
        ax.text(0.02, 0.98, f'Total Local Cones: {total_cones}', 
               transform=ax.transAxes, fontsize=10,
               bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
               verticalalignment='top')
    
    # Create animation
    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    create_local_plot()