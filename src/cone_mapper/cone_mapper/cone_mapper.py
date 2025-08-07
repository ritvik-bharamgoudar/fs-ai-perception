#!/usr/bin/env python3
"""
Cone Mapper with coordinate transformation
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import numpy as np
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation as R
import time
import math
import json
from typing import List, Tuple, Dict, Optional

# Color constants
BLUE_CONE = 0
YELLOW_CONE = 1  
ORANGE_CONE = 2

class PersistentGlobalMap:
    """Persistent global cone map with adjusted thresholds"""
    
    def __init__(self, confidence_threshold=0.7, min_detections=3):  # Balanced thresholds
        self.global_cones = []
        self.confidence_threshold = confidence_threshold
        self.min_detections = min_detections
        self.cone_id_counter = 0
        
    def try_add_cone(self, cone_data):
        """Try to add a cone to global map if it meets criteria"""
        if (cone_data['confidence'] > self.confidence_threshold and 
            cone_data['detections'] >= self.min_detections):
            
            # Check if already in global map
            for existing in self.global_cones:
                if (existing['color'] == cone_data['color'] and
                    np.linalg.norm([existing['x'] - cone_data['x'], 
                                   existing['y'] - cone_data['y']]) < 1.5):  # Slightly larger tolerance
                    return False  # Already exists
            
            # Add to global map
            self.cone_id_counter += 1
            global_cone = {
                'id': self.cone_id_counter,
                'x': cone_data['x'],
                'y': cone_data['y'], 
                'z': cone_data['z'],
                'color': cone_data['color'],
                'confidence': cone_data['confidence'],
                'detections': cone_data['detections'],
                'added_timestamp': time.time()
            }
            self.global_cones.append(global_cone)
            return True
        return False
    
    def get_global_map(self):
        return self.global_cones.copy()
    
    def get_local_view(self, vehicle_pos, radius=30.0):
        veh_x, veh_y = vehicle_pos
        local_cones = []
        
        for cone in self.global_cones:
            distance = math.sqrt((cone['x'] - veh_x)**2 + (cone['y'] - veh_y)**2)
            if distance <= radius:
                local_cones.append(cone)
        
        return local_cones
    
    def save_to_file(self, filename):
        with open(filename, 'w') as f:
            json.dump(self.global_cones, f, indent=2)
    
    def get_stats(self):
        color_counts = {BLUE_CONE: 0, YELLOW_CONE: 0, ORANGE_CONE: 0}
        for cone in self.global_cones:
            color_counts[cone['color']] += 1
        
        return {
            'total_cones': len(self.global_cones),
            'blue_cones': color_counts[BLUE_CONE],
            'yellow_cones': color_counts[YELLOW_CONE],
            'orange_cones': color_counts[ORANGE_CONE]
        }

class LocalConeBuffer:
    """Sliding window buffer with parameters"""
    
    def __init__(self, max_size=200, max_age=6.0):
        self.cones = []
        self.max_size = max_size
        self.max_age = max_age
        self.cone_id_counter = 0
        
    def add_cone_detection(self, x, y, z, color, confidence=1.0):
        """Add detection with distance-based confidence"""
        current_time = time.time()
        
        # Calculate distance for confidence adjustment
        distance = np.sqrt(x*x + y*y + z*z)
        
        # Find matching cone
        matching_idx = self._find_matching_cone(x, y, color)
        
        if matching_idx is not None:
            # Update existing
            cone = self.cones[matching_idx]
            cone['x'] = 0.3 * x + 0.7 * cone['x']  # EMA
            cone['y'] = 0.3 * y + 0.7 * cone['y']
            cone['z'] = 0.3 * z + 0.7 * cone['z']
            
            # Distance-based confidence gain - closer cones gain more confidence
            confidence_gain = 0.2 if distance < 5.0 else 0.15 if distance < 10.0 else 0.1
            cone['confidence'] = min(1.0, cone['confidence'] + confidence_gain)
            cone['detections'] += 1
            cone['last_seen'] = current_time
        else:
            # Add new cone with distance based initial confidence
            if distance < 3.0:
                initial_conf = 0.6
            elif distance < 8.0:
                initial_conf = 0.4
            else:
                initial_conf = 0.3
                
            self.cone_id_counter += 1
            self.cones.append({
                'id': self.cone_id_counter,
                'x': x, 'y': y, 'z': z, 'color': color,
                'confidence': initial_conf,
                'detections': 1,
                'first_seen': current_time,
                'last_seen': current_time
            })
    
    '''
    def update_frame(self):
        """Update confidence and prune old/low-confidence cones based on age"""
        current_time = time.time()
        
        # Decay confidence for unseen cones
        for cone in self.cones:
            age = current_time - cone['last_seen']
            if age > 0.1:  # Not seen this frame
                cone['confidence'] = max(0.0, cone['confidence'] - 0.04)
        
        # Remove old or low-confidence cones
        self.cones = [cone for cone in self.cones 
                     if (current_time - cone['first_seen'] < self.max_age and
                         cone['confidence'] > 0.15)]
        
        # Limit size
        if len(self.cones) > self.max_size:
            self.cones.sort(key=lambda x: x['confidence'], reverse=True)
            self.cones = self.cones[:self.max_size]
    '''

    def cull_by_position(self, vehicle_pos):
        vehicle_x, vehicle_y = vehicle_pos
        self.cones = [cone for cone in self.cones 
                  if self._is_spatially_relevant(cone, vehicle_x, vehicle_y)]

    def _is_spatially_relevant(self, cone, veh_x, veh_y):
        # Keep cones in moving window: 10m behind, 30m ahead, 15m either side
        relative_x = cone['x'] - veh_x  # Forward/backward relative to car
        relative_y = cone['y'] - veh_y  # Left/right relative to car
    
        return (-5 < relative_x < 20) and (abs(relative_y) < 10)

        
    
    def get_all_cones(self):
        return self.cones.copy()
    
    def get_high_confidence_cones(self, threshold=0.6):  # Low threshold
        return [cone for cone in self.cones if cone['confidence'] > threshold]
    
    def _find_matching_cone(self, x, y, color, radius=2.0):  # Increased radius
        candidates = [(i, cone) for i, cone in enumerate(self.cones) 
                     if cone['color'] == color]
        
        if not candidates:
            return None
        
        positions = np.array([[cone['x'], cone['y']] for _, cone in candidates])
        tree = KDTree(positions)
        
        dist, local_idx = tree.query([x, y])
        
        if dist < radius:
            global_idx, _ = candidates[local_idx]
            return global_idx
        
        return None

class ConeMapperNode(Node):
    """Cone mapper - maps detected cones from camera frame to global frame"""
    
    def __init__(self):
        super().__init__('cone_mapper')
        
        # Initialise mapping components 
        self.global_map = PersistentGlobalMap(
            confidence_threshold=0.8,  # Higher equals stricter position requirements
            min_detections=3          # Number of frames a cone is in
        )
        self.local_buffer = LocalConeBuffer()
        
        # Vehicle state
        self.latest_pose = None
        self.vehicle_position = (0.0, 0.0)
        self.vehicle_heading = 0.0
        
        # Subscriptions
        self.cone_sub = self.create_subscription(
            String, '/detected_cones', self.cone_callback, 10)
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/slam', self.pose_callback, 10)
        
        # Publishers
        self.local_map_pub = self.create_publisher(String, '/cone_map/local', 10)
        self.global_map_pub = self.create_publisher(String, '/cone_map/global', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/cone_map/markers', 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/mapping/diagnostics', 10)
        
        # Timers
        self.local_timer = self.create_timer(0.05, self.publish_local_map)
        self.global_timer = self.create_timer(0.5, self.publish_global_map)
        self.diagnostics_timer = self.create_timer(1.0, self.publish_diagnostics)
        
        # Statistics
        self.stats = {
            'total_detections': 0,
            'global_additions': 0,
            'processing_times': [],
            'coordinate_warnings': 0
        }
        
        self.get_logger().info('Cone Mapper Node initialised')
    
    def pose_callback(self, msg):
        """Handle pose updates with validation"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Validate position
        if (np.isnan(pos.x) or np.isnan(pos.y) or np.isnan(pos.z) or
            np.isinf(pos.x) or np.isinf(pos.y) or np.isinf(pos.z)):
            return
        
        # Validate quaternion
        quat = np.array([ori.x, ori.y, ori.z, ori.w])
        if np.any(np.isnan(quat)) or np.any(np.isinf(quat)) or np.allclose(quat, 0):
            return
        
        # Normalize quaternion
        quat = quat / np.linalg.norm(quat)
        
        self.latest_pose = {
            'position': np.array([pos.x, pos.y, pos.z]),
            'orientation': quat,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }
        
        self.vehicle_position = (pos.x, pos.y)

        #Calculate vehicle heading from quaternion
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        self.vehicle_heading = np.arctan2(siny_cosp, cosy_cosp)
    
    def cone_callback(self, msg):
        """Handle cone detections with coordinate transformation"""
        if self.latest_pose is None:
            return
        
        start_time = time.time()
        
        try:
            # Get vehicle pose components
            vehicle_pos = self.latest_pose['position']
            vehicle_quat = self.latest_pose['orientation']
            
            # Create rotation matrix from vehicle/robot to world
            rot = R.from_quat(vehicle_quat)
            R_world_vehicle = rot.as_matrix()  # Rotation from vehicle to world
            t_world_vehicle = vehicle_pos.reshape(3, 1)
            
            # Process each detection
            lines = msg.data.strip().split('\n')
            valid_detections = 0
            
            for line in lines:
                parts = line.strip().split(',')
                if len(parts) != 4:
                    continue
                
                x_cam, y_cam, z_cam, color = map(float, parts)
                
                # Validate camera coordinates
                if (np.isnan(x_cam) or np.isnan(y_cam) or np.isnan(z_cam) or
                    np.isinf(x_cam) or np.isinf(y_cam) or np.isinf(z_cam)):
                    self.stats['coordinate_warnings'] += 1
                    continue
                
                # Additional validation - reject unreasonable values
                if abs(x_cam) > 50 or abs(y_cam) > 50 or z_cam < 0.1 or z_cam > 30:
                    self.stats['coordinate_warnings'] += 1
                    continue
                
                # CORRECT COORDINATE TRANSFORMATION
                # Step 1: Convert camera coordinates to robot/vehicle coordinates
                # ZED Camera: X=right, Y=down, Z=forward
                # car frame: X=forward, Y=left, Z=up
                x_robot = z_cam    # Camera Z (forward) -> car X (forward)
                y_robot = -x_cam   # Camera X (right) -> car -Y (left)
                z_robot = -y_cam   # Camera Y (down) -> car -Z (up)
                
                # Step 2: Transform robot coordinates to world coordinates
                X_robot = np.array([[x_robot], [y_robot], [z_robot]])
                X_world = R_world_vehicle @ X_robot + t_world_vehicle
                
                # Extract world coordinates
                x_world, y_world, z_world = X_world[0, 0], X_world[1, 0], X_world[2, 0]
                
                # Add to local buffer
                self.local_buffer.add_cone_detection(x_world, y_world, z_world, int(color))
                
                self.stats['total_detections'] += 1
                valid_detections += 1
            
            # Log detection summary
            if valid_detections > 0:
                self.get_logger().info(f'Processed {valid_detections} valid cone detections')
            
            # Update local buffer
            #self.local_buffer.update_frame()
            self.local_buffer.cull_by_position(self.vehicle_position)
            
            # Try to promote high-confidence cones to global map
            promoted_count = 0
            for cone in self.local_buffer.get_high_confidence_cones():
                if self.global_map.try_add_cone(cone):
                    self.stats['global_additions'] += 1
                    promoted_count += 1
                    self.get_logger().info(
                        f'Added cone {cone["id"]} to global map at '
                        f'({cone["x"]:.1f}, {cone["y"]:.1f}) with confidence {cone["confidence"]:.2f}')
            
            if promoted_count > 0:
                self.get_logger().info(f'Promoted {promoted_count} cones to global map')
            
            # Track processing time
            self.stats['processing_times'].append(time.time() - start_time)
            if len(self.stats['processing_times']) > 100:
                self.stats['processing_times'] = self.stats['processing_times'][-100:]
            
        except Exception as e:
            self.get_logger().error(f'Error processing cone detections: {e}')
    
    def publish_local_map(self):
        """Publish local cone map"""
        local_cones = self.local_buffer.get_all_cones()
        
        if not local_cones:
            return
        
        # Create message
        output_lines = []
        for cone in local_cones:
            output_lines.append(
                f"{cone['x']:.2f},{cone['y']:.2f},{cone['z']:.2f},"
                f"{cone['color']},{cone['confidence']:.2f}")
        
        msg = String()
        msg.data = '\n'.join(output_lines)
        self.local_map_pub.publish(msg)
    
    def publish_global_map(self):
        """Publish global cone map"""
        global_cones = self.global_map.get_global_map()
        
        if not global_cones:
            return
        
        # Create message
        output_lines = []
        for cone in global_cones:
            output_lines.append(
                f"{cone['x']:.2f},{cone['y']:.2f},{cone['z']:.2f},{cone['color']}")
        
        msg = String()
        msg.data = '\n'.join(output_lines)
        self.global_map_pub.publish(msg)
        
        # Log stats
        stats = self.global_map.get_stats()
        self.get_logger().info(f'Global map: {stats["total_cones"]} cones total')

    def publish_diagnostics(self):
        """Publish system diagnostics"""
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'cone_mapping'
        status.hardware_id = 'cone_mapper'
        
        # Calculate metrics
        if self.stats['processing_times']:
            avg_time = np.mean(self.stats['processing_times']) * 1000
            max_time = np.max(self.stats['processing_times']) * 1000
        else:
            avg_time = max_time = 0
        
        local_count = len(self.local_buffer.get_all_cones())
        global_count = len(self.global_map.get_global_map())
        
        # Set status
        if self.stats['coordinate_warnings'] > 50:
            status.level = DiagnosticStatus.WARN
            status.message = 'High coordinate warning count'
        elif avg_time > 20:
            status.level = DiagnosticStatus.WARN
            status.message = 'High processing time'
        elif local_count > 150:
            status.level = DiagnosticStatus.WARN
            status.message = 'High local cone count'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'System healthy'
        
        # Add metrics
        metrics = {
            'total_detections': self.stats['total_detections'],
            'global_additions': self.stats['global_additions'],
            'local_cone_count': local_count,
            'global_cone_count': global_count,
            'coordinate_warnings': self.stats['coordinate_warnings'],
            'avg_processing_time_ms': avg_time,
            'max_processing_time_ms': max_time
        }
        
        for key, value in metrics.items():
            kv = KeyValue()
            kv.key = key
            kv.value = str(value)
            status.values.append(kv)
        
        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = ConeMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save global map on exit
        #node.global_map.save_to_file('final_cone_map.json')
        #node.get_logger().info('Saved global map to final_cone_map.json')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()