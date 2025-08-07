#!/usr/bin/env python3
"""
Robust Track Generator Node - Builds on cone mapping data
Uses Formula Student track constraints for robust track generation

Track structure:
- Blue cones = OUTER boundary (always outside)
- Yellow cones = INNER boundary (always inside) 
- Orange cones = Start/finish line (perpendicular pair)
- Track width ≈ ~3m, cone spacing ~ 1-2m
- Closed loop track for autocross
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import splprep, splev
import math
from typing import List, Tuple, Optional

# Color constants
BLUE_CONE = 0    # OUTER boundary
YELLOW_CONE = 1  # INNER boundary
ORANGE_CONE = 2  # Start/finish line

class TrackGeneratorNode(Node):
    """
    Robust track generator optimized for Formula Student constraints:
    - Blue = outer boundary, Yellow = inner boundary (ALWAYS)
    - Orange = start/finish markers (perpendicular pair)
    - ~10-12 visible cones, 3m track width, 1-2m cone spacing
    - Optimized for clean, efficient code over speed
    """
    
    def __init__(self):
        super().__init__('track_generator')
        
        # Track state
        self.vehicle_position = (0.0, 0.0)
        self.vehicle_heading = 0.0
        self.track_initialized = False
        
        # Start line detection
        self.start_line_detected = False
        self.start_line_position = None
        self.start_line_heading = None
        
        # Track memory for temporal consistency
        self.previous_outer_boundary = None
        self.previous_inner_boundary = None
        self.previous_centerline = None
        
        # Track statistics for optimization
        self.track_stats = {
            'avg_track_width': 3.0,
            'cone_density': 0.0,
            'track_curvature': 0.0
        }

        self.use_local_cones = True  # Set to False to use global cones
        
        # Subscriptions
        self.cone_sub = self.create_subscription(
            String, '/cone_map/global', self.cone_callback, 10)
        self.pose_sub = self.create_subscription(
            Odometry, '/odometry/slam', self.pose_callback, 10)
        if self.use_local_cones:
            self.cone_sub = self.create_subscription(
                String, '/cone_map/local', self.cone_callback, 10)
            self.get_logger().info('Track Generator using local cones')
        else:
            self.cone_sub = self.create_subscription(
                String, '/cone_map/global', self.cone_callback, 10)
            self.get_logger().info('Track Generator using global cones')
        
        # Publishers
        self.outer_boundary_pub = self.create_publisher(Path, '/track/outer_boundary', 10)  # Blue cones
        self.inner_boundary_pub = self.create_publisher(Path, '/track/inner_boundary', 10)  # Yellow cones
        self.centerline_pub = self.create_publisher(Path, '/track/centerline', 10)
        self.start_line_pub = self.create_publisher(Path, '/track/start_line', 10)  # Orange cones
        
        # Timer
        self.track_timer = self.create_timer(0.5, self.generate_track)  # 2Hz
        
        # Current cone data
        self.current_cones = []
        
        self.get_logger().info('Track Generator Node initialised')

        def declare_parameters(self):
            self.declare_parameter('use_local_cones', True)
            self.use_local_cones = self.get_parameter('use_local_cones').valu
    
    def pose_callback(self, msg):
        """Update vehicle state"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        self.vehicle_position = (pos.x, pos.y)
        
        # Calculate heading from quaternion
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        self.vehicle_heading = np.arctan2(siny_cosp, cosy_cosp)
    
    def cone_callback(self, msg):
        """Process cone data from cone mapper"""
        self.current_cones = []
        
        if not msg.data.strip():
            return
            
        lines = msg.data.strip().split('\n')
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) >= 4:
                try:
                    x, y, z, color = float(parts[0]), float(parts[1]), float(parts[2]), int(parts[3])
                    self.current_cones.append({
                        'x': x, 'y': y, 'z': z, 'color': color
                    })
                except ValueError:
                    continue
    
    def generate_track(self):
        """Main track generation function - optimized for FS constraints"""
        if len(self.current_cones) < 3:  # Need minimum cones
            return
        
        # Detect start line first (orange cones)
        if not self.start_line_detected:
            self.detect_start_line()
        
        # Get local cones (optimized for ~10-12 cone expectation)
        local_cones = self.get_local_cones(radius=25.0)  # Adjusted for FS track size
        
        if len(local_cones) < 3:
            return
        
        # Separate by color - MUCH simpler now with clear rules
        outer_cones = [c for c in local_cones if c['color'] == BLUE_CONE]   # Blue = outer
        inner_cones = [c for c in local_cones if c['color'] == YELLOW_CONE] # Yellow = inner
        orange_cones = [c for c in local_cones if c['color'] == ORANGE_CONE] # Orange = start/finish
        
        # Need at least one cone per boundary
        if len(outer_cones) < 1 and len(inner_cones) < 1:
            self.get_logger().warn('Insufficient boundary cones for track generation')
            return
        
        # Generate boundaries using Formula Student optimized approach
        outer_boundary = self.generate_fs_boundary(outer_cones, is_outer=True)
        inner_boundary = self.generate_fs_boundary(inner_cones, is_outer=False)
        
        # Handle missing boundaries (common with sparse cones)
        if outer_boundary is None and inner_boundary is not None:
            outer_boundary = self.extrapolate_missing_boundary(inner_boundary, offset=3.0)
        elif inner_boundary is None and outer_boundary is not None:
            inner_boundary = self.extrapolate_missing_boundary(outer_boundary, offset=-3.0)
        elif outer_boundary is None and inner_boundary is None:
            return
        
        # Validate track geometry with FS constraints
        if not self.validate_fs_track(outer_boundary, inner_boundary):
            self.get_logger().warn('Track validation failed, using fallback')
            outer_boundary, inner_boundary = self.generate_fs_fallback_track()
        
        # Generate optimized centerline
        centerline = self.generate_racing_line(outer_boundary, inner_boundary)
        
        # Temporal smoothing for consistency (important for control systems)
        if self.track_initialized:
            outer_boundary = self.temporal_smoothing(outer_boundary, self.previous_outer_boundary)
            inner_boundary = self.temporal_smoothing(inner_boundary, self.previous_inner_boundary)
            centerline = self.temporal_smoothing(centerline, self.previous_centerline)
        
        # Update track statistics for adaptive behavior
        self.update_track_statistics(outer_boundary, inner_boundary, local_cones)
        
        # Store for next iteration
        self.previous_outer_boundary = outer_boundary
        self.previous_inner_boundary = inner_boundary
        self.previous_centerline = centerline
        self.track_initialized = True
        
        # Publish tracks
        self.publish_boundary(self.outer_boundary_pub, outer_boundary)
        self.publish_boundary(self.inner_boundary_pub, inner_boundary)
        self.publish_boundary(self.centerline_pub, centerline)
        
        # Log for debugging/optimization
        self.get_logger().info(
            f'Track: {len(outer_cones)} outer, {len(inner_cones)} inner cones. '
            f'Width: {self.track_stats["avg_track_width"]:.1f}m'
        )
    
    def get_local_cones(self, radius=30.0):
        """Get cones within radius of vehicle"""
        vx, vy = self.vehicle_position
        local_cones = []
        
        for cone in self.current_cones:
            distance = math.sqrt((cone['x'] - vx)**2 + (cone['y'] - vy)**2)
            if distance <= radius:
                local_cones.append(cone)
        
        return local_cones
    
    def detect_start_line(self):
        """Detect start/finish line from orange cone pair"""
        orange_cones = [c for c in self.current_cones if c['color'] == ORANGE_CONE]
        
        if len(orange_cones) >= 2:
            # Find the pair that's most likely the start line (closest together)
            min_distance = float('inf')
            start_pair = None
            
            for i in range(len(orange_cones)):
                for j in range(i+1, len(orange_cones)):
                    cone1, cone2 = orange_cones[i], orange_cones[j]
                    distance = math.sqrt((cone1['x'] - cone2['x'])**2 + (cone1['y'] - cone2['y'])**2)
                    
                    # Start line should be ~3-4m apart (track width + margin)
                    if 2.5 <= distance <= 5.0 and distance < min_distance:
                        min_distance = distance
                        start_pair = (cone1, cone2)
            
            if start_pair:
                # Calculate start line position and orientation
                cone1, cone2 = start_pair
                self.start_line_position = (
                    (cone1['x'] + cone2['x']) / 2,
                    (cone1['y'] + cone2['y']) / 2
                )
                
                # Start line heading (perpendicular to cone pair)
                dx = cone2['x'] - cone1['x']
                dy = cone2['y'] - cone1['y']
                self.start_line_heading = math.atan2(-dx, dy)  # Perpendicular
                
                self.start_line_detected = True
                self.get_logger().info(f'Start line detected at {self.start_line_position}')
                
                # Publish start line
                start_line_points = np.array([[cone1['x'], cone1['y']], [cone2['x'], cone2['y']]])
                self.publish_boundary(self.start_line_pub, start_line_points)
    
    def generate_fs_boundary(self, cones, is_outer=True):
        """Generate boundary optimized for Formula Student constraints"""
        if len(cones) == 0:
            return None
        elif len(cones) == 1:
            return self.generate_single_cone_boundary(cones[0], is_outer)
        elif len(cones) == 2:
            return self.generate_two_cone_boundary(cones[0], cones[1])
        else:
            return self.generate_multi_cone_boundary(cones, is_outer)
    
    def generate_single_cone_boundary(self, cone, is_outer):
        """Generate boundary from single cone using vehicle heading"""
        vx, vy = self.vehicle_position
        heading = self.vehicle_heading
        
        # Project boundary from cone in direction of travel
        cone_pos = np.array([cone['x'], cone['y']])
        forward = np.array([np.cos(heading), np.sin(heading)])
        
        # Create boundary extending from cone
        boundary_points = []
        for distance in np.linspace(-5, 15, 8):  # 5m behind to 15m ahead
            point = cone_pos + forward * distance
            boundary_points.append(point)
        
        return np.array(boundary_points)
    
    def generate_two_cone_boundary(self, cone1, cone2):
        """Generate boundary from two cones with extrapolation"""
        p1 = np.array([cone1['x'], cone1['y']])
        p2 = np.array([cone2['x'], cone2['y']])
        
        # Direction vector between cones
        direction = p2 - p1
        direction_norm = np.linalg.norm(direction)
        
        if direction_norm > 0:
            direction = direction / direction_norm
            
            # Extrapolate beyond the cones
            boundary_points = []
            
            # Extend backwards from first cone
            for i in range(3):
                point = p1 - direction * (i + 1) * 2.0
                boundary_points.append(point)
            
            boundary_points.reverse()  # Put in correct order
            
            # Add the actual cone positions
            boundary_points.extend([p1, p2])
            
            # Extend forwards from second cone
            for i in range(3):
                point = p2 + direction * (i + 1) * 2.0
                boundary_points.append(point)
            
            return np.array(boundary_points)
        else:
            # Cones too close, treat as single cone
            return self.generate_single_cone_boundary(cone1, True)
    
    def generate_multi_cone_boundary(self, cones, is_outer):
        """Generate boundary from multiple cones using robust fitting"""
        # Sort cones by progress along track
        sorted_cones = self.sort_cones_by_track_progress(cones)
        
        # Extract positions
        positions = np.array([[c['x'], c['y']] for c in sorted_cones])
        
        # Remove obvious outliers (simple distance-based filter)
        filtered_positions = self.filter_cone_outliers(positions)
        
        if len(filtered_positions) < 2:
            return self.generate_single_cone_boundary(sorted_cones[0], is_outer)
        
        # Fit smooth boundary
        try:
            return self.fit_smooth_boundary(filtered_positions)
        except Exception as e:
            self.get_logger().warn(f'Spline fitting failed: {e}, using linear interpolation')
            return self.linear_interpolation(filtered_positions)
    
    def sort_cones_by_track_progress(self, cones):
        """Sort cones by progress around track using vehicle position as reference"""
        vx, vy = self.vehicle_position
        
        # Calculate angle from vehicle to each cone
        cone_angles = []
        for cone in cones:
            angle = math.atan2(cone['y'] - vy, cone['x'] - vx)
            cone_angles.append((angle, cone))
        
        # Sort by angle (this gives rough track order)
        return [cone for _, cone in sorted(cone_angles)]
    
    def filter_cone_outliers(self, positions):
        """Remove cone positions that are obviously wrong"""
        if len(positions) < 3:
            return positions
        
        filtered = []
        
        for i, pos in enumerate(positions):
            # Check distance to nearest neighbors
            distances = []
            for j, other_pos in enumerate(positions):
                if i != j:
                    dist = np.linalg.norm(pos - other_pos)
                    distances.append(dist)
            
            min_dist = min(distances) if distances else 0
            
            # Keep cone if it's not too far from others (max 5m to nearest neighbor)
            if min_dist <= 5.0:
                filtered.append(pos)
        
        return np.array(filtered) if len(filtered) > 0 else positions
    
    def extrapolate_missing_boundary(self, known_boundary, offset):
        """Create missing boundary by offsetting known boundary"""
        if known_boundary is None or len(known_boundary) < 2:
            return None
        
        # Calculate perpendicular offset direction for each point
        boundary_points = []
        
        for i in range(len(known_boundary)):
            if i == 0:
                # First point: use direction to next point
                direction = known_boundary[i+1] - known_boundary[i]
            elif i == len(known_boundary) - 1:
                # Last point: use direction from previous point
                direction = known_boundary[i] - known_boundary[i-1]
            else:
                # Middle point: use average direction
                dir1 = known_boundary[i] - known_boundary[i-1]
                dir2 = known_boundary[i+1] - known_boundary[i]
                direction = (dir1 + dir2) / 2
            
            # Normalize and get perpendicular
            if np.linalg.norm(direction) > 0:
                direction = direction / np.linalg.norm(direction)
                perpendicular = np.array([-direction[1], direction[0]])  # 90° rotation
                
                # Offset point
                offset_point = known_boundary[i] + perpendicular * offset
                boundary_points.append(offset_point)
        
        return np.array(boundary_points)
    
    def generate_racing_line(self, outer_boundary, inner_boundary):
        """Generate optimal racing line (not just midpoint)"""
        if outer_boundary is None or inner_boundary is None:
            return None
        
        # For now, use midpoint (could be enhanced with racing line optimization)
        return self.generate_centerline(outer_boundary, inner_boundary)
    
    def update_track_statistics(self, outer_boundary, inner_boundary, cones):
        """Update adaptive track statistics for optimization"""
        if outer_boundary is not None and inner_boundary is not None:
            # Calculate average track width
            widths = []
            min_len = min(len(outer_boundary), len(inner_boundary))
            for i in range(min_len):
                width = np.linalg.norm(outer_boundary[i] - inner_boundary[i])
                widths.append(width)
            
            if widths:
                self.track_stats['avg_track_width'] = np.mean(widths)
        
        # Calculate cone density
        if len(cones) > 0:
            # Estimate track length in view
            track_length = 20.0  # Approximate based on viewing radius
            self.track_stats['cone_density'] = len(cones) / track_length
    
    def validate_fs_track(self, outer_boundary, inner_boundary):
        """Validate track using Formula Student specific constraints"""
        if outer_boundary is None or inner_boundary is None:
            return False
        
        if len(outer_boundary) < 2 or len(inner_boundary) < 2:
            return False
        
        # Check track width constraints (FS: ~3m ± 1m tolerance)
        min_len = min(len(outer_boundary), len(inner_boundary))
        for i in range(min_len):
            width = np.linalg.norm(outer_boundary[i] - inner_boundary[i])
            if width < 1.5 or width > 5.0:  # Allow reasonable tolerance
                return False
        
        return True
    
    def generate_fs_fallback_track(self):
        """Generate Formula Student specific fallback track"""
        vx, vy = self.vehicle_position
        heading = self.vehicle_heading
        
        # Generate straight track with FS dimensions
        distances = np.linspace(0, 15, 8)  # 15m ahead
        outer_boundary = []
        inner_boundary = []
        
        for dist in distances:
            center_x = vx + dist * np.cos(heading)
            center_y = vy + dist * np.sin(heading)
            
            # 3m track width (FS standard)
            outer_x = center_x - 1.5 * np.sin(heading)
            outer_y = center_y + 1.5 * np.cos(heading)
            
            inner_x = center_x + 1.5 * np.sin(heading)
            inner_y = center_y - 1.5 * np.cos(heading)
            
            outer_boundary.append([outer_x, outer_y])
            inner_boundary.append([inner_x, inner_y])
        
        return np.array(outer_boundary), np.array(inner_boundary)
    
    def generate_robust_boundary(self, cones, is_left=True):
        """
        Generate boundary using constrained spline with outlier rejection
        """
        if len(cones) < 2:
            return self.generate_straight_boundary(is_left)
        
        # Sort cones by distance along track direction
        sorted_cones = self.sort_cones_by_progress(cones)
        
        # Extract positions
        positions = np.array([[c['x'], c['y']] for c in sorted_cones])
        
        # Outlier rejection based on track width constraint
        filtered_positions = self.reject_outliers(positions, expected_width=3.0)
        
        if len(filtered_positions) < 2:
            return self.generate_straight_boundary(is_left)
        
        # Generate smooth spline
        try:
            return self.fit_spline_boundary(filtered_positions)
        except:
            # Fallback to linear interpolation
            return self.linear_interpolation(filtered_positions)
    
    def sort_cones_by_progress(self, cones):
        """Sort cones by progress along track (relative to vehicle heading)"""
        vx, vy = self.vehicle_position
        heading = self.vehicle_heading
        forward = np.array([np.cos(heading), np.sin(heading)])
        
        cone_progress = []
        for cone in cones:
            to_cone = np.array([cone['x'] - vx, cone['y'] - vy])
            progress = np.dot(to_cone, forward)  # Distance along track direction
            cone_progress.append((progress, cone))
        
        # Sort by progress (closest first, then along track)
        return [cone for _, cone in sorted(cone_progress)]
    
    def reject_outliers(self, positions, expected_width=3.0):
        """Remove positions that violate track width constraints"""
        if len(positions) < 3:
            return positions
        
        # Simple outlier rejection based on distance from line of best fit
        # More sophisticated methods could use RANSAC
        
        # For now, just remove points too far from neighbors
        filtered = [positions[0]]  # Always keep first point
        
        for i in range(1, len(positions)-1):
            curr = positions[i]
            prev = positions[i-1]
            next_pos = positions[i+1]
            
            # Check if current point is reasonable relative to neighbors
            dist_to_prev = np.linalg.norm(curr - prev)
            dist_to_next = np.linalg.norm(curr - next_pos)
            
            # Reject if too far from neighbors (cone spacing should be 1-2m)
            if dist_to_prev < 5.0 and dist_to_next < 5.0:
                filtered.append(curr)
        
        if len(positions) > 0:
            filtered.append(positions[-1])  # Always keep last point
        
        return np.array(filtered)
    
    def fit_smooth_boundary(self, positions):
        """Fit smooth spline optimized for Formula Student tracks"""
        if len(positions) < 3:
            return self.linear_interpolation(positions)
        
        # Parameterize by arc length for smooth curves
        distances = np.zeros(len(positions))
        for i in range(1, len(positions)):
            distances[i] = distances[i-1] + np.linalg.norm(positions[i] - positions[i-1])
        
        # Fit spline with appropriate smoothing for FS tracks
        tck, u = splprep([positions[:, 0], positions[:, 1]], u=distances, s=0.5)  # Low smoothing for accuracy
        
        # Generate boundary points optimized for ~1m spacing
        total_length = distances[-1]
        num_points = max(8, int(total_length / 1.0))  # ~1m point spacing
        u_new = np.linspace(0, total_length, num_points)
        boundary_points = splev(u_new, tck)
        
        return np.column_stack([boundary_points[0], boundary_points[1]])
    
    def linear_interpolation(self, positions):
        """Linear interpolation optimized for cone spacing"""
        if len(positions) < 2:
            return positions
        
        # Generate points with appropriate density for FS tracks
        result = [positions[0]]
        
        for i in range(1, len(positions)):
            start = positions[i-1]
            end = positions[i]
            distance = np.linalg.norm(end - start)
            
            # Interpolate based on expected 1-2m cone spacing
            if distance > 1.5:  # Only interpolate if cones are far apart
                num_points = max(2, int(distance / 1.0))  # ~1m spacing
                for j in range(1, num_points):
                    alpha = j / num_points
                    interp_point = start + alpha * (end - start)
                    result.append(interp_point)
            
            result.append(end)
        
        return np.array(result)
    
    def generate_centerline(self, left_boundary, right_boundary):
        """Generate centerline between boundaries"""
        if left_boundary is None or right_boundary is None:
            return None
        
        # Resample both boundaries to same number of points
        min_len = min(len(left_boundary), len(right_boundary))
        
        if min_len < 2:
            return None
        
        # Simple midpoint calculation
        centerline = []
        for i in range(min_len):
            midpoint = (left_boundary[i] + right_boundary[i]) / 2
            centerline.append(midpoint)
        
        return np.array(centerline)
    
    def validate_track(self, left_boundary, right_boundary):
        """Validate track geometry"""
        if left_boundary is None or right_boundary is None:
            return False
        
        if len(left_boundary) < 2 or len(right_boundary) < 2:
            return False
        
        # Check track width constraints
        for i in range(min(len(left_boundary), len(right_boundary))):
            width = np.linalg.norm(left_boundary[i] - right_boundary[i])
            if width < 1.5 or width > 6.0:  # Track width should be ~3m ± tolerance
                return False
        
        return True
    
    def temporal_smoothing(self, current_boundary, previous_boundary, alpha=0.2):
        """Temporal smoothing optimized for control stability"""
        if previous_boundary is None or current_boundary is None:
            return current_boundary
        
        # Only smooth if boundaries are similar length (avoid major changes)
        if abs(len(current_boundary) - len(previous_boundary)) > 3:
            return current_boundary
        
        # Conservative smoothing for stability
        min_len = min(len(current_boundary), len(previous_boundary))
        smoothed = current_boundary.copy()
        
        for i in range(min_len):
            smoothed[i] = alpha * current_boundary[i] + (1 - alpha) * previous_boundary[i]
        
        return smoothed
    
    def generate_straight_boundary(self, is_outer):
        """Generate straight boundary - simplified for FS"""
        vx, vy = self.vehicle_position
        heading = self.vehicle_heading
        
        # Use appropriate offset for outer vs inner boundary
        offset = 1.5 if is_outer else -1.5
        
        distances = np.linspace(0, 12, 6)  # 12m ahead, 6 points
        boundary = []
        
        for dist in distances:
            center_x = vx + dist * np.cos(heading)
            center_y = vy + dist * np.sin(heading)
            
            boundary_x = center_x + offset * (-np.sin(heading))
            boundary_y = center_y + offset * np.cos(heading)
            
            boundary.append([boundary_x, boundary_y])
        
        return np.array(boundary)
    
    def generate_centerline(self, outer_boundary, inner_boundary):
        """Generate centerline between boundaries - FS optimized"""
        if outer_boundary is None or inner_boundary is None:
            return None
        
        # Resample both boundaries to same number of points
        min_len = min(len(outer_boundary), len(inner_boundary))
        
        if min_len < 2:
            return None
        
        # Simple midpoint calculation (could be enhanced with racing line optimization)
        centerline = []
        for i in range(min_len):
            midpoint = (outer_boundary[i] + inner_boundary[i]) / 2
            centerline.append(midpoint)
        
        return np.array(centerline)
    
    def publish_boundary(self, publisher, boundary):
        """Publish boundary as ROS Path message"""
        if boundary is None or len(boundary) == 0:
            return
        
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in boundary:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        
        publisher.publish(path_msg)

def main(args=None):
    """Main function optimized for Formula Student deployment"""
    rclpy.init(args=args)
    
    node = TrackGeneratorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Track generator shutting down gracefully')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()