import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial import KDTree
import time

# -----------------------------
# ConeTrack Class
# -----------------------------
class ConeTrack:
    def __init__(self, position, colour, timestamp):
        self.position = np.array(position)  # [x, y, z]
        self.colour = colour                # 0 = blue, 1 = yellow, 2 = orange
        self.score = 1                      # Confidence score
        self.last_seen = timestamp          # Last detection time

    def update(self, new_position, timestamp):
        # Weighted average update
        self.position = (self.position * self.score + new_position) / (self.score + 1)
        self.score += 1
        self.last_seen = timestamp

    def decay(self, current_time, max_age):
        if current_time - self.last_seen > max_age:
            self.score -= 1
        return self.score > 0

# -----------------------------
# ConeMapper Node
# -----------------------------
class ConeMapper(Node):
    def __init__(self):
        super().__init__('cone_mapper')

        self.cone_sub = self.create_subscription(String, '/detected_cones', self.cone_callback, 10)
        self.pose_sub = self.create_subscription(Odometry, '/odometry/slam', self.pose_callback, 10)
        self.publisher_ = self.create_publisher(String, '/world_cones', 10)

        self.latest_pose = None
        self.previous_pose_timestamp = None
        self.global_map = []  # List of ConeTrack objects

        self.match_radius = 0.3     # m
        self.publish_score_thresh = 3
        self.decay_time = 1.5       # seconds

    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.latest_pose = {
            'position': np.array([pos.x, pos.y, pos.z]),
            'orientation': np.array([ori.x, ori.y, ori.z, ori.w]),
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

    def cone_callback(self, msg):
        if self.latest_pose is None:
            self.get_logger().warn('No SLAM pose yet. Skipping frame.')
            return

        current_ts = self.latest_pose['timestamp']
        if self.previous_pose_timestamp == current_ts:
            self.get_logger().warn('Pose hasn’t changed. Skipping frame.')
            return
        self.previous_pose_timestamp = current_ts

        rot = R.from_quat(self.latest_pose['orientation'])
        R_wc = rot.as_matrix()
        t_wc = self.latest_pose['position'].reshape(3, 1)

        lines = msg.data.strip().split('\n')
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) != 4:
                continue
            x, y, z, colour = map(float, parts)
            X_cam = np.array([[x], [y], [z]])
            X_world = R_wc @ X_cam + t_wc
            X_flat = X_world[:3, 0]

            matched = False
            tree_points = [cone.position[:2] for cone in self.global_map if cone.colour == int(colour)]
            if tree_points:
                tree = KDTree(tree_points)
                dist, idx = tree.query(X_flat[:2])
                if dist < self.match_radius:
                    matching_cone = [c for c in self.global_map if c.colour == int(colour)][idx]
                    matching_cone.update(X_flat, current_ts)
                    matched = True

            if not matched:
                self.global_map.append(ConeTrack(X_flat, int(colour), current_ts))

        # Decay and filter
        now = time.time()
        self.global_map = [cone for cone in self.global_map if cone.decay(now, self.decay_time)]

        # Publish
        output_lines = []
        for cone in self.global_map:
            if cone.score >= self.publish_score_thresh:
                x, y, z = cone.position
                output_lines.append(f"{x:.2f},{y:.2f},{z:.2f},{cone.colour}")

        if output_lines:
            self.publisher_.publish(String(data='\n'.join(output_lines)))
            self.get_logger().info(f"Published {len(output_lines)} cones.")

# -----------------------------
# MAIN
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ConeMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
