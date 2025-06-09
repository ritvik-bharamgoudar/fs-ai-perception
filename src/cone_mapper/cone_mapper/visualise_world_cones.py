import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import KDTree

class ConeVisualizer(Node):
    def __init__(self):
        super().__init__('cone_visualizer')
        self.subscription = self.create_subscription(
            String,
            '/world_cones',
            self.listener_callback,
            10)
        self.subscription

        # Set up live plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.global_cones = []  # Persistent map to track all cones
        self.last_frame_count = 0

    def listener_callback(self, msg):
        lines = msg.data.strip().split('\n')
        points = []
        colors = []

        for line in lines:
            parts = line.strip().split(',')
            if len(parts) != 4:
                continue
            x, y, z, label = map(float, parts)
            points.append((x, y))
            if int(label) == 0:  # Blue
                colors.append('blue')
            elif int(label) == 1:  # Yellow
                colors.append('gold')
            elif int(label) == 2:  # Orange
                colors.append('orange')
            else:
                colors.append('black')

        self.ax.clear()
        self.ax.set_title("Global Cone Map (Filtered via KD-Tree)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle='--', linewidth=0.5)

        # Set axis limits dynamically or manually for clarity
        self.ax.set_xlim(-10, 30)   # Adjust based on your track
        self.ax.set_ylim(-5, 10)


        if points:
            points = np.array(points)
            tree=KDTree(points)
            pairs = tree.query_pairs(r=0.2)
            for i,j in pairs:
                x1, y1 = points[i]
                x2, y2 = points[j]
                self.ax.plot([x1, x2], [y1, y2], 'r--', linewidth=1)
            self.ax.scatter(points[:, 0], points[:, 1], c=colors, s=40)
            cone_count = len(points)
            if cone_count != self.last_frame_count:
                self.get_logger().info(f"Total cones in global map: {cone_count}")
                self.last_frame_count = cone_count
        else:
            self.ax.text(0.5, 0.5, "No cones", ha='center', va='center')

        plt.draw()
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = ConeVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.ioff()
    plt.show()


if __name__ == '__main__':
    main()

