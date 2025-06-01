import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_ros import Buffer, TransformListener
import numpy as np
import time

class MapCollisionInjector(Node):
    def __init__(self):
        super().__init__('map_collision_injector')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('collision_threshold', 0.05)  # m/s
        self.declare_parameter('stuck_time', 1.0)           # seconds
        
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_cb, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.get_parameter('map_topic').value,
            self.map_cb, 1)
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/modified_map', 1)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self.last_cmd = None
        self.last_odom = None
        self.stuck_since = None
        self.current_map = None

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd = (msg.linear.x, time.time())

    def odom_cb(self, msg: Odometry):
        self.last_odom = (msg.twist.twist.linear.x, time.time())
        self.check_stuck()

    def map_cb(self, msg: OccupancyGrid):
        # keep a local copy of the latest map
        self.current_map = msg

    def check_stuck(self):
        if not self.last_cmd or not self.last_odom or not self.current_map:
            return

        cmd_v, cmd_t = self.last_cmd
        odom_v, odom_t = self.last_odom
        threshold = self.get_parameter('collision_threshold').value
        stuck_time = self.get_parameter('stuck_time').value

        # If we commanded forward but odom says we're barely moving
        if cmd_v > threshold and odom_v < threshold:
            if self.stuck_since is None:
                self.stuck_since = odom_t
            elif odom_t - self.stuck_since > stuck_time:
                self.inject_obstacle()
                self.stuck_since = None
        else:
            self.stuck_since = None

    def inject_obstacle(self):
        try:
            # Look up the robot's base footprint in map frame
            t = self.tf_buffer.lookup_transform(
                self.current_map.header.frame_id,
                'base_link',
                rclpy.time.Time())
            mx = int((t.transform.translation.x - self.current_map.info.origin.position.x)
                     / self.current_map.info.resolution)
            my = int((t.transform.translation.y - self.current_map.info.origin.position.y)
                     / self.current_map.info.resolution)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # Stamp a small 3×3 black square around the robot cell
        data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height,
             self.current_map.info.width))
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                x, y = mx + dx, my + dy
                if 0 <= x < data.shape[1] and 0 <= y < data.shape[0]:
                    data[y, x] = 100  # Occupied

        # Publish modified map
        new_map = OccupancyGrid()
        new_map.header = self.current_map.header
        new_map.info = self.current_map.info
        new_map.data = list(data.flatten())
        self.map_pub.publish(new_map)
        self.get_logger().info(f"Injected obstacle at map cell ({mx},{my})")

def main(args=None):
    rclpy.init(args=args)
    node = MapCollisionInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
