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
        # … declare params, cmd_vel_sub, map_topic_sub …
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self.last_cmd = None       # (velocity, time)
        self.last_map_pose = None  # (x, y, time)
        self.timer = self.create_timer(0.1, self.check_stuck_tf)

    def cmd_vel_cb(self, msg: Twist):
        self.last_cmd = (msg.linear.x, time.time())

    def check_stuck_tf(self):
        # only check if we’re being asked to drive forward
        if not self.last_cmd or self.last_cmd[0] <= self.get_parameter('collision_threshold').value:
            self.last_map_pose = None
            return

        # lookup map→base_link
        try:
            tf = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())
        except Exception:
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        now = time.time()

        if self.last_map_pose is None:
            self.last_map_pose = (x, y, now)
            return

        last_x, last_y, last_t = self.last_map_pose
        dist = ((x - last_x)**2 + (y - last_y)**2)**0.5
        dt   = now - last_t

        stuck_time = self.get_parameter('stuck_time').value
        # if commanded forward but moved <10 mm in stuck_time → collision
        if dist < 10e-3 and dt > stuck_time:
            self.get_logger().warn("Stuck detected ➜ injecting obstacle")
            self.inject_obstacle()
            self.last_map_pose = (x, y, now)
        else:
            self.last_map_pose = (x, y, now)


def main(args=None):
    rclpy.init(args=args)
    node = MapCollisionInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
