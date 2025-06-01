#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Pose, Point, Quaternion, PoseWithCovarianceStamped
from std_msgs.msg import Bool
import yaml
import time
from transforms3d.euler import euler2quat
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
import random
import subprocess
import ast

class Explore(Node):
    def __init__(self):
        super().__init__('explore_node')
        
        # Parameters
        self.declare_parameter('planner_frequency', 1.0)
        self.declare_parameter('return_to_init', False)
        self.planner_frequency = self.get_parameter('planner_frequency').value
        self.return_to_init = self.get_parameter('return_to_init').value
        self.goal_in_progress = False
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoint_time_limit = 20 #seconds
        self.timeout_timer = None
        self.zones = [] 
        self.current_zone = 0
        self.current_contributors = []
        self.value_list = [] 
        # Waypoints
        self.waypoints = []
        self.current_waypoint_index = 0
        self.load_waypoints()
        self.random_value = 0
        self.emf_values = {}   
        # Timer for navigation
        self.timer = self.create_timer(1.0/self.planner_frequency, self.navigate_to_next_waypoint)
        
        # Subscription for resume/stop commands
        self.resume_sub = self.create_subscription(
            Bool,
            'explore/resume',
            self.resume_callback,
            10)
        
        self.get_logger().info("Exploration node initialized")

    def save_emf_values_yaml(self):
        waypoints = []

        for zone, (power, contributors) in self.emf_values.items():
            waypoints.append({
                'zone': zone,
                'power': power,
                'contributors': contributors
            })

        #with open('/home/ws/main_app/map_processor/emf_heatmap/emf_power.yaml', 'w') as f:
        #    yaml.dump({'emf_power': waypoints}, f)
        with open('/home/ws/main_app/map_tools/yaml_files/measurements.yaml', 'w') as f:
            yaml.dump({'measurements': waypoints}, f)

        #print(f"Saved {len(waypoints)} waypoints to /home/ws/main_app/map_processor/emf_heatmap/emf_power.yaml")
        print(f"Saved {len(waypoints)} waypoints to /home/ws/main_app/map_tools/yaml_files/measurements.yaml")

    def load_waypoints(self):
        #waypoints_file = "/home/ws/main_app/map_processor/zones_waypoints.yaml"
        waypoints_file = "/home/ws/main_app/map_tools/yaml_files/zones_waypoints.yaml"
        try:
            with open(waypoints_file, 'r') as file:
                waypoints_data = yaml.safe_load(file)
            
            temp_zone = 0
            for wp in waypoints_data['waypoints']:
                pose = Pose()
                pose.position.x = wp['x']
                pose.position.y = wp['y']
                temp_zone = wp['zone']
                # Convert theta to quaternion
                q = euler2quat(0, 0, wp['theta'])
                pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
                
                self.waypoints.append(pose)
                self.zones.append(temp_zone)
                self.get_logger().info(f"Loaded waypoint: x={wp['x']}, y={wp['y']}, theta={wp['theta']}")
                
            self.get_logger().info(f"Successfully loaded {len(self.waypoints)} waypoints")
        except Exception as e:
            self.get_logger().error(f"Failed to load waypoints: {str(e)}")
            raise

    def navigate_to_next_waypoint(self):
        if self.goal_in_progress:
            self.send_goal_future.add_done_callback(self.goal_response_callback)
            return  # Don't send a new goal if one is already in progress

        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("All waypoints visited")
            self.stop(True)
            return
        
        target_pose = self.waypoints[self.current_waypoint_index]
        self.current_zone = self.zones[self.current_waypoint_index]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint_index}: "
                            f"x={target_pose.position.x}, y={target_pose.position.y}")

        goal_msg = NavigateToPose.Goal()
        self.goal_in_progress = True
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose = target_pose

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self.goal_in_progress = True

    def handle_waypoint_timeout(self):
        self.get_logger().warn(f"Timeout: Waypoint {self.current_waypoint_index} took longer than {self.waypoint_time_limit} seconds. Cancelling...")

        if self.timeout_timer is not None:
            self.timeout_timer.cancel()
            self.timeout_timer = None

        if hasattr(self, 'current_goal_handle') and self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)

        self.goal_in_progress = False
        self.current_waypoint_index += 1

    def run_measure_and_parse(self):
        result = subprocess.run(
            "python3 /home/ws/main_app/spectrum_analyser/measure.py 1 mean",
            text=True,
            capture_output=True,
            shell=True,
        )
        print("Done scanning")
        output = result.stdout.strip()
        print(output)
        # Parse output manually
        lines = output.splitlines()
        try:
            self.random_value = float(lines[0])*100
        except Exception:
            self.random_value = None
            print("Failed to parse main value")

        self.current_contributors = []
        for line in lines[1:]:
            try:
                freq, perc = line.split(":")
                freq = freq.strip()
                perc = float(perc.strip().rstrip("%"))
                self.current_contributors.append([freq, perc])  # FIXED: append instead of overwrite
            except Exception as e:
                print(f"Failed to parse contributor line '{line}': {e}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        #print(future.result())
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_in_progress = False
            self.current_waypoint_index += 1
            return

        if self.goal_in_progress == False:
            self.get_logger().info('Goal accepted')
        

    
        #if self.timeout_timer is not None:
        #    self.timeout_timer.cancel()
        #    self.timeout_timer = None

        status = future.result().status
        #print(status)
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}. Waiting 3 seconds...")
            time.sleep(2)
            # ----TINYSA THREAD-------
            #self.random_value = round(random.uniform(1, 10), 1)
            try:
                self.run_measure_and_parse()
                time.sleep(2)
                self.value_list =[self.random_value, self.current_contributors]
            except Exception as e:
                self.get_logger().error(f"Failed to get value from TinySA: {e}")
                self.current_contributors = []
                self.random_value = 0.0  # fallback
            self.current_waypoint_index += 1
            self.goal_in_progress = False
            self.emf_values.update({self.current_zone: self.value_list})
            #Cancel the timer
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()
                self.timeout_timer = None
        
        elif status == GoalStatus.STATUS_EXECUTING or status == GoalStatus.STATUS_UNKNOWN or status == GoalStatus.STATUS_ACCEPTED:
            self.goal_in_progress = True
            #Cancel the timer
            if self.timeout_timer is not None:
                self.timeout_timer.cancel()
                self.timeout_timer = None
        else:
            self.get_logger().warn(f"Waypoint {self.current_waypoint_index} failed. Skipping...")
            self.current_waypoint_index += 1
            self.goal_in_progress = False

        # Start timeout timer
        if self.timeout_timer == None:
            self.timeout_timer = self.create_timer(
                self.waypoint_time_limit,
                self.handle_waypoint_timeout)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # You can add feedback handling here if needed

    def resume_callback(self, msg):
        if msg.data:
            self.get_logger().info("Resuming exploration")
            self.timer.reset()
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().info("Stopping exploration")
            self.stop()

    def run_map_post_process_emf(self):
        try:
            #subprocess.run(['python3', '/home/ws/main_app/map_processor/map_post_process.py', '--output=emf'], check=True)
            subprocess.run(['python3', '/home/ws/main_app/map_tools/main.py', '--output=heatmap'], check=True)
            print("EMF map post process main.py has been executed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Error running map_post_process.py: {e}")

    def stop(self, finished_exploring=False):
        self.timer.cancel()
        
        # Cancel all goals ---------------TO FIX----------------------
        #if hasattr(self, 'send_goal_future'):
        #    future = self.nav_client._cancel_goal_async()
        #    future.add_done_callback(self.cancel_done)
        
        # Send map via Python script
        try:
            self.save_emf_values_yaml()
            time.sleep(1)
            self.get_logger().info("Constructing EMF power heatmap...")
            self.run_map_post_process_emf()
            time.sleep(15)
            self.get_logger().info("Calling map_sender.py to e-mail the map...")
            result = subprocess.run(
                ['/usr/bin/python3', '/home/ws/main_app/report_generator/map_sender.py'],
                capture_output=True,
                text=True
            )
            self.get_logger().info(f"map_sender.py output:\n{result.stdout}")
        except Exception as e:
            self.get_logger().error(f"Failed to send map via email: {str(e)}")
        
        # Return to initial pose if configured
        if self.return_to_init and finished_exploring:
            self.return_to_initial_pose()

    def cancel_done(self, future):
        self.get_logger().info("All navigation goals canceled")

    def return_to_initial_pose(self):
        # Implement if you have an initial pose to return to
        self.get_logger().info("Returning to initial pose")
        # Similar to navigate_to_next_waypoint but with initial pose

def main(args=None):
    rclpy.init(args=args)
    explore_node = None  # <-- PREVENTS UnboundLocalError
    
    try:
        #tinysa_test_run = subprocess.run(
        #    ["python3", "/home/ws/main_app/spectrum_analyser/measure.py", "1", "mean"],
        #    text=True,
        #    capture_output=True,
        #    check=True
        #)
                # ⬇️ Load map.yaml to extract origin
        map_yaml_path = '/home/ws/ugv_ws/install/ugv_nav/share/ugv_nav/maps/map.yaml'
        with open(map_yaml_path, 'r') as file:
            map_data = yaml.safe_load(file)
            origin = map_data.get('origin', [0.0, 0.0, 0.0])  # [x, y, yaw]

        # ⬇️ Publish initial pose based on origin
        init_pose_node = rclpy.create_node('init_pose_pub')
        init_pose_pub = init_pose_node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = init_pose_node.get_clock().now().to_msg()
        msg.pose.pose.position.x = origin[0]
        msg.pose.pose.position.y = origin[1]

        # Convert yaw (origin[2]) to quaternion
        q = euler2quat(0, 0, origin[2])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        for _ in range(10):
            #init_pose_pub.publish(msg)
            time.sleep(0.1)

        init_pose_node.destroy_node()

        explore_node = Explore()
        executor = MultiThreadedExecutor()
        rclpy.spin(explore_node, executor)
    except KeyboardInterrupt:
        explore_node.get_logger().info("Shutting down exploration node")
    finally:
        explore_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()