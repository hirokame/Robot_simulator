# ============================================================
#  Gazebo + ROS 2 Quick Demo
#
#  Unlike PyBullet / MuJoCo (pip install), Gazebo requires
#  a full system install. Steps:
#
#  1. Install ROS 2 Humble (Ubuntu 22.04):
#       https://docs.ros.org/en/humble/Installation.html
#
#  2. Install Gazebo + ROS 2 bridge:
#       sudo apt install ros-humble-ros-gz
#       sudo apt install ros-humble-turtlebot3-gazebo
#
#  3. Source your workspace, then run this script:
#       source /opt/ros/humble/setup.bash
#       python3 demo_gazebo_ros2.py
#
# ============================================================
#
#  What makes Gazebo different from PyBullet / MuJoCo?
#
#  PyBullet / MuJoCo  → Python-first, great for RL training,
#                        you control the sim loop directly.
#
#  Gazebo + ROS 2     → Robot-first, great for full system
#                        simulation (sensors, comms, navigation).
#                        The simulator runs as a separate process;
#                        your code talks to it via ROS 2 topics.
#
# ============================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist       # velocity command message
from nav_msgs.msg    import Odometry      # position/velocity feedback
from sensor_msgs.msg import LaserScan     # LIDAR data
import math
import time

# ============================================================
#  Part 1: Robot Controller Node
#  This node sends velocity commands to a TurtleBot3 robot
#  in Gazebo and reads its odometry + laser scan.
# ============================================================

class RobotController(Node):
    """
    A ROS 2 node that controls a simulated TurtleBot3.
    Publishes to:   /cmd_vel    (move the robot)
    Subscribes to:  /odom       (robot's position in the world)
                    /scan       (LIDAR distance readings)
    """

    def __init__(self):
        super().__init__("koji_robot_controller")

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(
            Twist,          # message type
            "/cmd_vel",     # topic name
            10              # queue size
        )

        # --- Subscribers ---
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )

        # --- State variables ---
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0          # heading in radians
        self.min_obstacle_dist = float("inf")

        self.get_logger().info("RobotController node started!")

    # ----------------------------------------------------------
    # CALLBACK: called every time a new /odom message arrives
    # ----------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        self.x = pos.x
        self.y = pos.y

        # Convert quaternion → yaw angle
        # (simplified formula for 2D planar robot)
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y ** 2 + ori.z ** 2)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ----------------------------------------------------------
    # CALLBACK: called every time a new /scan (LIDAR) arrives
    # ----------------------------------------------------------
    def scan_callback(self, msg: LaserScan):
        # LIDAR returns a list of distances in a 360° sweep
        # Filter out invalid (inf / nan) readings
        valid_ranges = [r for r in msg.ranges
                        if not math.isinf(r) and not math.isnan(r)]
        if valid_ranges:
            self.min_obstacle_dist = min(valid_ranges)

    # ----------------------------------------------------------
    # PUBLISH: send a velocity command to the robot
    # ----------------------------------------------------------
    def move(self, linear_x: float, angular_z: float):
        """
        linear_x  : forward speed  (m/s), negative = backward
        angular_z : rotation speed (rad/s), positive = counter-clockwise
        """
        msg = Twist()
        msg.linear.x  = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        self.move(0.0, 0.0)

    def print_state(self, label=""):
        self.get_logger().info(
            f"{label:20s}  pos=({self.x:.3f}, {self.y:.3f})  "
            f"yaw={math.degrees(self.yaw):.1f}°  "
            f"nearest_obstacle={self.min_obstacle_dist:.3f}m"
        )


# ============================================================
#  Part 2: Simple Behaviour — Drive a Square
#
#  The robot drives forward, turns 90°, and repeats 4 times.
#  This is a classic "Hello World" for robot navigation.
# ============================================================

def drive_square(node: RobotController, side_length=1.0):
    """Drive the robot in a 1-metre square."""

    FORWARD_SPEED  = 0.2   # m/s
    TURN_SPEED     = 0.5   # rad/s

    time_per_side  = side_length / FORWARD_SPEED   # seconds
    time_per_turn  = (math.pi / 2) / TURN_SPEED    # 90° turn

    node.get_logger().info("Starting square drive sequence...")

    for side in range(4):
        node.get_logger().info(f"  Side {side+1}: moving forward {side_length}m")

        # --- Move forward ---
        end_time = time.time() + time_per_side
        while time.time() < end_time:
            # Obstacle avoidance: stop if something is < 0.3 m away
            if node.min_obstacle_dist < 0.3:
                node.get_logger().warn("Obstacle detected! Stopping.")
                node.stop()
                return

            node.move(FORWARD_SPEED, 0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        # --- Turn left 90° ---
        node.get_logger().info(f"  Side {side+1}: turning 90°")
        end_time = time.time() + time_per_turn
        while time.time() < end_time:
            node.move(0.0, TURN_SPEED)
            rclpy.spin_once(node, timeout_sec=0.05)

    node.stop()
    node.print_state("After square:")
    node.get_logger().info("Square drive complete!")


# ============================================================
#  Part 3: Main entry point
# ============================================================

def main():
    rclpy.init()
    node = RobotController()

    # Let subscribers warm up (process a few messages)
    print("\n[ROS2]  Warming up — spinning for 1 second...")
    end = time.time() + 1.0
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.print_state("Initial state:")

    # Run the square drive behaviour
    drive_square(node, side_length=1.0)

    # Cleanup
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    print("[ROS2]  Shutdown complete.")


if __name__ == "__main__":
    main()


# ============================================================
#  BONUS: How to launch TurtleBot3 in Gazebo
#
#  Open TWO terminal windows, both sourced with ROS 2:
#
#  Terminal 1 — Start Gazebo with TurtleBot3:
#    export TURTLEBOT3_MODEL=burger
#    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
#
#  Terminal 2 — Run this script:
#    python3 demo_gazebo_ros2.py
#
#  You can also visualise everything in RViz:
#    ros2 launch turtlebot3_bringup rviz2.launch.py
#
#  Topics to explore with ros2 topic list:
#    /cmd_vel         — send velocity commands
#    /odom            — robot odometry (pose + velocity)
#    /scan            — LIDAR laser scan
#    /camera/image_raw — camera feed (if equipped)
#    /clock           — simulation clock
#
# ============================================================
