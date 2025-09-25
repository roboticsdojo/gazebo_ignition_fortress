#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros
from py_trees.common import ParallelPolicy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
import sys

def print_same_line(msg1, msg2):
    sys.stdout.write("\033[F\033[F")  # カーソルを2行上に移動
    sys.stdout.write(f"{msg1}\n")
    sys.stdout.write(f"{msg2}\n")
    sys.stdout.flush()

# ---- Task 1 & Task 2 ----
class PrintHello(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHello"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hello")
            self.executed = True
        return py_trees.common.Status.SUCCESS


class PrintHi(py_trees.behaviour.Behaviour):
    def __init__(self, name="PrintHi"):
        super().__init__(name)
        self.executed = False

    def update(self):
        if not self.executed:
            print("hi")
            self.executed = True
        return py_trees.common.Status.SUCCESS


class MoveToPosition(py_trees.behaviour.Behaviour):
    # Class variables to share origin and odom across all instances
    origin_x = None
    origin_y = None
    origin_yaw = None  # Store initial orientation
    origin_samples = []  # Store multiple samples to ensure robot is stationary
    origin_sample_count = 10  # Increased samples for better stability
    origin_locked = False  # Prevent origin from being reset once established

    # Shared odom data
    global_x = 0.0
    global_y = 0.0
    global_yaw = 0.0
    odom_initialized = False
    odom_sub = None  # Single subscription shared across instances

    def __init__(self, name, target_x, target_y, tolerance=1.0):
        super().__init__(name)
        self.target_x = target_x
        self.target_y = target_y
        self.cmd_vel_pub = None
        self.completed = False
        self.tolerance = tolerance  # Configurable tolerance

    def setup(self, **kwargs):
        node = kwargs.get('node')
        if node:
            self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
            # Create single shared odom subscription
            if MoveToPosition.odom_sub is None:
                MoveToPosition.odom_sub = node.create_subscription(
                    Odometry, '/diff_drive_base_controller/odom', MoveToPosition.shared_odom_callback, 10)
                print(f"Shared odom subscription created")
            print(f"{self.name}: Setup completed")

    @classmethod
    def shared_odom_callback(cls, msg):
        import math
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Set initial position and orientation as origin only when robot is stationary
        # Once origin is locked, never change it again
        if not cls.origin_locked and cls.origin_x is None:
            # Collect samples to ensure robot is stationary
            cls.origin_samples.append((raw_x, raw_y, raw_yaw))

            if len(cls.origin_samples) >= cls.origin_sample_count:
                # Check if all samples are close to each other (robot is stationary)
                samples = cls.origin_samples
                x_values = [s[0] for s in samples]
                y_values = [s[1] for s in samples]
                yaw_values = [s[2] for s in samples]

                x_range = max(x_values) - min(x_values)
                y_range = max(y_values) - min(y_values)
                yaw_range = max(yaw_values) - min(yaw_values)

                if x_range < 0.005 and y_range < 0.005 and yaw_range < 0.02:  # Very strict tolerance
                    cls.origin_x = sum(x_values) / len(x_values)
                    cls.origin_y = sum(y_values) / len(y_values)
                    cls.origin_yaw = sum(yaw_values) / len(yaw_values)
                    cls.origin_locked = True  # Lock the origin permanently
                    print(f"ORIGIN LOCKED at stationary pose: ({cls.origin_x:.3f}, {cls.origin_y:.3f}, {cls.origin_yaw:.3f})")
                else:
                    # Robot is moving, clear samples and wait for it to stop
                    print("Robot is moving, waiting for it to stop before setting origin...")
                    cls.origin_samples = []

        # Only process coordinates if origin is locked (permanently established)
        if cls.origin_locked and cls.origin_x is not None:
            # Calculate relative position with rotation compensation
            # Transform to origin-centered coordinates
            dx = raw_x - cls.origin_x
            dy = raw_y - cls.origin_y

            # Rotate coordinates to align with initial orientation
            cos_yaw = math.cos(-cls.origin_yaw)
            sin_yaw = math.sin(-cls.origin_yaw)

            cls.global_x = dx * cos_yaw - dy * sin_yaw
            cls.global_y = dx * sin_yaw + dy * cos_yaw
            cls.global_yaw = raw_yaw - cls.origin_yaw

            cls.odom_initialized = True  # Mark that we received odom data
            # Only print from first instance to reduce spam
        else:
            pass  # Reduced logging

    def update(self):
        import math

        # If already completed, don't run again
        if self.completed:
            return py_trees.common.Status.SUCCESS

        # Wait for origin to be locked and odom data to be ready
        if not MoveToPosition.origin_locked or not MoveToPosition.odom_initialized:
            print(f"{self.name}: Waiting for origin to be LOCKED and odom data...")
            return py_trees.common.Status.RUNNING

        # Get current position from shared data
        current_x = MoveToPosition.global_x
        current_y = MoveToPosition.global_y

        # Calculate distance to target
        dx = self.target_x - current_x
        dy = self.target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)

        # print(f"{self.name}: Current({current_x:.2f}, {current_y:.2f}) -> Target({self.target_x}, {self.target_y}), Distance: {distance:.2f}m")
        distance_msg = f"{self.name}: Current({current_x:.2f}, {current_y:.2f}) -> Target({self.target_x}, {self.target_y}), Distance: {distance:.2f}m"

        # Check if reached target
        if distance < self.tolerance:
            # Stop and complete
            twist = Twist()  # All velocities = 0
            if self.cmd_vel_pub:
                self.cmd_vel_pub.publish(twist)
                # Publish stop command multiple times to ensure it's received
                import time
                time.sleep(0.1)
                self.cmd_vel_pub.publish(twist)
            print(f"{self.name}: Reached target position! Final distance: {distance:.2f}m")
            self.completed = True
            return py_trees.common.Status.SUCCESS

        # Get current robot orientation
        current_yaw = MoveToPosition.global_yaw

        # Calculate desired angle to target
        angle_to_target = math.atan2(dy, dx)

        # Calculate angular error (difference between desired and current orientation)
        angular_error = angle_to_target - current_yaw

        # Normalize angular error to [-pi, pi] (shortest rotation)
        while angular_error > math.pi:
            angular_error -= 2 * math.pi
        while angular_error < -math.pi:
            angular_error += 2 * math.pi

        # Improved proportional controller
        twist = Twist()

        # Only move forward if roughly facing the right direction
        if abs(angular_error) < math.pi/4:  # Wiithin 45 degrees
            # Linear velocity - reduce when close to target
            if distance > 1.0:
                twist.linear.x = 0.3  # Full speed when far
            else:
                twist.linear.x = max(0.1, distance * 0.3)  # Slow down when close
        else:
            # Turn in place when not facing target
            twist.linear.x = 0.0

        # Angular velocity - proportional to angular error with limits
        twist.angular.z = max(-0.8, min(0.8, angular_error * 2.0))  # Stronger angular control

        # print(f"\r{self.name}: angle_to_target={angle_to_target:.2f}, current_yaw={current_yaw:.2f}, angular_error={angular_error:.2f}", end="")
        angular_msg = f"\r{self.name}: angle_to_target={angle_to_target:.2f}, current_yaw={current_yaw:.2f}, angular_error={angular_error:.2f}"
        print(distance_msg+"\n"+angular_msg+"\n"+"\033[2A",end="")

        if self.cmd_vel_pub:
            self.cmd_vel_pub.publish(twist)

        return py_trees.common.Status.RUNNING


# ---- Helper for goals ----
def make_pose(x, y, yaw=0.0, frame="map"):
    import math
    pose = PoseStamped()
    pose.header.frame_id = frame
    # Leave timestamp as zero to use current time
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Convert yaw to quaternion
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)

    print(f"Goal created: x={x}, y={y}, yaw={yaw}, frame={frame}")
    return pose


# ---- Tree Definition ----
def create_root():
    root = py_trees.composites.Sequence("RootSequence", memory=True)

    # Move to specific coordinates
    move1 = MoveToPosition("MoveToPoint1", 1.0, 1.0, tolerance=0.45)  # Move to (1.0, 1.0) with 1.5m tolerance
    task1 = PrintHello("Task1")
    move2 = MoveToPosition("MoveToPoint2", -1.0, -1.0, tolerance=0.45)  # Move to (-1.0, -1.0) with 1.5m tolerance
    task2 = PrintHi("Task2")

    # Add to sequence
    root.add_children([move1, task1, move2, task2])
    return root


# ---- Main ----
def main():
    rclpy.init()

    root = create_root()

    # ROS2 node wrapper
    node = py_trees_ros.trees.BehaviourTree(root)

    try:
        node.setup(timeout=10.0, node=node.node)  # Pass the ROS node to behaviors

        # Execute the tree with regular timer
        print("Behavior tree starting...")

        tree_completed = False  # Track completion state

        def tick_tree():
            nonlocal tree_completed

            if tree_completed:
                return  # Don't execute if already completed

            result = node.tick_tock(period_ms=100)
            #print(f"Tree status: {node.root.status}")

            # Print status of each child
            for i, child in enumerate(node.root.children):
                print(f"  Child {i} ({child.name}): {child.status}")

            if node.root.status == py_trees.common.Status.SUCCESS:
                print("Behavior tree completed successfully!")
                tree_completed = True  # Mark as completed
                return
            elif node.root.status == py_trees.common.Status.FAILURE:
                print("Behavior tree failed!")
                tree_completed = True  # Mark as completed
                return

        # Create a regular timer to execute the tree
        timer = node.node.create_timer(0.5, tick_tree)  # Execute every 0.5 seconds

        print("Behavior tree is running...")
        rclpy.spin(node.node)
    except RuntimeError as e:
        print(f"Setup failed: {e}")
        print("Make sure Nav2 is running: ros2 launch nav2_bringup navigation_launch.py")
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
