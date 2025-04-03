import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import threading
import math, time

# Lookahead distance for Pure Pursuit and constant speed
lookahead_distance = 0.22
speed = 0.18

# Convert quaternion to yaw (robot's orientation in 2D plane)
def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)  # Roll (not used here)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)  # Pitch (not used here)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)  # Yaw (used for robot heading)
    return yaw_z

# Pure Pursuit controller to compute velocity and steering angle
def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None  # Point beyond lookahead distance
    v = speed  # Default speed

    # Iterate over the path to find the next waypoint
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)  # Euclidean distance
        if lookahead_distance < distance:
            closest_point = (x, y)
            index = i  # Update index to the next waypoint
            break

    # Compute desired steering angle
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y, closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        # If no valid point, aim for the last waypoint
        target_heading = math.atan2(path[-1][1] - current_y, path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1

    # Normalize angle to [-π, π]
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi

    # Limit the steering angle for safety and reduce speed if turning sharply
    if desired_steering_angle > math.pi / 6 or desired_steering_angle < -math.pi / 6:
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi / 4
        v = 0.0  # Stop during sharp turns

    return v, desired_steering_angle, index

# ROS 2 Node for Path Following
class pathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        # Subscribe to topics
        self.subscription_path = self.create_subscription(Float32MultiArray, '/path', self.get_path, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers for path visualization and robot velocity
        self.publisher_visual_path = self.create_publisher(Path, '/visual_path', 10)
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        print("Path follower node has been started")

    # Callback to handle the path received
    def get_path(self, msg):
        print("Path has been received")
        # Convert the path from Float32MultiArray to list of (x, y) tuples
        data_list = [msg.data[i] for i in range(len(msg.data))]
        reshaped_data_list = [(data_list[i], data_list[i + 1]) for i in range(0, len(data_list), 2)]
        self.path = reshaped_data_list

        # Start following the path in a separate thread
        threading.Thread(target=self.follow_path).start()

    # Main loop for following the path
    def follow_path(self):
        twist = Twist()  # Twist message to publish velocity
        path_msg = Path()  # Path message for visualization
        path_msg.header.frame_id = "merge_map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Create visual path message
        for i in range(len(self.path)):
            pose = PoseStamped()
            pose.pose.position.x = self.path[i][0]
            pose.pose.position.y = self.path[i][1]
            path_msg.poses.append(pose)

        v = 0.0
        w = 0.0
        i = 0  # Index of the current waypoint
        while True:
            if not hasattr(self, 'x'):
                continue  # Wait for odometry data

            # Calculate velocity and steering using Pure Pursuit
            v, w, i = pure_pursuit(self.x, self.y, self.yaw, self.path, i)

            # Check if the robot has reached the final waypoint
            if abs(self.x - self.path[-1][0]) < 0.15 and abs(self.y - self.path[-1][1]) < 0.15:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher_cmd_vel.publish(twist)
                print("Path has been followed")
                break

            # Publish velocity and visual path
            twist.linear.x = v
            twist.angular.z = w
            self.publisher_visual_path.publish(path_msg)
            self.publisher_cmd_vel.publish(twist)

            # Sleep to control update frequency
            time.sleep(0.1)

    # Callback to process odometry updates
    def odom_callback(self, msg):
        # Extract position and orientation from Odometry message
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )

# Main function to start the ROS 2 node
def main(args=None):
    rclpy.init(args=args)
    path_follower = pathFollower()
    rclpy.spin(path_follower)  # Keep the node running
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
