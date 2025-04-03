import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import sqlite3
from datetime import datetime
import json

class Navigator(Node):
    def __init__(self, namespace, goal_pose):
        super().__init__('navigator')
        self._action_client = ActionClient(self, NavigateToPose, f'/{namespace}/navigate_to_pose')
        self.goal_pose = goal_pose
        self._goal_done_future = rclpy.task.Future()
        self.namespace = namespace

    def send_goal(self):
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            self._goal_done_future.set_result(False)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.goal_pose

        self.get_logger().info(f'Sending goal to {self._action_client._action_name} with pose: {self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y}')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._goal_done_future.set_result(False)
            SystemExit(1)
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')
        self._goal_done_future.set_result(True)

def remove_bot_from_db(namespace, db_path='tasks.db'):
    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()
        cursor.execute("DELETE FROM AvailableBots WHERE namespace = ?", (namespace,))
        connection.commit()
        print(f"Removed bot with namespace '{namespace}' from the database.")

def add_bot_to_db(namespace, db_path='tasks.db'):
    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()
        cursor.execute("INSERT INTO AvailableBots (namespace) VALUES (?);", (namespace,))
        connection.commit()
        print(f"Added bot with namespace '{namespace}' back to the database.")

def move_to_completed(task_id, namespace, x, y, distance_json, timestamp_added, db_path='tasks.db'):
    timestamp_completed = datetime.now().isoformat()
    timestamp_started = timestamp_added  # Assuming the task started when it was added
    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()
        # Remove from Pending table
        cursor.execute("DELETE FROM Pending WHERE id = ?", (task_id,))
        # Add to Completed table
        cursor.execute("""
            INSERT INTO Completed (id, x_coordinate, y_coordinate, bot, timestamp_completed, timestamp_added, timestamp_started, bot_intial_pose)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        """, (task_id, x, y, namespace, timestamp_completed, timestamp_added, timestamp_started, distance_json))
        connection.commit()
        print(f"Moved task with ID '{task_id}' for bot '{namespace}' to Completed table.")

def main():
    parser = argparse.ArgumentParser(description='Send a robot to a specified goal using nav2.')
    parser.add_argument('--x', type=float, required=True, help='Target position x')
    parser.add_argument('--y', type=float, required=True, help='Target position y')
    parser.add_argument('--namespace', type=str, required=True, help='Robot namespace')
    parser.add_argument('--id', type=int, default=None, help='Task ID')
    parser.add_argument('--distance_json', type=str, default=None, help='Distance JSON')
    parser.add_argument('--timestamp_added', type=str, default=None, help='Timestamp when the task was added')
    args = parser.parse_args()

    # Remove the bot from the database
    remove_bot_from_db(args.namespace)

    rclpy.init()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = args.x
    goal_pose.pose.position.y = args.y
    goal_pose.pose.orientation.w = 1.0

    navigator = Navigator(args.namespace, goal_pose)
    navigator.send_goal()

    rclpy.spin_until_future_complete(navigator, navigator._goal_done_future)
    rclpy.shutdown()

    # Add the bot back to the database
    add_bot_to_db(args.namespace)

    # Move the task to the Completed table
    move_to_completed(args.id, args.namespace, args.x, args.y, args.distance_json, args.timestamp_added)

if __name__ == '__main__':
    main()