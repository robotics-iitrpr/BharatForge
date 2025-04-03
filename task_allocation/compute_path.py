import math
import argparse
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.task import Future
import json

class MultiRobotPathDistance(Node):
    def __init__(self, robot_names, goal_pose):
        super().__init__('multi_robot_path_distance')
        self.robot_action_clients = {
            robot_name: ActionClient(self, ComputePathToPose, f'/{robot_name}/compute_path_to_pose')
            for robot_name in robot_names
        }
        self.goal_pose = goal_pose
        self.results = {}
        self.remaining_robots = len(robot_names)
        self.computation_done_future = Future()
        
        for robot_name in robot_names:
            self.compute_path(robot_name)
    
    def compute_path(self, robot_name):
        if not self.robot_action_clients[robot_name].wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"Action server for {robot_name} not available!")
            self.remaining_robots -= 1
            if self.remaining_robots == 0:
                self.computation_done_future.set_result(True)
            return

        goal = ComputePathToPose.Goal()
        goal.goal = self.goal_pose

        future = self.robot_action_clients[robot_name].send_goal_async(goal)
        future.add_done_callback(lambda future, robot_name=robot_name: self.handle_path_response(future, robot_name))

    def handle_path_response(self, future, robot_name):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.remaining_robots -= 1
                if self.remaining_robots == 0:
                    self.computation_done_future.set_result(True)
                return

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda result_future, robot_name=robot_name: self.process_result(result_future, robot_name))
        except Exception as e:
            self.remaining_robots -= 1
            if self.remaining_robots == 0:
                self.computation_done_future.set_result(True)

    def process_result(self, result_future, robot_name):
        try:
            result = result_future.result().result
            if result.path.poses:
                path_distance = self.calculate_path_distance(result.path)
                self.results[robot_name] = path_distance
            else:
                self.get_logger().error(f"No path found for {robot_name}.")
        except Exception as e:
            self.get_logger().error(f"Exception in processing result for {robot_name}: {str(e)}")
        finally:
            self.remaining_robots -= 1
            if self.remaining_robots == 0:
                self.computation_done_future.set_result(True)

    @staticmethod
    def calculate_path_distance(path):
        """Calculate the total distance of the path."""
        total_distance = 0.0
        for i in range(1, len(path.poses)):
            p1 = path.poses[i-1].pose.position
            p2 = path.poses[i].pose.position
            total_distance += math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
        return total_distance

def main():
    parser = argparse.ArgumentParser(description='Compute path for multiple robots.')
    parser.add_argument('--x', type=float, required=True, help='Target position x')
    parser.add_argument('--y', type=float, required=True, help='Target position y')
    parser.add_argument('--robot_names', nargs='+', required=True, help='List of robot names')
    args = parser.parse_args()

    rclpy.init()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = args.x
    goal_pose.pose.position.y = args.y
    goal_pose.pose.orientation.w = 1.0

    multi_robot_path_distance = MultiRobotPathDistance(args.robot_names, goal_pose)

    # Wait for all robots to compute their paths
    rclpy.spin_until_future_complete(multi_robot_path_distance, multi_robot_path_distance.computation_done_future)

    rclpy.shutdown()
    print(json.dumps(multi_robot_path_distance.results))
    # with open('results.json', 'w') as f:
    #     json.dump(multi_robot_path_distance.results, f, indent=4)

if __name__ == '__main__':
    main()