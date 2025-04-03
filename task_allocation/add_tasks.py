import argparse
import json
import sqlite3
import datetime
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

db_path = 'tasks.db'

def add_tasks_to_db(tasks):
    # Connect to the SQLite database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Create the Pending table if it doesn't exist
    cursor.execute('''
    CREATE TABLE IF NOT EXISTS Pending (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        x_coordinate REAL NOT NULL,
        y_coordinate REAL NOT NULL,
        timestamp_added TEXT NOT NULL
    )
    ''')
    current_datetime = datetime.datetime.now().isoformat()

    # Insert the coordinates into the Pending table
    for task in tasks:
        cursor.execute('INSERT INTO Pending (x_coordinate, y_coordinate, timestamp_added) VALUES (?, ?, ?);',
                       (task['x'], task['y'], current_datetime))

    # Commit the transaction and close the connection
    conn.commit()
    conn.close()
    print(f"{len(tasks)} tasks added to the database.")

class RvizPointSubscriber(Node):
    def __init__(self, db_path):
        super().__init__('rviz_point_subscriber')
        self.db_path = db_path
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',  # Topic where points from RViz are published
            self.point_callback,
            10
        )
        self.get_logger().info("Subscribed to /clicked_point.")

    def point_callback(self, msg):
        # Extract the point coordinates from the message
        x = msg.point.x
        y = msg.point.y
        current_datetime = datetime.datetime.now().isoformat()

        # Insert the received coordinates into the database
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        cursor.execute('''
        INSERT INTO Pending (x_coordinate, y_coordinate, timestamp_added)
        VALUES (?, ?, ?);
        ''', (x, y, current_datetime))
        conn.commit()
        conn.close()
        self.get_logger().info(f"Received point: x={x}, y={y} and added to the database.")

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Add tasks to the database and subscribe to RViz points.")
    parser.add_argument(
        "--tasks",
        type=str,
        required=False,
        help="Tasks in JSON format. Example: '[{\"x\":1.0, \"y\":2.0}, {\"x\":3.0, \"y\":4.0}]'"
    )
    args = parser.parse_args()

    # If tasks are provided via command line, add them to the database
    if args.tasks:
        try:
            # Parse JSON string into a Python object
            tasks = json.loads(args.tasks)

            # Validate task structure
            if not all('x' in task and 'y' in task for task in tasks):
                print("Error: Each task must contain 'x' and 'y' fields.")
                return

            # Add tasks to the database
            add_tasks_to_db(tasks)
        except json.JSONDecodeError:
            print("Error: Invalid JSON format. Make sure the tasks are in valid JSON format.")

    # Start ROS2 node to subscribe to RViz point data
    rclpy.init()
    rviz_subscriber = RvizPointSubscriber(db_path)
    rclpy.spin(rviz_subscriber)

    # Shutdown ROS2 node after use
    rviz_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()