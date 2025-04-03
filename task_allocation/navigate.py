import json
import subprocess
import numpy as np
from scipy.optimize import linear_sum_assignment
import sqlite3
from datetime import datetime
from concurrent.futures import ThreadPoolExecutor
import time

db_path = 'tasks.db'

def hungarian_algorithm(cost_matrix):
    cost_matrix = np.array(cost_matrix)

    # Add dummy columns if there are more rows than columns
    if cost_matrix.shape[0] > cost_matrix.shape[1]:
        dummy_targets = np.zeros((cost_matrix.shape[0], cost_matrix.shape[0] - cost_matrix.shape[1]))
        cost_matrix = np.hstack((cost_matrix, dummy_targets))

    # Add dummy rows if there are more columns than rows
    if cost_matrix.shape[0] < cost_matrix.shape[1]:
        dummy_bots = np.full((cost_matrix.shape[1] - cost_matrix.shape[0], cost_matrix.shape[1]), 1e6)  # Use a large number instead of np.inf
        cost_matrix = np.vstack((cost_matrix, dummy_bots))

    # Perform the assignment
    row_ind, col_ind = linear_sum_assignment(cost_matrix)
    optimal_cost = cost_matrix[row_ind, col_ind].sum()

    return row_ind, col_ind, optimal_cost

def get_namespaces_from_db():
    namespaces = []
    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()
        cursor.execute("SELECT namespace FROM AvailableBots")
        rows = cursor.fetchall()
        namespaces = [row[0] for row in rows]
    return namespaces

def get_pending_tasks():
    tasks = []
    with sqlite3.connect(db_path) as connection:
        cursor = connection.cursor()
        cursor.execute("SELECT id, x_coordinate, y_coordinate FROM Pending")
        rows = cursor.fetchall()
        tasks = [{"id": row[0], "x": row[1], "y": row[2]} for row in rows]
    return tasks

def run_navigate_to_pose(x, y, namespace, task_id, distance_json, timestamp_added):
    cmd = [
        'python3', 'navigate_to_pose.py',
        '--x', str(x),
        '--y', str(y),
        '--namespace', namespace,
        '--id', str(task_id),
        '--distance_json', distance_json,
        '--timestamp_added', timestamp_added
    ]
    subprocess.run(cmd)

def main(args=None):
    while True:
        # Read the targets from the Pending table
        targets = get_pending_tasks()

        if not targets:
            print("No pending tasks found. Exiting.")
            break

        # Define the list of robot namespaces
        namespaces = get_namespaces_from_db()
        compute_file = 'install/turtlebot3_multi_robot/lib/turtlebot3_multi_robot/scripts/compute_path.py'

        # Compute paths for all robots to all targets and capture the output
        results = []
        for target in targets:
            cmd = [
                'python3', compute_file, 
                '--x', str(target['x']), 
                '--y', str(target['y']), 
                '--robot_names'
            ] + namespaces
            result = subprocess.run(cmd, capture_output=True, text=True)
            target_results = json.loads(result.stdout)
            results.append(target_results)

        print(results)

        # Convert array of dicts to cost matrix, handling missing values
        cost_matrix = []
        for result in results:
            row = []
            for robot in namespaces:
                row.append(result.get(robot, 1e6))  # Use a large number for missing values
            cost_matrix.append(row)

        # Allocate targets to robots using Hungarian algorithm
        row_ind, col_ind, optimal_cost = hungarian_algorithm(cost_matrix)
        allocated_targets = {}
        for row, col in zip(row_ind, col_ind):
            if row < len(targets) and col < len(namespaces):
                allocated_targets[namespaces[col]] = targets[row]
        print(allocated_targets)

        # Convert row_ind and targets to lists
        row_ind = row_ind.tolist()
        targets_list = list(targets)

        # Send goals to the allocated robots using navigate_to_pose.py in parallel
        with ThreadPoolExecutor() as executor:
            futures = []
            timestamp_added = datetime.now().isoformat()
            for robot_name, target in allocated_targets.items():
                distance_json = json.dumps(results[row_ind.index(targets_list.index(target))])
                futures.append(executor.submit(
                    run_navigate_to_pose,
                    target['x'],
                    target['y'],
                    robot_name,
                    target['id'],
                    distance_json,
                    timestamp_added
                ))

            # Wait for all futures to complete
            for future in futures:
                future.result()

        # Add a delay between iterations to avoid excessive querying
        time.sleep(5)

if __name__ == '__main__':
    main()