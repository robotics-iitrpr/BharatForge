import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid , Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose
import numpy as np
import heapq , math , time , threading
import scipy.interpolate as si
import datetime
import argparse
from functools import partial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import OccupancyGrid


## Getting number of bots
parser = argparse.ArgumentParser(description="Argparse.")
parser.add_argument('--bots', type=int, required=True, help='Your name.')
args = parser.parse_args()

lookahead_distance = 0.22
speed = 0.18
expansion_size = 4
target_error = 0.15

TB_PATH = []
TB_PATHF = []
for i in range(args.bots):
    TB_PATH.append([(0,0)])
    TB_PATHF.append(0)
VISITED = []

def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    close_set = set()
    came_from = {}
    gscore = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:                
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False

def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i],ry[i]) for i in range(len(rx))]
    except:
        path = array
    return path

def frontierB(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i-1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix)-1 and matrix[i+1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j-1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i])-1 and matrix[i][j+1] < 0:
                    matrix[i][j] = 2
    return matrix

def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups

def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups) 
    dfs(matrix, i - 1, j - 1, group, groups) 
    dfs(matrix, i - 1, j + 1, group, groups) 
    dfs(matrix, i + 1, j - 1, group, groups) 
    return group + 1

def fGroups(groups):
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]), reverse=True)
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) > 2]    
    return top_five_groups

def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    return centroid

def visitedControl(targetP):
    global VISITED
    for i in range(len(VISITED)):
        k = VISITED[i]
        d = math.sqrt((k[0] - targetP[0])**2 + (k[1] - targetP[1])**2)
        if d < 0.2:
            return 1
    return 0

def findClosestGroup(matrix,groups, current,resolution,originX,originY,choice):
    global TB_PATH
    global TB_PATHF
    targetP = None
    distances = []
    paths = []
    lengths = []
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],[p[1] for p in groups[i][1]]) 
        t = (middle[1]*resolution+originX,middle[0]*resolution+originY)
        if visitedControl(t) == 0:
            path = astar(matrix, current, middle)
            path = [(p[1]*resolution+originX,p[0]*resolution+originY) for p in path]
            total_distance = pathLength(path)
            distances.append(total_distance)
            paths.append(path)
            lengths.append(len(groups[i][1]))
    
    arrays = list(zip(lengths,distances,paths))
    arrays = sorted(arrays, key=lambda x: x[1], reverse=False)
    
    arrays_a = [a for a in arrays if a[1] > target_error*2]
    p1 = [a for a in arrays_a if a[1] < 2.0]
    
    p1 = sorted(p1, key=lambda x: x[0], reverse=True)
    
    if len(p1) > 0:
        targetP = p1[0][2]
    else:
        p1 = [a for a in arrays_a if a[1] < 4.0]
        p1 = sorted(p1, key=lambda x: x[0], reverse=True)
        if len(p1) > 0:
            targetP = p1[0][2]
    if targetP == None:
        arrays_a = sorted(arrays, key=lambda x: x[0], reverse=True)
        targetP = arrays_a[0][2]
    if targetP == None:
        
        p1 = [a for a in arrays if a[0] == max(lengths)]
        if len(p1) > 0:
            targetP = p1[0][2]
    return targetP

def pathLength(path):
    for i in range(len(path)):
        path[i] = (path[i][0],path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:,0], differences[:,1])
    total_distance = np.sum(distances)
    return total_distance

def costmap(data,width,height,resolution):
    data = np.array(data).reshape(height,width)
    wall = np.where(data == 100)
    for i in range(-expansion_size,expansion_size+1):
        for j in range(-expansion_size,expansion_size+1):
            if i  == 0 and j == 0:
                continue
            x = wall[0]+i
            y = wall[1]+j
            x = np.clip(x,0,height-1)
            y = np.clip(y,0,width-1)
            data[x,y] = 100
    data = data*resolution
    return data

def exploration(data,width,height,resolution,column,row,originX,originY,choice):
    global TB_PATH
    global TB_PATHF
    f = 1
    data = costmap(data,width,height,resolution)
    data[row][column] = 0
    data[data > 5] = 1
    data = frontierB(data)
    data,groups = assign_groups(data)
    groups = fGroups(groups)
    if len(groups) == 0:
        f = -1
    else:
        data[data < 0] = 1
        path = findClosestGroup(data,groups,(row,column),resolution,originX,originY,choice)
        if path != None:
            path = bspline_planning(path,len(path)*5)
        else:
            f = -1
    if f == -1:
        TB_PATHF[choice] = f
    else:
        TB_PATH[choice] = path
        TB_PATHF[choice] = f
    return

def get(choice):
    now = datetime.datetime.now()
    time_string = now.strftime("%H:%M:%S")
    print(f"[INFO] {time_string}: {choice+1}. Robot received route request")

def response(choice):
    now = datetime.datetime.now()
    time_string = now.strftime("%H:%M:%S")
    print(f"[INFO] {time_string}: {choice+1}. Robot responded to request")

class HeadquartersControl(Node):
    def __init__(self):
        super().__init__('Exploration')
        self.subscription = self.create_subscription(OccupancyGrid,'merge_map',self.map_callback,10)
        for i in range(args.bots):
            setattr(self, f'publisher_tb3_'+str(i)+'_path', self.create_publisher(Float32MultiArray,'tb3_'+str(i)+'/path', 10))
            setattr(self, f'subscription_tb3_'+str(i)+'_odom', self.create_subscription(Odometry,'tb3_'+str(i)+'/odom',partial(self.tb_odom_callback, tb_id=i),10))
            setattr(self, f'subscription_tb3_'+str(i)+'_cmd_vel', self.create_subscription(Twist,'tb3_'+str(i)+'/cmd_vel',partial(self.tb_status_control, tb_id=i),4))
        print("Discover Code")
        self.Exploration = True
        for i in range(args.bots):
            threading.Thread(target=self.start_exploration, args=(i,)).start() # Exploration thread - Robot

    def start_exploration(self, tb_id):
        while True:
            if not hasattr(self, 'map_data') or not hasattr(self, 'tb'+str(tb_id)+'_x'):
                continue
            tb_x = getattr(self, f'tb{tb_id}_x', None)
            tb_y = getattr(self, f'tb{tb_id}_y', None)
            c_tb = int((tb_x - self.originX)/self.resolution)
            setattr(self, f'c_tb{tb_id}', c_tb)
            r_tb = int((tb_y - self.originY)/self.resolution)
            setattr(self, f'r_tb{tb_id}', r_tb)
            if self.Exploration:
                if TB_PATHF[tb_id] == 0:
                    get(tb_id)
                    exploration(self.data,self.width,self.height,self.resolution,c_tb,r_tb,self.originX,self.originY,tb_id)
                    response(tb_id)
                    self.tb_path_pub(tb_id)
                    setattr(self, f'tb{tb_id}_s', False)
                if TB_PATHF[tb_id] == -1:
                    self.Exploration = False
                    print("[INFO] Exploration Completed")
                    break
                time.sleep(0.1)
                tb_s = getattr(self, f'tb{tb_id}_s', None)
                if tb_s == True:
                    get(tb_id)
                    test = getattr(self, f't{tb_id}', None)
                    test.join()
                    response(tb_id)
                    self.tb_path_pub(tb_id)
                time.sleep(0.4)
                
    def target_callback(self,msg):
        c = int((TB_PATH[msg][-1][0] - self.originX)/self.resolution)
        r = int((TB_PATH[msg][-1][1] - self.originY)/self.resolution)
        exploration(self.data,self.width,self.height,self.resolution,c,r,self.originX,self.originY,msg)

    def map_callback(self,msg):
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data

    def tb_odom_callback(self,msg, tb_id):
        setattr(self, f'tb{tb_id}_x', msg.pose.pose.position.x)
        setattr(self, f'tb{tb_id}_y', msg.pose.pose.position.y)

    def tb_status_control(self,msg,tb_id):
        if msg.linear.x == 0 and msg.angular.z == 0:
            setattr(self, f'tb{tb_id}_s', True)
        else:
            setattr(self, f'tb{tb_id}_s', False)

    def tb_path_pub(self,tb_id):
        global TB_PATH
        global VISITED
        VISITED.append(TB_PATH[tb_id][-1])
        message = Float32MultiArray()
        message.data = [elem for tup in TB_PATH[tb_id] for elem in tup]
        test = getattr(self, f'publisher_tb3_{tb_id}_path', None)
        test.publish(message)
        t = pathLength(TB_PATH[tb_id])/speed
        t = t - 0.2
        if t < 0:
            t = 0
        thread = threading.Timer(t, self.target_callback,args=(tb_id,))
        setattr(self, f't{tb_id}', thread)
        thread.start()


def main(args=None):
    rclpy.init(args=args)
    headquarters_control = HeadquartersControl()
    rclpy.spin(headquarters_control)
    headquarters_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
