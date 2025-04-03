from functools import partial
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import argparse


bots = 2

def merge_maps(maps):
    """
    Merge multiple occupancy grid maps into one.
    
    :param maps: List of OccupancyGrid maps to merge
    :return: A single merged OccupancyGrid map
    """
    if not maps:
        raise ValueError("No maps provided for merging.")

    # Initialize the merged map with the first map
    merged_map = maps[0]

    for next_map in maps[1:]:
        # Create a temporary merged map
        temp_map = OccupancyGrid()
        temp_map.header = merged_map.header
        temp_map.header.frame_id = 'merge_map'

        # Calculate the boundaries of the merged map
        min_x = min(merged_map.info.origin.position.x, next_map.info.origin.position.x)
        min_y = min(merged_map.info.origin.position.y, next_map.info.origin.position.y)
        max_x = max(
            merged_map.info.origin.position.x + (merged_map.info.width * merged_map.info.resolution),
            next_map.info.origin.position.x + (next_map.info.width * next_map.info.resolution),
        )
        max_y = max(
            merged_map.info.origin.position.y + (merged_map.info.height * merged_map.info.resolution),
            next_map.info.origin.position.y + (next_map.info.height * next_map.info.resolution),
        )

        temp_map.info.origin.position.x = min_x
        temp_map.info.origin.position.y = min_y
        temp_map.info.resolution = min(merged_map.info.resolution, next_map.info.resolution)
        temp_map.info.width = int(np.ceil((max_x - min_x) / temp_map.info.resolution))
        temp_map.info.height = int(np.ceil((max_y - min_y) / temp_map.info.resolution))
        temp_map.data = [-1] * (temp_map.info.width * temp_map.info.height)

        # Transfer data from the existing merged map
        for y in range(merged_map.info.height):
            for x in range(merged_map.info.width):
                i = x + y * merged_map.info.width
                merged_x = int(np.floor(
                    (merged_map.info.origin.position.x + x * merged_map.info.resolution - min_x) /
                    temp_map.info.resolution
                ))
                merged_y = int(np.floor(
                    (merged_map.info.origin.position.y + y * merged_map.info.resolution - min_y) /
                    temp_map.info.resolution
                ))
                merged_i = merged_x + merged_y * temp_map.info.width
                temp_map.data[merged_i] = merged_map.data[i]

        # Transfer data from the next map
        for y in range(next_map.info.height):
            for x in range(next_map.info.width):
                i = x + y * next_map.info.width
                merged_x = int(np.floor(
                    (next_map.info.origin.position.x + x * next_map.info.resolution - min_x) /
                    temp_map.info.resolution
                ))
                merged_y = int(np.floor(
                    (next_map.info.origin.position.y + y * next_map.info.resolution - min_y) /
                    temp_map.info.resolution
                ))
                merged_i = merged_x + merged_y * temp_map.info.width
                if temp_map.data[merged_i] == -1:  # Prefer existing data, if present
                    temp_map.data[merged_i] = next_map.data[i]

        # Update the merged map
        merged_map = temp_map

    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')
        self.publisher = self.create_publisher(OccupancyGrid, '/merge_map', 10)
        for i in range(bots):
            setattr(self, 'subscription', self.create_subscription(OccupancyGrid, '/tb3_'+str(i)+'/map', partial(self.map_callback, tb_id=i), 10))

        for i in range(bots):
            setattr(self, f'map{i + 1}', None)
        

    def map_callback(self, msg, tb_id):
        setattr(self, f'map{tb_id + 1}', msg)
        maps = []
        for i in range(bots):
            mapp = getattr(self, f'map{i + 1}', None)
            if mapp is not None:
                maps.append(mapp)
            else:
                 return   
        msg = merge_maps(maps)
        self.publisher.publish(msg)

def main(args=None):
    global bots
    ## Getting number of bots
    parser = argparse.ArgumentParser(description="Argparse.")
    parser.add_argument('--bots', type=int, required=True, help='Your name.')
    args, unknown = parser.parse_known_args(args=args)
    bots = args.bots
    rclpy.init(args=unknown)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
