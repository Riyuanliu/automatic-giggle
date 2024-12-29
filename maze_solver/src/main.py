#!/usr/bin/env python3

import rospy
from TurtlebotDriving import TurtlebotDriving

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import yaml
import logging
import time

from maze import Maze
from algorithm.wallfollower import WallFollower

# Configuration
config = {
    "map_dir": "maps",
    "map_info": "map1.yaml",
    "algorithm": "wallfollowing"  # Fixed to wallfollowing
}

# Input Handling
def load_map():
    try:
        os.chdir('./src/automatic-giggle/maze_solver')
        print("Current working directory:", os.getcwd())
        
        # List items in the current directory
        print("Directory contents:", os.listdir(os.getcwd()))
        
        logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

        # Open map yaml file
        with open(os.path.join(config["map_dir"], config["map_info"])) as file:
            map_config = yaml.load(file, Loader=yaml.FullLoader)

        # Read image
        input_map = cv2.imread(os.path.join(config["map_dir"], map_config["image"]), -1)

        # Make wall = 1, path = 0
        input_map = (input_map != 254).astype(int)

        return input_map, map_config
    
    except Exception as e:
        print(f"Error: {e}")
        raise


# Maze Creation
def create_maze(input_map):
    print("Creating Maze...")
    t0 = time.time()
    maze = Maze(input_map)
    t1 = time.time()

    print("\nNode Count:", maze.nodecount)
    print("Time elapsed:", t1 - t0, "\n")
    return maze

# Solve the Maze
def solve_maze():
    logging.info(f"Starting Solve with Wall Following")
    algorithm = WallFollower(speed=0.2, distance_wall=0.4, side="left")
    return algorithm.run()

# Display Solution Path
def display_solution(input_map, path):
    input_map = (input_map == 0).astype(int)
    for x, y in path:
        input_map[x, y] = 2

    plt.imshow(input_map, cmap="gray")
    plt.title("Maze Solution")
    plt.axis("off")
    plt.show()

# Main Function
def main():
    input_map, map_config = load_map()
    maze = create_maze(input_map)

    # Solve Maze
    path, length, timetaken, completed = solve_maze()

    if completed:
        print("Path found:")
        print(path)
        print("Path length:", length)
    else:
        print("\nNo path found")

    # Display solution
    display_solution(input_map, path)

if __name__ == '__main__':
    main()
