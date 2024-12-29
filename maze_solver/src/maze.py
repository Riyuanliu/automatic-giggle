import numpy as np

class Maze:
    class Node:
        def __init__(self, position=None):
            """
            Initializes a node with a given position and empty neighbors.
            :param position: Tuple (x, y) representing the position of the node.
            """
            self.position = position
            self.neighbours = [None, None, None, None]  # [left, right, top, bottom]

        def __lt__(self, other):
            """Comparison of nodes based on position."""
            return self.position < other.position

        def __gt__(self, other):
            """Comparison of nodes based on position."""
            return self.position > other.position

        def __le__(self, other):
            """Less than or equal comparison based on position."""
            return self < other or self == other

        def __ge__(self, other):
            """Greater than or equal comparison based on position."""
            return self > other or self == other

    def __init__(self, arr):
        """
        Initializes the maze using a 2D array where 0 represents an open space
        and 1 represents a wall. The maze will identify the start and end positions.
        
        :param arr: 2D list representing the maze structure.
        """
        maze = np.array(arr)
        self.width = maze.shape[0]
        self.height = maze.shape[1]

        self.start = None
        self.end = None
        self.nodecount = 0
        self.nodes = {}

        # Initialize start and end nodes
        self._initialize_start_and_end(maze)
        
        # Process the maze to create nodes and set neighbors
        self._process_maze(maze)

    def _initialize_start_and_end(self, maze):
        """
        Identifies the start and end nodes based on the maze structure.
        The start node is placed at the top row (first available space),
        and the end node is placed at the bottom row (first available space).
        """
        # Find start node (first open space in the top row)
        for y in range(self.height):
            if maze[0, y] == 0:
                self.start = Maze.Node((0, y))
                self.nodecount += 1
                break

        # Find end node (first open space in the bottom row)
        for y in range(self.height):
            if maze[-1, y] == 0:
                self.end = Maze.Node((self.height - 1, y))
                self.nodecount += 1
                break

    def _process_maze(self, maze):
        """
        Processes the maze array, creating nodes for open spaces and setting
        the neighbor relationships (left, right, top, bottom).
        """
        # Create a list to store top-row nodes for neighbor connections
        toprownodes = [None] * self.width

        for x in range(1, self.width - 1):
            left = None

            for y in range(1, self.height - 1):
                current_is_open = maze[x, y] == 0
                prev_is_open = maze[x, y - 1] == 0 if y > 0 else False
                next_is_open = maze[x, y + 1] == 0 if y < self.height - 1 else False

                # Skip if the current position is a wall
                if not current_is_open:
                    continue

                # Create a new node for open space
                node = Maze.Node((x, y))

                # Handle left neighbor if previous cell was open
                if prev_is_open:
                    left.neighbours[1] = node
                    node.neighbours[3] = left

                # Set the left reference for next iteration
                left = node

                # Link vertical neighbors (top/bottom)
                if maze[x - 1, y] == 0:  # Top neighbor exists
                    t = toprownodes[y]
                    if t is not None:  # Ensure t is not None
                        t.neighbours[2] = node
                        node.neighbours[0] = t

                # Update the top row node if we are at the bottom of the column
                if maze[x + 1, y] == 0:  # Bottom neighbor exists
                    toprownodes[y] = node
                else:
                    toprownodes[y] = None

                self.nodecount += 1


    def print_maze(self):
        """
        A helper function to print the maze structure in a readable format.
        """
        for y in range(self.height):
            row = ""
            for x in range(self.width):
                if (x, y) == self.start.position:
                    row += "S"  # Start
                elif (x, y) == self.end.position:
                    row += "E"  # End
                elif (x, y) in self.nodes:
                    row += "O"  # Open space
                else:
                    row += "#"  # Wall
            print(row)
