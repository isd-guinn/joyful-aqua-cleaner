#from numpy import matrix as np

class Cell:
    def __init__(self):
        self.status = 'u'  # Default status

    def set_wall(self):
        self.status = 'w'

    def set_floor(self):
        self.status = 'f'

    def __repr__(self):
        return self.status  # For easy printing (w, f, u)
        

class Map:
    def __init__(self):
        self.y_size = 2
        self.x_size = 2
        # Initialize a 2x2 grid with Cell instances
        self.map = [[Cell() for _ in range(self.x_size)] for _ in range(self.y_size)]
        self.current_position = (1, 1)  # Starting position
        self.starting_position = (1, 1)
        self.closed_loop_detected = False  # Flag for closed loop
        

    def update_map(self, x, y, status):
        """Mark the area with the given status."""
        
        # Extend map size
        if y >= len(self.map):
            for _ in range(len(self.map), y + 1):
                self.map.append([Cell() for _ in range(len(self.map[0]) if self.map else 1)])
        if x >= len(self.map[y]):
            for row in self.map:
                row.extend([Cell() for _ in range(x - len(row) + 1)])

        # Update the cell's status
        if status == 'wall':
            self.map[y][x].set_wall()
        elif status == 'floor':
            self.map[y][x].set_floor()

    def print_map(self, x, y, direction):
        #print("In print map: ", x, ", ", y)
        """Print the current state of the map with the robot's location."""
        self.current_position = x, y
        robot_x, robot_y = self.current_position
        for y, row in enumerate(self.map):
            for x, cell in enumerate(row):
                if (x, y) == (robot_x, robot_y):
                    print('R', end=' ')  # Representing the robot with 'R'
                else:
                    print(str(cell), end=' ')
            print()  # New line after each row
        print("Direction of car: ", direction)

    def set_position(self, x, y, status):
        """Update the robot's current position and mark the cell status."""
        self.current_position = (x, y)
        self.update_map(x, y, status)  # Mark the position as discovered

        # Check for closed loop
        if self.current_position == self.starting_position:
            self.closed_loop_detected = True
            print("Closed loop detected! Stopping map resizing.")


    
def convert_corners_to_walls(map_instance):
    """Convert corners of walls to walls in the given map in place."""
    rows = len(map_instance.map)
    cols = len(map_instance.map[0])

    for i in range(rows):
        for j in range(cols):
            # Check if the current cell is 'u' (undiscovered) and could be a corner
            if map_instance.map[i][j].status == 'u':
                # Check surrounding cells to determine if it's a corner
                if (
                    (i > 0 and j > 0 and map_instance.map[i-1][j].status == 'w' and map_instance.map[i][j-1].status == 'w') or  # Top-left corner
                    (i > 0 and j < cols - 1 and map_instance.map[i-1][j].status == 'w' and map_instance.map[i][j+1].status == 'w') or  # Top-right corner
                    (i < rows - 1 and j > 0 and map_instance.map[i+1][j].status == 'w' and map_instance.map[i][j-1].status == 'w') or  # Bottom-left corner
                    (i < rows - 1 and j < cols - 1 and map_instance.map[i+1][j].status == 'w' and map_instance.map[i][j+1].status == 'w')  # Bottom-right corner
                ):
                    map_instance.map[i][j].status = 'w'  # Convert to wall
