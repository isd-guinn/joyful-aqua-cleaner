# what action to return when entered the go_to function?
# how to pass the action right out from here?

import math
import time
from . import mapping
# import mapping

# Create an instance of the Map class
#map_instance = mapping.Map()

# Global variables representing sensor data
R_ang = 0
L_ang = 0
acel = 0
x = 1
y = 1
direction = 0
wall_flag = False
last_wall_is_r = False
first_loop_done = False

STOP = 0
FORWARD = 1
BACKWARD = 2
ANTICLOCKWISE = 3
CLOCKWISE = 4

# def #read_sensors(map_instance)
# return:
#     global R_ang, L_ang, acel  # Declare all globals that will be modified

#     Read sensor data
#     try:
#         R_ang = int(input("Enter right angle (R_ang): "))
#         L_ang = int(input("Enter left angle (L_ang): "))
#         acel = int(input("Enter acceleration (acel): "))
#         #x = int(input("Enter global x-coordinate: "))
#         #y = int(input("Enter global y-coordinate: "))
#     except ValueError:
#         print("Invalid input. Defaulting to 0 for all.")
#         R_ang = L_ang = acel = 0
#     R_ang = L_ang = acel = 0
#     #print("x is now: ", x, "    y is now: ", y)
#     return x, y, direction

def go_straight():
    global x, y
    new_x, new_y = calculate_coordinates(x, y, "front", direction)
    print("Going straight")
    x, y = new_x, new_y
    return FORWARD

def go_right():
    global x, y, direction
    new_x, new_y = calculate_coordinates(x, y, "right", direction)
    print("Going right")
    #x, y = new_x, new_y
    direction = (direction + 90)%360
    return CLOCKWISE

def go_left():
    global x, y, direction
    new_x, new_y = calculate_coordinates(x, y, "left", direction)
    print("Going left")
    #x, y = new_x, new_y
    direction = (direction + 360 - 90)%360
    return ANTICLOCKWISE

def calculate_coordinates(x, y, direction, car_direction):
    # Implementation remains unchanged
    #print("In calculate coordinate, car_direction: ", car_direction)
    #print("In calculate coordinate, direction: ", direction)
    #print("In calculate_coordinate: ", x,", ", y)
    new_x, new_y = x, y
    if car_direction == 0:
        if(direction == "front"):
            new_x = x 
            new_y = y - 1
        if(direction == "left"):
            new_x = x - 1
            new_y = y 
        if(direction == "right"):
            new_x = x + 1
            new_y = y

    elif car_direction == 90:
        if(direction == "front"):
            new_x = x + 1
            new_y = y 
        if(direction == "left"):
            new_x = x 
            new_y = y - 1
        if(direction == "right"):
            new_x = x 
            new_y = y + 1

    elif car_direction == 180:
        if(direction == "front"):
            new_x = x 
            new_y = y + 1
        if(direction == "left"):
            new_x = x + 1
            new_y = y 
        if(direction == "right"):
            new_x = x - 1
            new_y = y

    elif car_direction == 270:
        if(direction == "front"):
            new_x = x - 1
            new_y = y 
        if(direction == "left"):
            new_x = x
            new_y = y + 1 
        if(direction == "right"):
            new_x = x 
            new_y = y - 1
    else:
        raise ValueError("Invalid move_direction. Use 'front', 'back', 'left', or 'right'.")

    #print("Returning ", new_x, ", ", new_y)
    return new_x, new_y

def mark_cells(map_instance, read_x, read_y, foc_left, foc_right, is_moving):
    """Mark cells based on current sensor readings."""
    global wall_flag, last_wall_is_r, x, y, L_ang, R_ang

    x, y = read_x, read_y
    L_ang, R_ang = foc_left, foc_right

    map_instance.update_map(x, y, 'floor')
    if not is_moving: # sth in front
        front_x, front_y = calculate_coordinates(x, y, "front", direction)
        map_instance.update_map(front_x, front_y, 'wall')
        #print("Marked ", front_x, ",", front_y, "as wall")
    
    if L_ang >= 100: # sth on the left
        left_x, left_y = calculate_coordinates(x, y, "left", direction)
        map_instance.update_map(left_x, left_y, 'wall')
        #print("Marked ", left_x, ",", left_y, "as wall")
        wall_flag = True
        last_wall_is_r = False

    if R_ang >= 100: # sth on the right
        right_x, right_y = calculate_coordinates(x, y, "right", direction)
        map_instance.update_map(right_x, right_y, 'wall')
        #print("Marked ", right_x, ",", right_y, "as wall")
        wall_flag = True
        last_wall_is_r = True

    if is_moving: # moving -> front clear
        front_x, front_y = calculate_coordinates(x, y, "front", direction)
        map_instance.update_map(front_x, front_y, 'floor')
        #print("Marked ", front_x, ",", front_y, "as floor")

    if L_ang < 90: # left clear
        left_x, left_y = calculate_coordinates(x, y, "left", direction)
        map_instance.update_map(left_x, left_y, 'floor')
        #print("Marked ", left_x, ",", left_y, "as floor")

    if R_ang < 90: # right clear
        right_x, right_y = calculate_coordinates(x, y, "right", direction)
        map_instance.update_map(right_x, right_y, 'floor')
        #print("Marked ", right_x, ",", right_y, "as floor")

def is_undiscovered(map_instance, x, y): # (front, left, right) bool for undiscovered
    """Check for walls in front, left, and right of the robot, including all cells in that direction."""
    # print("In have wall: ", x, ", ", y)
    # Define direction offsets for checking
    offsets = {
        0: {
            'front': (0, -1),
            'left': (-1, 0),
            'right': (1, 0)
        },
        90: {
            'front': (1, 0),
            'left': (0, -1),
            'right': (0, 1)
        },
        180: {
            'front': (0, 1),
            'left': (1, 0),
            'right': (-1, 0)
        },
        270: {
            'front': (-1, 0),
            'left': (0, 1),
            'right': (0, -1)
        }
    }

    discoverable = {}
    
    for position in ['front', 'left', 'right']:
        dx, dy = offsets[direction][position]
        current_x, current_y = x + dx, y + dy
        
        # Check all cells in that direction
        while 0 <= current_x < len(map_instance.map[0]) and 0 <= current_y < len(map_instance.map):
            # WIthin the map boundary
            #print("Checking ", current_y, ", ", current_x, " which is ", map_instance.map[current_y][current_x])
            #print("T/F? ", map_instance.map[current_y][current_x].status == 'w')
            if map_instance.map[current_y][current_x].status == 'u':
                discoverable[position] = True
                break
            if map_instance.map[current_y][current_x].status == 'w':
                #print("Wall found in ", position)
                discoverable[position] = False  # Wall found
                break
            current_x += dx
            current_y += dy

            discoverable[position] = True  # No walls found in that direction

    return discoverable

def find_nearest_undiscovered_grid(map_instance):
    """Find the coordinates of the nearest undiscovered grid ('u') from the current position."""
    global x, y
    
    print("Finding nearest undiscovered")
    # Start checking layers from distance 1 outward
    distance = 1
    
    while True:
        # Check all positions at the current distance
        for dx in range(-distance, distance + 1):
            for dy in range(-distance, distance + 1):
                if abs(dx) + abs(dy) == distance:  # To keep within the diamond shape
                    check_x = x + dx
                    check_y = y + dy
                    
                    # Check bounds
                    if 0 <= check_x < len(map_instance.map[0]) and 0 <= check_y < len(map_instance.map):
                        if map_instance.map[check_y][check_x].status == 'u':
                            return check_x, check_y  # Return the coordinates of the first undiscovered grid found

        # Increment distance for the next layer
        distance += 1
        
        # Check if we've reached the boundaries of the map
        if distance > max(len(map_instance.map), len(map_instance.map[0])):
            break  # Stop if we've exceeded the map boundaries

    return None  # If no undiscovered grid found

def is_closed_wall(map_instance, wall_x, wall_y):
    """Check if the wall loop is closed at the given coordinates."""
    
    # Define the directions to check (8 neighboring cells)
    directions = [
        (-1, -1), (-1, 0), (-1, 1),  # Top-left, Top, Top-right
        (0, -1),            (0, 1),   # Left, Right
        (1, -1), (1, 0), (1, 1)       # Bottom-left, Bottom, Bottom-right
    ]
    
    # Count adjacent walls
    wall_neighbor_count = 0
    
    # Check each direction
    for dx, dy in directions:
        check_x = wall_x + dx
        check_y = wall_y + dy
        
        # Check bounds before accessing the neighbor
        if 0 <= check_x < len(map_instance.map[0]) and 0 <= check_y < len(map_instance.map):
            if map_instance.map[check_y][check_x].status == 'w':
                wall_neighbor_count += 1

    # A wall is considered closed if it has 8 neighboring walls (or fewer depending on its position)
    # Here, we can adjust the threshold based on the wall's position in the grid
    if wall_neighbor_count >= 2:  # Adjust the threshold as needed
        return True
    else:
        return False

def find_nearest_unclosed_wall(map_instance, robot_x, robot_y):
    """Find the nearest unclosed wall from the robot's location."""
    
    # Check each grid cell in the surrounding area
    for j in range(max(0, robot_y - 1), min(len(map_instance.map), robot_y + 2)):
        for i in range(max(0, robot_x - 1), min(len(map_instance.map[0]), robot_x + 2)):
            # Check if the current cell is a wall
            if map_instance.map[j][i].status == 'w':
                # Use the is_closed_wall function to check if this wall is closed
                if not is_closed_wall(map_instance, i, j):
                    return (i, j)  # Return the coordinates of the unclosed wall

    return None  # Return None if no unclosed wall is found

def follow_wall(map_instance, car_x, car_y, car_direction, L_ang, R_ang, is_moving):
    print("Following wall")
    global wall_flag, x, y, direction
    x, y = car_x, car_y
    direction = car_direction
    # Robot should be still touching the wall right now
    # Get wall coordinates
    if last_wall_is_r:
        wall_x, wall_y = calculate_coordinates(x, y, "right", direction)
    else:
        wall_x, wall_y = calculate_coordinates(x, y, "left", direction)
    while not is_closed_wall(map_instance, wall_x, wall_y):
        if not is_moving:
            if L_ang > 90:
                return go_right()
            elif R_ang > 90:
                return go_left()
        elif is_moving:
            # When lose touch with a wall
            if L_ang == 45 and R_ang == 45: 
                print("Finding wall")
                if last_wall_is_r:
                    return go_right()
                else:
                    return go_left()
            else: 
                return go_straight()

    else:
        wall_flag = False
        mapping.convert_corners_to_walls(map_instance)
        return navigate(map_instance, x, y, is_moving, L_ang, R_ang)
    
def go_to(map_instance, target_x, target_y):
    """Navigate to a specific coordinate (target_x, target_y) step by step."""
    global x, y, direction

    if (x, y) == (target_x, target_y):
        return None  # Return None if the target is reached
    
    # Move towards the target x-coordinate
    if x < target_x:  # Need to move right
        while direction != 90:  # Adjust direction to face right
            return go_right() 
        next_x = x + 1
        if next_x < len(map_instance.map[0]) and map_instance.map[y][next_x].status != 'w':
            x = next_x
            return go_straight()  # Move right

    elif x > target_x:  # Need to move left
        while direction != 270:  # Adjust direction to face left
            return go_left()  # Implement your turn_left function
        next_x = x - 1
        if next_x >= 0 and map_instance.map[y][next_x].status != 'w':
            x = next_x
            return go_straight()  # Move left

    # Move towards the target y-coordinate
    if y < target_y:  # Need to move down
        while direction != 180:  # Adjust direction to face down
            return go_right()  # Implement your turn_right function
        next_y = y + 1
        if next_y < len(map_instance.map) and map_instance.map[next_y][x].status != 'w':
            y = next_y
            return go_straight  # Move down

    elif y > target_y:  # Need to move up
        while direction != 0:  # Adjust direction to face up
            return go_left()  # Implement your turn_left function
        next_y = y - 1
        if next_y >= 0 and map_instance.map[next_y][x].status != 'w':
            y = next_y
            return go_straight  # Move up

    # If the robot cannot move in any direction, handle accordingly
    print("Cannot move towards target. Current position:", (x, y))
    return None  # Return None if no valid action can be taken

def navigate(map_instance, car_x, car_y, is_moving, L_ang, R_ang):
    """Main navigation function to decide actions based on sensor data."""
    print("Navigating")
    global x, y, first_loop_done, direction, action
    x, y = car_x, car_y
    
    if wall_flag:
        follow_wall(map_instance, x, y, direction, L_ang, R_ang, is_moving)
        # wall_flag back to False

    # #front_x, front_y = calculate_coordinates(x, y, "front", direction)
    # if first_loop_done: # Done first loop will start going inside
    #     if map_instance.map[x][y].status != 'u':
    #         target_x, target_y = find_nearest_undiscovered_grid(map_instance)
    #         go_to(map_instance, target_x, target_y)


    undiscovered = is_undiscovered(map_instance, x, y) # check grid around
    print("Undiscovered: ", undiscovered) # debug
    # print(undiscovered) 
    # if undiscovered["front"]:
    if undiscovered['front']:
        return go_straight()
    # elif undiscovered["left"]:
    elif undiscovered['left']:
        return go_left()
    # elif undiscovered["right"]:
    elif undiscovered['right']:
        return go_right()
    else:
        print("Navigating to nearest undiscovered areas...")
        nearest = find_nearest_undiscovered_grid(map_instance)
        if nearest:
            target_x, target_y = nearest
            print(f"Going to nearest undiscovered grid at ({target_x}, {target_y}).")
            return go_to(map_instance, target_x, target_y)
        else:
            print("No more undiscovered area")
    #print("Navigation returning: ", x, ", ", y)
    map_instance.print_map(x, y, direction)
    return x, y, direction  # Return updated coordinates