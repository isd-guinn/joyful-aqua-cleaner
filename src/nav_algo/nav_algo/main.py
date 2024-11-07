# done integration for this file

# main.py
import time
from . import mapping
from . import navigation 
# import mapping
# import navigation
import math

# Global variables representing sensor data
# R_ang = navigation.R_ang
# L_ang = navigation.L_ang
# L_ang = 0
# R_ang = 0
# acel = navigation.acel
# direction = navigation.direction
# x  = navigation.x
# y = navigation.y

x, y= 1, 1  # Initial coordinates
direction = 0

map_instance = mapping.Map()

def run_nav_algo(foc_left, foc_right, is_moving, arg=None):
    global x, y, direction
     
    try:
        print("time: ", time.time())

        x = navigation.x
        y = navigation.y
        direction = navigation.direction

        navigation.mark_cells(map_instance, x, y, foc_left, foc_right, is_moving)

        wall_flag = navigation.wall_flag
        print("wall_flag: ", wall_flag) # debug
        if wall_flag:
            action = navigation.follow_wall(map_instance, x, y, direction, foc_left, foc_right, is_moving)
            #return action
        # L_ang = foc_left 
        # R_ang = foc_right
        else:
            action = navigation.navigate(map_instance, x, y, is_moving, foc_left, foc_right)
        
        x = navigation.x
        y = navigation.y
        direction = navigation.direction

        print("x, y after navigation: ", x, ", ", y)
        print("direction after navigation: ", direction)
        print("action: ", action)
        #time.sleep(1)
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        # return action
        print("program reached finally block") # debug
        return action
        

# while True:
#     # Get user inputs
#     L_ang = int(input("Enter the left angle (L_ang): "))
#     R_ang = int(input("Enter the right angle (R_ang): "))
#     is_moving_input = input("Is the system moving? (y/n): ")

#     # Convert the input for is_moving to a boolean
#     is_moving = is_moving_input.lower() == 'y'

#     # Call the function with the user inputs
#     run_nav_algo(L_ang, R_ang, is_moving)
    
#     map_instance.print_map(x, y, direction)