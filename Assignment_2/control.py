import robot_params
import numpy as np 
import time

prev_heading_error = 0.0
total_heading_error = 0.0
previous_time = time.time()

def at_goal(robot_state, goal_state):    
    
    #check if we have reached goal point
    d = distance to goal point??
    
    if d <= robot_params.goal_threshold:
        return True
    else:
        return False

def gtg(robot_state, goal_state):  
    #The Go to goal controller
    
    global prev_heading_error
    global total_heading_error  
    global previous_time
    
    #Controller parameters
    Kp = 0.00656
    Kd = 0.0001
    K = 0.4
    
    #determine how far to rotate to face the goal point
    e_new = error between desired and current orientation of robot (in degress) ??
    
    #Remember to restrict error to (-180 ,180)
    
    current_time = time.time()
    dt = current_time-previous_time
    previous_time = current_time
    
    #PD controller for angular velocity
    e_dot = ??
    W = ??
    prev_heading_error = ??

    #find distance to goal
    d = distance to goal point??
    
    #P control for linear velocity
    V = ??
    
    #request robot to execute velocity
    return[V,W]
