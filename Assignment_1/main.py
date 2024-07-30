#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time
import matplotlib.pyplot as mp

#Import files
import sim_interface
import robot_model

#Plotter setup  
mp.close('all')
fig = mp.figure() 
mp.axis([0.0, 10, 0.0, 10])
mp.ion()  
mp.xlabel('X(m)')
mp.ylabel('Y(m)') 

def plot_state(prev_state, current_state, color, alp):

    #Plot robot
    mp.plot(current_state[0], current_state[1], color, ms = 10.0)
    #plot trace
    mp.plot([prev_state[0], current_state[0]], [prev_state[1], current_state[1]], '-'+color, alpha = alp)
    return

def plot_ground_truth():
    
    global prev_robot_state
    
    robot_state = sim_interface.localize_robot()
    plot_state(robot_state, prev_robot_state, 'r', 0.5 )
    prev_robot_state = robot_state
    return

def plot_odom(v, w):
    
    global prev_odom_state
    
    odom_state = robot_model.simulate(prev_odom_state, [v, w], sim_interface.sim_timer())
    plot_state(odom_state, prev_odom_state, 'y', 0.5 )
    prev_odom_state = odom_state

def main():
    global prev_robot_state
    global prev_odom_state
    
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):
            
            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)
            
            #Obtain robots intial position and setup plots
            prev_robot_state =  sim_interface.localize_robot()
            prev_odom_state =  sim_interface.localize_robot()
            state = sim_interface.localize_robot()
            mp.plot([state[0], prev_odom_state[0]], [state[1], prev_odom_state[1]], '-y', alpha = 0.5, label='Odometry')
            mp.plot([state[0], prev_robot_state[0]], [state[1], prev_robot_state[1]], '-r', alpha = 0.5, label='Ground truth')
            mp.legend(loc="upper left")
            
            #Move robot straight
            for i in range(10):
                sim_interface.setvel_pioneers(0.2, 0.0)
                plot_odom(0.2, 0.0)
                plot_ground_truth()
                time.sleep(0.1)
                
            #Turn robot 
            for i in range(15):
                sim_interface.setvel_pioneers(0.0, 0.1)
                plot_odom(0.0, 0.1)
                plot_ground_truth()
                time.sleep(0.1)
                
            #Move robot straight
            for i in range(10):                
                sim_interface.setvel_pioneers(0.2, 0.0)
                plot_odom(0.2, 0.0)
                plot_ground_truth()
                time.sleep(0.1)
            
            #Turn robot 
            for i in range(15):
                sim_interface.setvel_pioneers(0.0, -0.1)
                plot_odom(0.0, -0.1)
                plot_ground_truth()
                time.sleep(0.1)
            
            #Move robot straight
            for i in range(10):                
                sim_interface.setvel_pioneers(0.2, 0.0)
                plot_odom(0.2, 0.0)
                plot_ground_truth()
                time.sleep(0.1)
            
            #Turn robot 
            for i in range(15):
                sim_interface.setvel_pioneers(0.0, -0.1)
                plot_odom(0.0, -0.1)
                plot_ground_truth()
                time.sleep(0.1)
            
            #Move robot straight
            for i in range(10):                
                sim_interface.setvel_pioneers(0.2, 0.0)
                plot_odom(0.2, 0.0)
                plot_ground_truth()
                time.sleep(0.1)
                
            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)
            
        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 