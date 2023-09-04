#!/usr/bin/env python

"""
Mobile robot simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time

#Import files
import sim_interface
import robot_params

def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Extract the maze segments from the simulation, could be used in planning
        obstacles = sim_interface.get_maze_segments()

        #Start simulation
        if (sim_interface.start_simulation()):
            
            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

            #Obtain goal state
            goal_state = sim_interface.get_goal_pose()

            #Obtain robots position
            robot_state = sim_interface.localize_robot()

            #Drive forward
            sim_interface.setvel_pioneers(robot_params.pioneer_max_V, 0.0)
            time.sleep(0.5)
            print("New robot pose", sim_interface.localize_robot())
            
            #turn            
            sim_interface.setvel_pioneers(0.0, robot_params.pioneer_max_W)
            time.sleep(0.5)
            print("New robot pose", sim_interface.localize_robot())
                                
            #Stop robot
            sim_interface.setvel_pioneers(0.0, 0.0)

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #stop robots
    sim_interface.setvel_pioneers(0.0, 0.0)
    sim_interface.sim_shutdown()
    time.sleep(2.0)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 