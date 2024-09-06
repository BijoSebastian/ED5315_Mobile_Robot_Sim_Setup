import numpy as np #importing thr numericla python library
from math import sin, cos, pi #importing the necessary library 
import sim_interface
import localization
import covar_mat_sub
import visualization
import time
import signal
import sys  # Import the sys module for sys.exit()
import matplotlib
matplotlib.use('Qt5Agg')  # or 'Agg' for non-interactive use
import matplotlib.pyplot as plt
#Global Variables

def signal_handler(sig,frame):
    print('Shutting down...')
    plt.close('all')
    sys.exit(0)

signal.signal(signal.SIGINT,signal_handler)

def main():

    global odom_cal

    visualization.plot_initialization()

    #Class object for odometry calculation
    mat_cal=covar_mat_sub.matrix_calculator()
    odom_cal = localization.Odometry_calculation(mat_cal)

    k=0
    r=0
    v,w=1,0.2


    
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):
            
            #Stop robot
            sim_interface.setvel_pioneers1(v,w)
            #sim_interface.setvel_pioneers2(v,w)
            start=time.time()

            #time.sleep(9)

            #Obtain robots position
            realpose = sim_interface.localize_robot1()
            #robot_state2 = sim_interface.localize_robot2()

            try:
                while(True):
                    # Update encoder ticks (simulating the process)
                    # You can replace the values with your own logic or input
                    now=time.time()
                    delt=now-start
                    dl,dr=localization.encoder_output(v,w,delt)
                    tl,tr=(dl/localization.ticks_to_millimeter),(dr/localization.ticks_to_millimeter)
                    odom_cal.update_encoder_tick([tl,tr])

                    realpose = sim_interface.localize_robot1()
                    
                    # Update the plot with the new data
                    localization.update_plot(odom_cal,realpose)
                    x,y,theta=odom_cal.pose
                    sim_interface.change_pioneer2_pose(x,y,theta)
                    # Sleep for a short duration to simulate real-time updates
                    time.sleep(0.1)  # Adjust this to control the update frequency
                    k-=10

            except KeyboardInterrupt:
                pass

            # Show the plot at the end (block=True to keep it open)
            plt.show(block=True)
            
            
            
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