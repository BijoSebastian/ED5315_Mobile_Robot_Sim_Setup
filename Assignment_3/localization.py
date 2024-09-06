#Importing the nescessary libraries 
import numpy as np #importing thr numericla python library
import visualization
from math import sin, cos, pi #importing the necessary library 
import matplotlib
matplotlib.use('Qt5Agg')  # or 'Agg' for non-interactive use
import matplotlib.pyplot as plt
#Global Variables

'''Robot properties'''
width_robo=1
ticks_to_millimeter=0.05
control_motion_factor=1
control_turn_factor=1
odometry_offset=0

#class to keep track of the ticks which arise from the motor encoders
class Odometry_calculation:

    #variables storing the previous left and right tick 
    prev_left_tick=0
    prev_right_tick=0

    #variables storing the current left and right tick
    left_tick=0
    right_tick=0
    L=width_robo
    
    pose = np.array([4.0, 4.0, 0.0]) #Variable to store pose of the robot 
    covariance = np.diag([0.0, 0.0, 0.0]) #Covariance of the robot

    def __init__(self,matrix_calculator):
        self.matrix_calculator=matrix_calculator
    
    def get_system_covariance(self, left_dist_moved, right_dist_moved):

        """Calculates the increase in covariance due to previous state"""

        F_p=self.matrix_calculator.f_p(left_dist_moved, right_dist_moved,self.L,self.pose)

        return F_p

    def get_motion_covariance(self, left_dist_moved, right_dist_moved):

        """Calculates the increase in covariance due to control inputs"""
        
        F_u=self.matrix_calculator.f_u(left_dist_moved, right_dist_moved,self.L,self.pose)


        return F_u

    def get_covariance(self): 

        """Calculates the covariance of the new state"""
    
        #Calculate tick increment 
        tick_difference=[0,0] #intialisng the tick difference array which is the control input
        tick_difference[0] = self.left_tick - self.prev_left_tick #assigning the left tick difference 
        tick_difference[1] = self.right_tick - self.prev_right_tick #assigning the right tick difference 
    
        #extracting the left and right control input 
        left_dist_moved = tick_difference[0] * ticks_to_millimeter
        right_dist_moved = tick_difference[1] * ticks_to_millimeter

        if left_dist_moved!=0 or right_dist_moved !=0:
            #exracting the motion and tuen factors
            alpha_1 = control_motion_factor
            alpha_2 = control_turn_factor

            abs_left_dist_moved = abs(left_dist_moved)
            abs_right_dist_moved = abs(right_dist_moved)

            #Simplyfy this later on.
            #defining the covariances associated with x the left and right componenet of the control
            sigma_l = (alpha_1 * abs_left_dist_moved) + (alpha_2 * abs((abs_left_dist_moved - abs_right_dist_moved))) # covariance propotional to left and to difference in left and right control
            sigma_r = (alpha_1 * abs_right_dist_moved) + (alpha_2 * abs((abs_left_dist_moved- abs_right_dist_moved)))  # covariance propotional to right and to difference in left and right control

            control_covariance = np.diag([sigma_l, sigma_r]) # froming a diagonal matrix using the control covariance 
    
            #getting the covariance involved due to the covariance in the previous state 
            F_p = self.get_system_covariance(left_dist_moved, right_dist_moved)
    
            #getting the covariance involved due to the error in the control input 
            F_u = self.get_motion_covariance(left_dist_moved, right_dist_moved)
    
            #calcualting the final covariance 
            self.covariance = np.dot(F_p, np.dot(self.covariance, F_p.T)) + np.dot(F_u, np.dot(control_covariance, F_u.T))

        return self.covariance

    #function to update the current pose bsed on previous pose and control input 
    def odometry_update(self):

        """Calcualtes the new state based on old state and change in encoder ticks"""
    
        #Calculate tick increment 
        tick_difference=[0,0] #intialisng the tick difference array which is the control input
        tick_difference[0] = self.left_tick - self.prev_left_tick #assigning the left tick difference 
        tick_difference[1] = self.right_tick - self.prev_right_tick #assigning the right tick difference 
    
        self.prev_left_tick = self.left_tick #updating the left tick variable in the class
        self.prev_right_tick = self.right_tick #updating the right tick variable in the class

        #variables to store new pose and change in orientation
        x = 0.0
        y = 0.0
        theta = 0.0
        delta_theta=0.0

        #Calculate robot motion based on tick increment 
        #first case robot travels in straight line 
        if tick_difference[0]==tick_difference[1]:

            theta = self.pose[2] #Get previous orientation  
            x = self.pose[0]+tick_difference[0]*ticks_to_millimeter*cos(theta) #updating x 
            y = self.pose[1]+tick_difference[1]*ticks_to_millimeter*sin(theta) #updating y 

        #second case in case of a curve
        else:
            theta = self.pose[2] #Get previous orientation 
            x = self.pose[0]-odometry_offset*sin(theta)
            y = self.pose[1]-odometry_offset*cos(theta)
            

            #change in oreintation and radius of curvature of the turn
            delta_theta = ticks_to_millimeter*(tick_difference[1]-tick_difference[0])/(width_robo)# change in orientation
            R = ticks_to_millimeter*tick_difference[0]/delta_theta #radius of curvature

            #calulating the center of curvature 
            centerx = x-(R+width_robo/2)*sin(theta) # x coordinate 
            centery = y+(R+width_robo/2)*cos(theta) # y coordinate
        
            #updating the x and using newly calualted theta value 
            theta=theta+delta_theta
            x = centerx + ((R+width_robo/2)*sin(theta)) + (odometry_offset*sin(theta)) # caluating the new x
            y = centery - ((R+width_robo/2)*cos(theta)) + (odometry_offset*cos(theta)) # cacualting the new y

        #Update x,y position
        self.pose[0] = x
        self.pose[1] = y
        self.pose[2] = theta

        return self.pose
    
    #call back function to update ticks    
    def update_encoder_tick(self, tick_list):

        """Updates the encoder data"""
        left_tick,right_tick=tick_list
        self.left_tick = left_tick #updating the left_tick
        self.right_tick = right_tick#updating the right tick

        return

def update_plot(odom_cal,realpose):
    #plt.clf()  # Clear the current figure
    # Update the odometry and covariance
    odom_cal.get_covariance()
    odom_cal.odometry_update()
    # Update the plot with the new pose and covariance
    visualization.plot_updation(odom_cal.pose, odom_cal.covariance,realpose)
    
    # Redraw the plot
    plt.draw()
    plt.pause(0.01)  # Pause to allow for plot update

def encoder_output(v,w,delt):
    l=width_robo
    vr=((2*v)+(w*l))/2
    vl=((2*v)-(w*l))/2
    
    dr=vr*delt
    dl=vl*delt

    return dl,dr