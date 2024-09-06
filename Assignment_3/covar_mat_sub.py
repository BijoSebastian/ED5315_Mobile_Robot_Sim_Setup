import numpy as np

class matrix_calculator:

    def f_p(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled by right and left wheel
        delta_d = (left_dist_moved + right_dist_moved)/2
        delta_theta = (right_dist_moved - left_dist_moved)/L

        #calcualting the jacobian of the states with respect to the previous state
        F_p = np.array(([1,0,-abs(delta_d*np.sin(pose[2]+delta_theta/(2)))],[0,1,abs(delta_d*np.cos(pose[2]+delta_theta/(2)))],[0,0,1]))

        #########################################################

        return F_p
    
    def f_u(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled y right and left wheel
        delta_d = (left_dist_moved + right_dist_moved)/2
        delta_theta = (right_dist_moved - left_dist_moved)/L
        
        #calcualting the components of the jacobain of the state with respect to the control inputs 
        f11= (1/2)*(np.cos(pose[2]+delta_theta/2)) - (delta_d/(2*L))*np.sin(pose[2]+delta_theta/2)
        f12= (1/2)*(np.cos(pose[2]+delta_theta/2)) + (delta_d/(2*L))*np.sin(pose[2]+delta_theta/2)

        f21 = (1/2)*(np.sin(pose[2]+delta_theta/2)) + (delta_d/(2*L))*np.cos(pose[2]+delta_theta/2)
        f22 = (1/2)*(np.sin(pose[2]+delta_theta/2)) - (delta_d/(2*L))*np.cos(pose[2]+delta_theta/2)

        f31 = (1/L)
        f32 = -(1/L)

        F_u = np.array(([f11,f12],[f21,f22],[f31,f32]))

        #########################################################

        return F_u
