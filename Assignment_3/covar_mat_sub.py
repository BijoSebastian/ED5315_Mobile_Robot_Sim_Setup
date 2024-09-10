import numpy as np

class matrix_calculator:

    def f_p(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled by right and left wheel
        delta_d = (left_dist_moved + right_dist_moved)/2
        delta_theta = (right_dist_moved - left_dist_moved)/L

        #calcualting the jacobian of the states with respect to the previous state
        F_p = ??

        #########################################################

        return F_p
    
    def f_u(self, left_dist_moved, right_dist_moved,L,pose):

        ###################Edit this part########################

        #defining deltad as the average distance travelled y right and left wheel
        delta_d = (left_dist_moved + right_dist_moved)/2
        delta_theta = (right_dist_moved - left_dist_moved)/L
        
        #calcualting the jacobain of the states with respect to the control inputs 
        

        F_u = ??

        #########################################################

        return F_u
