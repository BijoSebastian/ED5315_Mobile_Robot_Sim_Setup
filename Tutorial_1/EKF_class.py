# -*- coding: utf-8 -*-
"""
Created on Fri Sep 20 21:51:13 2024

@author: Bijo Sebastian
"""

import numpy as np

class EKF:
    def __init__(self, state, covariance):
        self.state = state 
        self.covariance = covariance
        self.L = 2.5 #wheelbase in m
        self.U = 5 #vehicle speed in m/s 
        self.Q = np.array([[0.1]]) #variance in control
        self.R = np.diag([0.0625, 0.0625, 0.007621]) #Measurement covariance
        self.H = ??
        
    def state_prediction(self, delta_t):
        #State prediction
        ??
        return 

    def compute_Fp(self, delta_t):
        #Compute Fp based on state, control input, and fixed params
        ??
        return Fp
        
    def compute_Fu(self, delta_t):
        #Compute Fu based on state, control input, and fixed params
        ??                    
        return Fu

    def covariance_prediction(self, delta_t):
        #Covariance propogate function
        ??            
        return
    
    def correction(self, measurement):
        #Correction step
        ??
        return
