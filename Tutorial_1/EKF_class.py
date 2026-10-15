#This IS what you submit for Tutorial 1 - the only file that's graded. See
#README.md for the motion model, the measurement model and the Submission
#section.
"""
@author: Bijo Sebastian
"""

import numpy as np

class EKF:
    def __init__(self, state, covariance):
        #state: [x, y, vx, vy, omega] - the target's position [m], velocity
        #    [m/s] and turn rate [rad/s], as a numpy array of length 5
        #covariance: 5x5 numpy array
        self.state = np.array(state, dtype=float)
        self.covariance = np.array(covariance, dtype=float)
        self.Q = np.diag([0.15 ** 2, 0.15 ** 2, 0.30 ** 2]) #Variance of the noise inputs [a_x, a_y, alpha]
        self.R = np.diag([0.15 ** 2, 0.15 ** 2]) #Measurement covariance
        self.H = None #TODO: 2x5 measurement Jacobian

    def state_prediction(self, delta_t):
        #State prediction: move self.state forward by delta_t seconds using
        #the motion model with the noise inputs set to zero
        raise NotImplementedError

    def compute_Fp(self, delta_t):
        #Compute Fp (5x5): Jacobian of the motion model with respect to the
        #state, evaluated at the current self.state
        raise NotImplementedError

    def compute_Fdelta(self, delta_t):
        #Compute Fdelta (5x3): Jacobian of the motion model with respect to
        #the noise inputs [a_x, a_y, alpha], evaluated at the current self.state
        raise NotImplementedError

    def covariance_prediction(self, delta_t):
        #Covariance propagation: update self.covariance using Fp, Fdelta and
        #self.Q. main.py calls this BEFORE state_prediction, so self.state is
        #still the state from before this step
        raise NotImplementedError

    def correction(self, measurement):
        #Correction step: measurement is a numpy array [x, y] from the
        #sensor. Update self.state and self.covariance. main.py only calls
        #this when the target is visible
        raise NotImplementedError
