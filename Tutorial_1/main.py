# -*- coding: utf-8 -*-
"""
Created on Fri Sep 20 22:07:58 2024

@author: Bijo Sebastian
"""

import numpy as np
import csv
import matplotlib.pyplot as plt
plt.close("all")

import EKF_class

#Read measrument data
with open('measurement_data.csv', 'r') as f:
    reader = csv.reader(f)
    data = list(reader)
    measurement_data = np.array(data, dtype=float)

#Create and initialise tracker instance
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) #Initial state
covariance =  np.diag([0.2, 0.2, 0.01, 0.01, 0.1]) #Initial covariance
tracking_instance = EKF_class.EKF(state, covariance)

#Setup data recording for plotting
est_x, est_y, est_theta, est_delta = [], [], [], []
est_x.append(state[0])
est_y.append(state[1])
est_theta.append(state[2])
est_delta.append(state[3])

#Run tracker over measurement data
print("Time", measurement_data[0,3] )
for i in range(1,len(measurement_data)):
     print("Time", measurement_data[i, 3] )
    
     delta_t = measurement_data[i, 3] - measurement_data[i-1, 3] #in seconds
     
     #Prediction step
     tracking_instance.state_prediction(delta_t)
     tracking_instance.covariance_prediction(delta_t)
     
     #Correction step
     tracking_instance.correction(measurement_data[i, 0:3])
     
     #Data record
     est_x.append(tracking_instance.state[0])
     est_y.append(tracking_instance.state[1])
     est_theta.append(tracking_instance.state[2])
     est_delta.append(tracking_instance.state[3])

#Create plots
plt.figure()
plt.plot(est_x, est_y, label='EKF Estimated Trajectory', color='b')
plt.scatter(measurement_data[:, 0], measurement_data[:, 1], label='Measured Trajectory', color='r', marker='*')
plt.xlabel('X (metres)')
plt.ylabel('Y (metres)')
plt.title('Estimated Trajectory vs Measured Trajectory')
plt.legend()
plt.grid(True)
plt.show()
 
plt.figure()
plt.plot(measurement_data[:, 3], est_theta, label='Estimated Theta', color='b')
plt.scatter(measurement_data[:, 3], est_theta, label='Measured Theta', color='r', marker='*')
plt.xlabel('Time (s)')
plt.ylabel('Theta (radians)')
plt.title('Estimated vs Measured Theta')
plt.legend()
plt.grid(True)
plt.show()
plt.figure()
 
plt.plot(measurement_data[:, 3], est_delta, label='Estimated Delta (Steering Angle)', color='g')
plt.xlabel('Time (s)')
plt.ylabel('Delta (radians)')
plt.title('Estimated Steering Angle (Delta) over Time')
plt.legend()
plt.grid(True)
plt.show()
