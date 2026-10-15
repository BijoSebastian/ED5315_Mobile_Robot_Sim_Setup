#!/usr/bin/env python

"""
ED5315 Tutorial 1 - track a randomly moving robot with an EKF
@author: Bijo Sebastian
"""

#Import files
import math
import os

import numpy as np
import matplotlib
matplotlib.use('Agg')  # headless - the result is saved as a PNG
import matplotlib.pyplot as plt

from ed5315 import sim_interface, plotting, robot_params
import target_driver
import target_sensor
import EKF_class

DURATION = 120.0    # [s] of simulation time to track the target for
SENSOR_EVERY = 2    # the sensor is read every 2nd simulation step (10 Hz)
SEED = None         # set to an integer to repeat the same target trajectory

RESULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'result.png')


def run(target):
    #Wait for the first sighting, and start the tracker from it: position
    #from the measurement, velocity and turn rate unknown
    first = None
    t_start = sim_interface.sim_time()
    while first is None:
        if sim_interface.sim_time() - t_start > 30.0:
            raise RuntimeError('The target never came into view - try reloading the scene')
        target_driver.drive()
        sim_interface.step()
        first = target_sensor.measure(target)

    state = np.array([first[0], first[1], 0.0, 0.0, 0.0])  #Initial state [x, y, vx, vy, omega]
    covariance = np.diag([target_sensor.NOISE_STD ** 2, target_sensor.NOISE_STD ** 2,
                          0.3 ** 2, 0.3 ** 2, 0.3 ** 2])  #Initial covariance
    tracking_instance = EKF_class.EKF(state, covariance)

    #Setup data recording for plotting
    log = {'t': [], 'true': [], 'est': [], 'cov': [], 'visible': [], 'meas': [], 'obstacles': None}
    t0 = sim_interface.sim_time()
    prev_time = t0
    visible = True
    k = 0

    #Run tracker
    while sim_interface.sim_time() - t0 < DURATION:

        #The target moves (you don't know how)
        target_driver.drive()
        sim_interface.step()

        now = sim_interface.sim_time()
        delta_t = now - prev_time
        prev_time = now

        #Prediction step - covariance first, so that its Jacobians are
        #evaluated at the state from BEFORE this step's prediction
        tracking_instance.covariance_prediction(delta_t)
        tracking_instance.state_prediction(delta_t)

        #Correction step - only when the sensor can see the target
        k += 1
        if k % SENSOR_EVERY == 0:
            measurement = target_sensor.measure(target)
            visible = measurement is not None
            if visible:
                tracking_instance.correction(measurement)
                log['meas'].append(measurement)

        #Data record
        log['t'].append(now - t0)
        log['true'].append(target_driver.true_state())
        log['est'].append(np.array(tracking_instance.state, dtype=float))
        log['cov'].append(np.array(tracking_instance.covariance, dtype=float))
        log['visible'].append(visible)

    log['obstacles'] = sim_interface.get_obstacle_positions()
    return log


def plot(log):
    t = np.array(log['t'])
    true = np.array(log['true'])
    est = np.array(log['est'])
    cov = np.array(log['cov'])
    visible = np.array(log['visible'], dtype=bool)
    meas = np.array(log['meas']).reshape(-1, 2)

    #true is [x, y, theta, v, omega]; est is [x, y, vx, vy, omega] - heading
    #and speed follow from the estimated velocity vector
    est_theta = np.arctan2(est[:, 3], est[:, 2])
    est_speed = np.hypot(est[:, 2], est[:, 3])
    pos_err = np.hypot(est[:, 0] - true[:, 0], est[:, 1] - true[:, 1])
    pos_sigma = np.sqrt(cov[:, 0, 0] + cov[:, 1, 1])
    theta_err = np.arctan2(np.sin(est_theta - true[:, 2]), np.cos(est_theta - true[:, 2]))

    def rms(x):
        return np.sqrt(np.mean(x ** 2)) if len(x) else float('nan')

    print("Tracked for %.0f s - target visible %.0f%% of the time" % (t[-1], 100.0 * visible.mean()))
    print("Position error: %.3f m RMS while visible, %.3f m RMS while not visible (sensor noise %.2f m per axis)"
          % (rms(pos_err[visible]), rms(pos_err[~visible]), target_sensor.NOISE_STD))
    print("Heading error %.1f deg RMS, speed error %.3f m/s RMS, turn-rate error %.3f rad/s RMS"
          % (math.degrees(rms(theta_err)), rms(est_speed - true[:, 3]), rms(est[:, 4] - true[:, 4])))

    fig, axs = plt.subplots(2, 2, figsize=(14, 12))

    def shade_not_visible(ax):
        ax.fill_between(t, 0, 1, where=~visible, transform=ax.get_xaxis_transform(),
                        color='orange', alpha=0.15, label='Target not visible')

    #Map: obstacles, sensor range, measurements, true and estimated path
    ax = axs[0, 0]
    ox, oy, _ = target_driver.OBSERVER_POSE
    plotting.draw_obstacles(ax, log['obstacles'], radius=robot_params.obstacle_radius,
                            label='Obstacles (at end of run)')
    ax.add_patch(plt.Circle((ox, oy), target_sensor.SENSE_RANGE, fill=False, linestyle='--',
                            color='gray', label='Sensor range'))
    ax.plot(ox, oy, 's', color='black', markersize=10, label='Our robot (parked)')
    ax.scatter(meas[:, 0], meas[:, 1], s=6, color='red', alpha=0.5, label='Measurements')
    plotting.draw_path(ax, true[:, :2].tolist(), color='blue', label='True path')
    plotting.draw_path(ax, est[:, :2].tolist(), color='purple', linestyle='--', label='EKF estimate')
    r = target_driver.PLAY_RADIUS + 1.0
    ax.set_xlim(ox - r, ox + r)
    ax.set_ylim(oy - r, oy + r)
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.legend(loc='upper right', fontsize=8)
    ax.set_title('Estimated vs true trajectory')

    #Heading - plotted on the true heading's unwrapped branch so 2*pi jumps don't show
    ax = axs[0, 1]
    true_theta = np.unwrap(true[:, 2])
    ax.plot(t, true_theta, color='blue', label='True heading')
    ax.plot(t, true_theta + theta_err, color='purple', linestyle='--', label='Estimated heading')
    shade_not_visible(ax)
    ax.set_xlabel('t [s]')
    ax.set_ylabel('theta [rad]')
    ax.legend(fontsize=8)
    ax.set_title('Heading = atan2(vy, vx) (never measured)')

    ax = axs[1, 0]
    ax.plot(t, true[:, 3], color='blue', label='True speed v [m/s]')
    ax.plot(t, est_speed, color='purple', linestyle='--', label='Estimated v = hypot(vx, vy)')
    ax.plot(t, true[:, 4], color='green', label='True turn rate omega [rad/s]')
    ax.plot(t, est[:, 4], color='olive', linestyle='--', label='Estimated omega')
    shade_not_visible(ax)
    ax.set_xlabel('t [s]')
    ax.legend(fontsize=8)
    ax.set_title('Speed and turn rate (never measured)')

    ax = axs[1, 1]
    ax.plot(t, pos_err, color='purple', label='Position error')
    ax.plot(t, 2 * pos_sigma, color='gray', linestyle='--', label='2-sigma (from your covariance)')
    shade_not_visible(ax)
    ax.set_xlabel('t [s]')
    ax.set_ylabel('[m]')
    ax.legend(fontsize=8)
    ax.set_title('Position error and EKF uncertainty')

    fig.tight_layout()
    fig.savefig(RESULT_PATH, dpi=110)
    plt.close(fig)
    print('Saved result plot to', RESULT_PATH)


def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        log = None
        try:
            #Park our robot and add the target - needs the simulation stopped
            target = target_driver.setup(SEED)

            #Start simulation
            if (sim_interface.start_simulation()):

                #Our robot stays parked throughout
                sim_interface.setvel_pioneers(0.0, 0.0)
                log = run(target)

            else:
                print('Failed to start simulation')
        finally:
            #Always stop, remove the target and put our robot back - even if
            #the run was interrupted
            sim_interface.sim_shutdown()
            target_driver.cleanup()

        if log is not None:
            plot(log)
    else:
        print('Failed connecting to remote API server')

    return


#run
if __name__ == '__main__':

    main()
    print('Program ended')
