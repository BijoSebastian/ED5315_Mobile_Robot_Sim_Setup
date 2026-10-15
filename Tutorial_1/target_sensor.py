#Provided - do not modify.
"""
Tracking sensor: reports the target's (x, y) position with Gaussian noise,
but only when our robot has a direct line of sight to it.

Line of sight is checked with a real ray: our robot's lidar beam is pointed
straight at the target and fired once. A reading is returned only if the
first thing the beam hits is the target itself, within SENSE_RANGE. Obstacle
spheres (including the moving one), walls, or simply being too far away all
mean no reading - measure() returns None and your EKF has to carry on with
prediction alone.

The sensor never reports the target's heading, speed or turn rate.
"""

import math

import numpy as np

from ed5315 import sim_interface

SENSE_RANGE = 4.5  # [m] maximum distance at which the target can be seen
NOISE_STD = 0.15   # [m] standard deviation of the noise on each of x and y

_rng = np.random.default_rng()
_parts = {}  # target handle -> set of handles of every object in its model


def measure(target):
    # target: the target's object handle (from target_driver.setup)
    # returns np.array([x, y]) in world coordinates [m], or None if not visible
    sim = sim_interface._sim
    joint = sim_interface._h['lidar_joint']
    laser = sim_interface._h['lidar']

    if target not in _parts:
        _parts[target] = set(sim.getObjectsInTree(target, sim.handle_all, 0))

    # Point the beam at the target's centre, in the lidar's own frame
    rel = sim.getObjectPosition(target, sim.getObjectParent(joint))
    sim.setJointPosition(joint, math.atan2(rel[1], rel[0]))
    res, dist, _, hit, _ = sim.handleProximitySensor(laser)
    if not res or dist > SENSE_RANGE or hit not in _parts[target]:
        return None

    x, y = sim.getObjectPosition(target, -1)[:2]
    return np.array([x + NOISE_STD * _rng.standard_normal(),
                     y + NOISE_STD * _rng.standard_normal()])
