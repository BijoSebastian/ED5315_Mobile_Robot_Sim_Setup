#Provided - do not modify.
"""
Spawns and drives the target: a second Pioneer that wanders around the
parked observer (our own /Pioneer1) on a smooth random trajectory.

The target's forward speed and turn rate each drift randomly over several
seconds, then pass through a first-order lag and acceleration limits, so the
motion is smooth but never repeats (a fresh random seed every run unless
main.py fixes one). Three soft steering biases are blended in on top:
  - turn back inward when nearing the edge of the play area,
  - turn away when getting too close to the observer,
  - steer around obstacle spheres ahead of it (they are non-respondable, so
    without this it would drive straight through them).

Your EKF knows none of this - it only ever sees noisy (x, y) positions from
target_sensor.py.

setup() must run while the simulation is STOPPED (it stops it if needed):
it parks the observer and copies it to make the target. cleanup(), after the
simulation stops, removes the target and puts the observer back where it
was - the shared scene file never changes. (Copying the model while the
simulation is running crashed CoppeliaSim's renderer in testing.)
"""

import math

import numpy as np

from ed5315 import sim_interface, robot_params

OBSERVER_POSE = (12.0, 10.0, 0.0)  # [x, y, theta] where our robot is parked
TARGET_START = (14.5, 7.5, 0.0)    # [x, y, theta] where the target appears

PLAY_RADIUS = 5.5   # [m] target is steered back inside this distance from the observer
KEEPOUT = 1.5       # [m] ...and kept at least this far from it
RAMP = 1.5          # [m] width over which each of those two biases fades in
OFFSET_HOLD = 5.0   # [s] a bias must be idle this long before it picks a new random aim offset
AVOID_START = 2.5   # [m] start steering around an obstacle ahead at this centre distance
AVOID_FULL = 1.2    # [m] fully committed to going around it at this distance

V_MEAN, V_STD, V_MIN, V_MAX = 0.35, 0.10, 0.15, 0.50  # [m/s] random forward speed
W_STD, W_MAX = 0.12, 0.25                             # [rad/s] random turn rate
W_BIAS_MAX = 0.30          # [rad/s] turn rate allowed for the steering biases
TAU_V, TAU_W = 6.0, 4.0    # [s] how slowly the random speed / turn rate drift
TAU_LAG = 1.5              # [s] extra smoothing on top
ACC_V, ACC_W = 0.15, 0.30  # [m/s^2], [rad/s^2] acceleration limits
K_HEAD = 0.6               # steering gain for the biases

_sim = None
_dt = 0.05
_target = None
_observer_home = None  # observer's pose before setup(), restored by cleanup()
_left = None
_right = None
_rng = None
_v_ref = _w_ref = _v_lag = _w_lag = _v = _w = 0.0
_bias_offset = {'out': 0.0, 'in': 0.0}
_bias_idle = {'out': OFFSET_HOLD, 'in': OFFSET_HOLD}  # [s] since each bias was last active
_pass_side = {}  # obstacle index -> side we committed to passing it on


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _clip(val, lo, hi):
    return max(lo, min(hi, val))


def _closer_ramp(d, start, full):
    # 0 at distance >= start, rising linearly to 1 at distance <= full
    return _clip((start - d) / (start - full), 0.0, 1.0)


def _pose(handle):
    p = _sim.getObjectPosition(handle, -1)
    o = _sim.getObjectOrientation(handle, -1)
    return p[0], p[1], o[2]


def _steer(heading, theta):
    return _clip(K_HEAD * _wrap(heading - theta), -W_BIAS_MAX, W_BIAS_MAX)


def _wait_until_stopped():
    if _sim.getSimulationState() != _sim.simulation_stopped:
        _sim.stopSimulation()
        while _sim.getSimulationState() != _sim.simulation_stopped:
            pass


def setup(seed=None):
    # Call once, after sim_interface.get_handles() and BEFORE
    # start_simulation(). Parks our robot at OBSERVER_POSE and spawns the
    # target. Returns the target's object handle (target_sensor.measure
    # needs it).
    global _sim, _dt, _target, _left, _right, _rng, _observer_home
    global _v_ref, _w_ref, _v_lag, _w_lag, _v, _w
    _sim = sim_interface._sim
    _dt = _sim.getSimulationTimeStep()
    _rng = np.random.default_rng(seed)
    _wait_until_stopped()

    # Leftover from an interrupted run?
    try:
        _sim.removeModel(_sim.getObject('/Target'))
    except Exception:
        pass

    me = sim_interface._h['pioneer']
    _observer_home = (_sim.getObjectPosition(me, -1), _sim.getObjectOrientation(me, -1))
    z = _observer_home[0][2]
    _sim.setObjectPosition(me, -1, [OBSERVER_POSE[0], OBSERVER_POSE[1], z])
    _sim.setObjectOrientation(me, -1, [0.0, 0.0, OBSERVER_POSE[2]])

    _target = _sim.copyPasteObjects([me], 1)[0]
    _sim.setObjectAlias(_target, 'Target')
    _sim.setObjectPosition(_target, -1, [TARGET_START[0], TARGET_START[1], z])
    _sim.setObjectOrientation(_target, -1, [0.0, 0.0, TARGET_START[2]])
    _left = _sim.getObject('/Target/Pioneer1_left')
    _right = _sim.getObject('/Target/Pioneer1_right')

    _v_ref = _v_lag = V_MEAN
    _w_ref = _w_lag = 0.0
    _v = _w = 0.0
    _pass_side.clear()
    return _target


def drive():
    # Call once per simulation step, before sim_interface.step().
    global _v_ref, _w_ref, _v_lag, _w_lag, _v, _w
    dt = _dt
    x, y, th = _pose(_target)

    # Slowly drifting random speed / turn rate (mean-reverting random walks)
    _v_ref += (V_MEAN - _v_ref) * dt / TAU_V + V_STD * math.sqrt(2 * dt / TAU_V) * _rng.standard_normal()
    _w_ref += (0.0 - _w_ref) * dt / TAU_W + W_STD * math.sqrt(2 * dt / TAU_W) * _rng.standard_normal()
    _v_ref = _clip(_v_ref, V_MIN, V_MAX)
    _w_ref = _clip(_w_ref, -W_MAX, W_MAX)
    _v_lag += (_v_ref - _v_lag) * dt / TAU_LAG
    _w_lag += (_w_ref - _w_lag) * dt / TAU_LAG
    v_des, w_des = _v_lag, _w_lag

    ox, oy, _ = OBSERVER_POSE
    dx, dy = x - ox, y - oy
    r = math.hypot(dx, dy)

    # Soft biases, blended in order of increasing priority
    weights = {
        'out': _clip((r - (PLAY_RADIUS - RAMP)) / RAMP, 0.0, 1.0),  # nearing edge of play area
        'in': _closer_ramp(r, KEEPOUT + RAMP, KEEPOUT),              # too close to the observer
    }
    headings = {'out': math.atan2(-dy, -dx), 'in': math.atan2(dy, dx)}
    for key in ('out', 'in'):
        wt = weights[key]
        if wt > 0:
            if _bias_idle[key] >= OFFSET_HOLD:
                _bias_offset[key] = _rng.uniform(-0.7, 0.7)  # don't always aim straight at / away from the observer
            _bias_idle[key] = 0.0
            w_des = (1 - wt) * w_des + wt * _steer(headings[key] + _bias_offset[key], th)
        else:
            _bias_idle[key] += dt

    for i, (bx, by) in enumerate(sim_interface.get_obstacle_positions()):
        d = math.hypot(bx - x, by - y)
        bearing = math.atan2(by - y, bx - x)
        rel = _wrap(bearing - th)
        # fades out smoothly as the obstacle moves from dead ahead to beside us
        wt = _closer_ramp(d, AVOID_START, AVOID_FULL) * max(0.0, math.cos(rel))
        if wt == 0:
            _pass_side.pop(i, None)
            continue
        side = _pass_side.setdefault(i, 1 if rel > 0 else -1)  # obstacle on the left -> pass it on the right
        w_des = (1 - wt) * w_des + wt * _steer(bearing - side * (math.pi / 2 + 0.3), th)
        v_des *= 1 - 0.4 * wt

    # Acceleration limits
    _v += _clip(v_des - _v, -ACC_V * dt, ACC_V * dt)
    _w += _clip(w_des - _w, -ACC_W * dt, ACC_W * dt)

    L, R = robot_params.track_width, robot_params.wheel_radius
    _sim.setJointTargetVelocity(_left, (_v - _w * L / 2) / R)
    _sim.setJointTargetVelocity(_right, (_v + _w * L / 2) / R)


def true_state():
    # Ground truth [x, y, theta, v, omega] - for plotting only. Your EKF must
    # never use this.
    x, y, th = _pose(_target)
    lin, ang = _sim.getObjectVelocity(_target)
    v = lin[0] * math.cos(th) + lin[1] * math.sin(th)
    return [x, y, th, v, ang[2]]


def cleanup():
    # Stops the simulation if it is still running, removes the target and
    # puts our robot back where it was. Safe to call more than once, or if
    # setup() failed part-way.
    global _target, _observer_home
    if _sim is None:
        return
    _wait_until_stopped()
    try:
        _sim.removeModel(_sim.getObject('/Target'))
    except Exception:
        pass
    _target = None
    if _observer_home is not None:
        me = sim_interface._h['pioneer']
        _sim.setObjectPosition(me, -1, _observer_home[0])
        _sim.setObjectOrientation(me, -1, _observer_home[1])
        _observer_home = None
