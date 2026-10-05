# ED5315 Assignment-4
EKF-based localization for a differential-drive mobile robot (Pioneer p3dx) in CoppeliaSim -
the robot is given a known map of every obstacle's true position and must localize itself
against it, fusing wheel odometry with landmark observations.

## Setup
See the [repository root README](../README.md) for installation and setup.

## What's different from Assignment 3

Wheel odometry alone (Assignment 3) drifts unboundedly - nothing ever corrects it. This
assignment adds that correction. `main.py` now also queries
`sim_interface.get_ground_truth_map()` once at the start: a `{tag_id: (x, y)}` map giving
every obstacle's *true* position. Every loop iteration, instead of calling
`odometry.estimate_pose(robot_state, Vl, Vr)` directly, it calls your
**localization.estimate_pose(robot_state, Vl, Vr, tracked_obstacles, ground_truth_map)** -
an Extended Kalman Filter that:
- **predicts** a new pose the same way Assignment 3 did (in fact, your EKF should call your
  own `odometry.estimate_pose` directly for this step - see below), then
- **corrects** that prediction using any obstacles detected this step, by comparing where
  you expected to see them (from your predicted pose and their known position in
  `ground_truth_map`) against where perception.py actually detected them. **Please note that
  obstacle ID 5 is dynamic and should not be used in localisation unless you can account for its motion**.

This is **EKF localization**, not EKF SLAM: the map (`ground_truth_map`) is given and fixed
throughout - your filter only ever estimates the robot's own pose, never the map.

Everything else is unchanged from Assignment 1-3: the goal is still ground truth, obstacles
are still detected via `perception.py`, and `control.py`'s `gtg`/`avoid_obstacles`/
`navigation_state_machine` still just take `robot_state` as a plain `[x, y, theta]`.

## Task: EKF localization

Complete **estimate_pose** in **localization.py** - this is the file you submit (see
Submission below). Read its docstring carefully; it explains:
- how to recover the raw (range, bearing) measurement perception.py's detections were built
  from (needed for the EKF's update step),
- how to reuse `odometry.py`'s `estimate_pose` for your prediction step's mean, and
- what to propagate for your prediction step's covariance and your update step's
  correction.

`covariance` is your own filter state - maintain it as module-level state in
`localization.py` (the same pattern `odometry.py`'s `previous_time` and `control.py`'s
`previous_time` already use). `main.py` never sees or passes it in.

## control.py / perception.py / odometry.py

All three carried over unchanged from Assignment 1/2/3 - same function names, signatures,
and `dt`-handling. None of them are graded here; see Submission below. Note that
`odometry.py` is more than just a local-testing convenience this time: your
`localization.py` is expected to call its `estimate_pose` directly as the EKF's prediction
step, so a working `odometry.py` is required for `localization.py` to run at all. Do not
make any changes to `main.py`.

## Submission

**localization.py** is the only file you submit, and the only one that's graded -
`control.py`/`perception.py`/`odometry.py` in this folder are *not* evaluated, they're only
here so you can run and test `main.py` locally end-to-end. Paste your own working
**control.py** from Assignment 1/2/3, **perception.py** from Assignment 2/3, and
**odometry.py** from Assignment 3 into this folder - all three drop in completely unchanged.

## Instructions

  1. Download the setup provided in this repository (or `git pull` if you already have it).

  2. Copy your working **control.py**, **perception.py**, and **odometry.py** from
     Assignment 1/2/3 over this folder's copies.

  3. Complete **estimate_pose** in **localization.py** - this is the file you'll submit.

  4. Launch CoppeliaSim. Click File -> Open Scene, and open the shared
     [`scenes/mobile_robot.ttt`](../scenes/mobile_robot.ttt). Run the simulation with the play button.

  5. Run `main.py` from this folder (or `python Assignment_4/main.py` from the repository
     root).

  6. Always ensure the simulation is running before you launch the code, otherwise you'll
     get "Failed connecting to remote API server."

  7. If your implementation is correct, the robot will find its way to the goal while
     avoiding obstacles, and its EKF-estimated path should track the ground-truth path much
     more closely than Assignment 3's raw odometry did - visibly correcting itself whenever
     an obstacle comes into view.
