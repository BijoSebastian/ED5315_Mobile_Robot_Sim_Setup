# ED5315 Assignment-2
Obstacle detection and tracking for a differential-drive mobile robot (Pioneer p3dx) in
CoppeliaSim, using the onboard camera and lidar, with velocity tracking to tell moving
obstacles apart from stationary ones.

## Setup
See the [repository root README](../README.md) for installation and setup.

## What's different from Assignment 1

The robot still knows its own pose and the goal pose from simulation - both are ground
truth, exactly like Assignment 1. The only thing that's changed is obstacles: instead of
being handed their positions directly, you must detect them yourself each control-loop
iteration, from:
- `image`: an RGB array from the onboard camera (`ed5315.sensors.read_camera_image`) -
  already upscaled for reliable ArUco decoding. CoppeliaSim's standard perspective vision sensor assumes an ideal pinhole camera with zero distortion.
  `robot_params.vision_fov` / `robot_params.vision_resolution` describe its field of view
  and native resolution. `robot_params.aruco_tag_size` describes the Aruco tag size. The obstacles in the scene uses ArUco tag family: cv2.aruco.DICT_4X4_50, with ids 0, 1, 2, 3, 4, 5.
  Each obstacle has six tags on it but all with the same id, to allow identification from any angle.
- `lidar_scan`: a full-circle range scan (`ed5315.sensors.read_lidar`).

Some obstacle in the scene moves; some are stationary. Use `robot_state` (still ground truth) to convert a detection
from the robot's own frame into world coordinates, so a moving obstacle's own motion
isn't confounded with the robot's ego-motion.

## Task: Obstacle detection and tracking

Complete **detect_obstacles** in **perception.py** - this is the file you submit. Each obstacle carries a unique ArUco tag (`robot_params.aruco_tag_size`
gives its physical size) - use it to find the obstacle and to read its id.

`tracked_obstacles` is a list of the
`TrackedObstacle` dataclass (`ed5315.sensors.TrackedObstacle`, already imported at the top
of `perception.py` - not defined there, so your submission can't change its shape), with
fields `id`, `world_x`, `world_y`, `velocity_x`, `velocity_y`, `detected_time`. `main.py` passes
back exactly what you returned last call (starting from an empty list on the first call).
When an id is seen again, update its existing entry (use `ed5315.sim_interface.sim_time()`
and the entry's previous `detected_time` to work out `velocity_x`/`velocity_y`); the first
time an id is seen, append a new entry for it with velocity `0`.

Return the updated `tracked_obstacles`. `main.py` hands this list straight to `control.py` for navigation, so if you want
avoidance to ignore obstacles that haven't been seen recently, that's a decision for
`control.py` (e.g. using an entry's `detected_time`).

## control.py

Mostly identical to Assignment 1's `control.py`: `at_goal`, `gtg`, and
`differential_drive_ik` are unchanged (`goal_state` is ground truth, same as Assignment 1,
`dt` sourced internally via `ed5315.sim_interface.sim_time()`, never passed in).
`avoid_obstacles` and `navigation_state_machine` differ, since they now take the full
`tracked_obstacles` list (`ed5315.sensors.TrackedObstacle` instances - use `.world_x`/
`.world_y`/`.velocity_x`/`.velocity_y`, not `[x, y]` indexing).
Do not make any changes to `main.py`.

## Submission

**perception.py** is the only file you submit, and the only one that's graded -
`control.py` in this folder is *not* evaluated, it's only here so you can run and test
`main.py` locally end-to-end. Paste your own working **control.py** from Assignment 1 into
it: `at_goal`/`gtg`/`differential_drive_ik` drop in unchanged, but you'll need to adapt
`avoid_obstacles`/`navigation_state_machine` to work with `tracked_obstacles` instead of
`nearby_obstacles`.

## Instructions

  1. Download the setup provided in this repository (or `git pull` if you already have it).

  2. Copy your working **control.py** from Assignment 1 over this folder's **control.py**.

  3. Complete **detect_obstacles** in **perception.py** - this is the file you'll submit.

  4. Launch CoppeliaSim. Click File -> Open Scene, and open the shared
     [`scenes/mobile_robot.ttt`](../scenes/mobile_robot.ttt). Run the simulation with the play button.

  5. Run `main.py` from this folder (or `python Assignment_2/main.py` from the repository
     root).

  6. Always ensure the simulation is running before you launch the code, otherwise you'll
     get "Failed connecting to remote API server."

  7. If your implementation is correct, the robot will drive itself to the goal sphere
     while steering around obstacles it detects along the way, reacting appropriately to
     the one that's moving.
