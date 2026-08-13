# ED5315 — Introduction to Field and Service Robotics

Assignment and demo files for **ED5315: Introduction to Field and Service Robotics**, offered at the Dept. of Engineering Design, IIT Madras.

## Setup

- OS: Windows 10/11, or Linux (Ubuntu) 22.04 or 24.04
- Python: >=3.10
- CoppeliaSim: 4.10 (EDU) or later

## Installation

1. Download the EDU version of CoppeliaSim [here](https://www.coppeliarobotics.com/downloads). This version is for educational use by students, teachers, professors, schools, and universities — read the license agreement. Familiarize yourself with the CoppeliaSim environment using the documentation [here](https://www.coppeliarobotics.com/helpFiles/index.html).

2. Download this repository (git clone, or download + extract the zip).

3. From the repository root, install the shared `ed5315` package and its dependencies:

   ```
   pip install -e .
   ```

4. Launch CoppeliaSim and open `scenes/mobile_robot.ttt` — this is the one scene shared across every assignment.

To confirm your setup is working before starting an assignment, run the compatibility check in [`Demo`](Demo/README.md).

## Submission

Each assignment has exactly one file you write and submit for automated evaluation -
everything else in that assignment's folder is provided and must not be modified. See
each assignment's own README for which file that is and how to test locally.

## Repository structure

- `Assignment_1` – `Assignment_4`, `Tutorial_1` — individual assignments and tutorial
- `Demo` — compatibility / sensor smoke test ([details](Demo/README.md))
- `ed5315/` — shared Python package imported by every assignment
- `scenes/` — the shared CoppeliaSim scene
