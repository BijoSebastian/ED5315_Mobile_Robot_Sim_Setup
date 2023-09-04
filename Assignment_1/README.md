# ED5315 Assignment-1
Waypoint control of a mobile robot(differential drive) in Coppleiasim(V-REP)

## Setup:
OS: Windows 10/11; Ubuntu: 18/20; Mac


Python: 3.6.x
Coppeliasim: V4.3.0

To check the compatibility of your system, follow the instructions [here]().

## Instructions:

  1. Download the setup provided in this repository. If you are familiar with how to use git on windows do that, if not click on the green button that says code and click on download zip. Once the download is complete, double click to extract the contents and place them in a location of your choice, the downloads folder itself works fine.

  2. Complete the **gtg function** (go to goal) in the file **control.py**.

  Read through the **control.py** file and implement **PD controller** for **angular velocity**, and **P controller** for **linear velocity**. 
  
  To get the pseudo code for waypoint control read the lecture slides from .

  3. Once you have completed the go to goal implementation(**gtg function**), launch Coppeliasim, open the provided [scenario](). Click on File->Open Scene. Navigate to the downloaded setup and select the file “mobile robot.ttt”. Run the simulation by clicking on the light blue play button.

  4. Launch Spyder. Click on File -> Open and navigate to the downloaded setup. Select the file main.py, run it by clicking on the green play button.(Always ensure you are in the same repository!) 

  5. Always ensure that the simulation is running before you launch the code, otherwise you will get an error that says **"Failed connecting to the remote API server. Program ended"**.

  6.	If your implementation of the gtg function(goal to goal) is correct, you will see robot moving to the position of green blob(which is goal). To check for robustness of your implementation, the goal/green blob will spawn again.(Thus implementation is checked for two random goals!)

7. ***Instructions for uploading solution***:	You need to add these three files in a zip folder and upload it to moodle: 
    - Upload  **only** the completed **control.py** file.(Do not make any changes to other files)
    - A writeup/report on the control calculation that you implemented.
    - Take a screen recording of the coppeliasim screen and IDE side by side as shown in solution video below.

## Solution video:
The goal pose moves randomly every time a new instance in launched.


  ![Example Solution](ED5315_Mobile_Robot_Sim_Setup/Assignment_1/solution/solution_example_2.mp4)
     
## Working explained:

The provided scenario consists of a Pioneer P3DX robot, a camera that points to the robot and a green goal point for the robot. The robot is in a flat terrain with walls. The view from the camera pointing to the robot is shown in inset on the top right corner of the screen. 

### main.py
Sets up the simulation. All of the interactions with Coppleiasim are contained within the sim_interface file. For the sim_interface to work make sure you have following files in your directory:
1. sim.py
2. simConst.py
3. the appropriate remote API library: "remoteApi.dll" (Windows), "remoteApi.dylib" (Mac) or "remoteApi.so" (Linux) 

### sim_interface.py
Sets up the connection to the simulator, obtains handles for objects in simulation, uses the handles to obtain the position of the walls within the simulated scenario, read the position and orientation of the robot as well as goal point, set velocities to the wheels on the robot, as well as start and shutdown the simulation. 

The code relies entirely on the [Legacy remote API functions (Python)](https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm). The [sim_interface file](https://github.com/BijoSebastian/mobile_robot_sim_setup/blob/main/sim_interface.py) contains wrappers on top of the API functions to simplify mobile robot simulation within Coppeliasim. Some of the wrapper functions are explained below:

#### sim_init()
Closes all existing connections to Coppleiasim and creates a new connection to the default socket opened by Coppleiasim at the start of the simulation

#### get_handles()
To interact with any object in Coppleiasim we need to first obtain the object's handle. This handle needs to be provided in subsequent function calls for setting the objects position, reading the objects position, setting velocity for motor etc.

#### start_simulation()
Wrapper for the simxStartSimulation function to check if the simulation started without errors.

### robot_params.py 
Stores the physical parameter values that are important for the simulation
