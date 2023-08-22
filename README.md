# mobile_robot_sim_setup
Setup for simulating mobile robot in Coppleiasim(V-REP)

## Setup:
OS: Windows 10 
Python: 3.6.x
Coppeliasim: V4.3.0

## Usage:

  1. Download the Edu version of Coppeliasim [here]( https://www.coppeliarobotics.com/downloads). Note that this version is meant only for educational purposes by students, teachers, professors, schools, and universities. Read the license agreement. Once downloaded double click the .exe file to launch the installation. Familiarize yourself with the Coppeliasim environment, use documentation provided [here](https://www.coppeliarobotics.com/helpFiles/index.html)

  2. Download Spyder IDE for Python [here](https://docs.spyder-ide.org/current/installation.html). Remember to use the windows Installer. Once downloaded double click the .exe file to launch the installation. Familiarize yourself with the Spyder IDE interface, refer to the documentation [here](https://docs.spyder-ide.org/current/videos/first-steps-with-spyder.html#getting-started)

  3. Download the setup provided in this repository. If you are familiar with how to use git on windows do that, if not click on the green button that says code and click on download zip. Once the download is complete, double click to extract the contents and place them in a location of your choice, the downloads folder itself works fine.

  4. Launch Coppeliasim, open the provided [scenario](https://github.com/BijoSebastian/mobile_robot_sim_setup/blob/main/mobile%20robot.ttt). Click on File->Open Scene. Navigate to the downloaded setup and select the file “mobile robot.ttt”. Run the simulation by clicking on the light blue play button.

  5. Launch Spyder. Click on File -> Open and navigate to the downloaded setup. Select the file main.py, run it by clicking on the green play button. 
  
  You should be able to see the robot moving towards the green sphere which acts as the goal point. You can move the goal point while the program is running and evaluate how the controller responds. 

Note that this is just a demonstration of some of the basic capabilities available.

     
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

### control.py 
Implements a simple waypoint control. 

### robot_params.py 
Stores the physical parameter values that are important for the simulation
