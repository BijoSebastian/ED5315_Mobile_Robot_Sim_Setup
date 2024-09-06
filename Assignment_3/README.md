# ED5315 Assignment-3
Odometry with Covariance for a mobile robot (Visualized in Coppleiasim(V-REP))

## Setup:
OS: Windows 10/11; Ubuntu 20.04


Python: 3.6.x
Coppeliasim: V4.3.0

To check the compatibility of your system, follow the instructions [here](https://github.com/BijoSebastian/ED5315_Mobile_Robot_Sim_Setup/tree/main/Demo) and run the demo script.

## Instructions:

  1. Download the setup provided in this repository. If you are familiar with how to use git on windows do that, if not click on the green button that says code and click on download zip. Once the download is complete, double click to extract the contents and place them in a location of your choice, the downloads folder itself works fine.

  2. Complete the **f_p** and **f_u** functions in the file **covar_mat_sub.py**. You need to define the Jacobian matrix for Error propogation calculations.  Do not make any changes to the other code files provided to you.

  3. Once you have completed defining the matrices (**F_p and F_u**), launch Coppeliasim. Click on File->Open Scene. Navigate to the downloaded setup and select the file “mobile robot_odom.ttt”. Run the simulation by clicking on the light blue play button.

  4. Launch Spyder. Click on File -> Open and navigate to the downloaded setup. Select the file main.py, run it by clicking on the green play button.(Always ensure you are in the same repository!) 

  5. Always ensure that the simulation is running before you launch the code, otherwise you will get an error that says **"Failed connecting to the remote API server. Program ended"**.

  6.	If you had defined the matrix correctly, you will see a robot moving along with a phantom robot. The real robot is the ground truth and the phantom robot 's position is based on the odometry calculations.

## Solution video:
The goal pose moves randomly every time a new instance is launched.

![Solution run 1](solution/Solution1.gif)
