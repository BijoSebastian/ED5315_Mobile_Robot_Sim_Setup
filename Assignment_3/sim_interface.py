import numpy as np
import robot_params

try:
  import sim
except:
  print ('--------------------------------------------------------------')
  print ('"sim.py" could not be imported. This means very probably that')
  print ('either "sim.py" or the remoteApi library could not be found.')
  print ('Make sure both are in the same folder as this file,')
  print ('or appropriately adjust the file "sim.py"')
  print ('--------------------------------------------------------------')
  print ('')

client_ID = []
prev_time = 0.0


def sim_init():
  global sim
  global client_ID
  
  #Initialize sim interface
  sim.simxFinish(-1) # just in case, close all opened connections
  client_ID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to CoppeliaSim   
  sim_timer() 
  if client_ID!=-1:
    print ('Connected to remote API server')
    return True
  else:
    return False

def get_handles():
  #Get the handles to the sim items

  global pioneer_handle1
  global pioneer_left_motor_handle1
  global pioneer_right_motor_handle1
  global pioneer_handle2
  global pioneer_left_motor_handle2
  global pioneer_right_motor_handle2
  global goal_handle

  # Handle to Pioneer1:
  res , pioneer_handle1 = sim.simxGetObjectHandle(client_ID, "/Pioneer1", sim.simx_opmode_blocking)
  res,  pioneer_left_motor_handle1 = sim.simxGetObjectHandle(client_ID, "/Pioneer1/Pioneer1_left", sim.simx_opmode_blocking)
  res,  pioneer_right_motor_handle1 = sim.simxGetObjectHandle(client_ID, "/Pioneer1/Pioneer1_right", sim.simx_opmode_blocking)

  # Handle to Pioneer2:
  res , pioneer_handle2 = sim.simxGetObjectHandle(client_ID, "/Pioneer2", sim.simx_opmode_blocking)
  res,  pioneer_left_motor_handle2 = sim.simxGetObjectHandle(client_ID, "/Pioneer2/Pioneer2_left", sim.simx_opmode_blocking)
  res,  pioneer_right_motor_handle2 = sim.simxGetObjectHandle(client_ID, "/Pioneer2/Pioneer2_right", sim.simx_opmode_blocking)

  # Get the position of the Pioneer1 for the first time in streaming mode
  res , pioneer_1_Position = sim.simxGetObjectPosition(client_ID, pioneer_handle1, -1 , sim.simx_opmode_streaming)
  res , pioneer_1_Orientation = sim.simxGetObjectOrientation(client_ID, pioneer_handle1, -1 , sim.simx_opmode_streaming)

  # Get the position of the Pioneer2 for the first time in streaming mode
  res , pioneer_2_Position = sim.simxGetObjectPosition(client_ID, pioneer_handle2, -1 , sim.simx_opmode_streaming)
  res , pioneer_2_Orientation = sim.simxGetObjectOrientation(client_ID, pioneer_handle2, -1 , sim.simx_opmode_streaming)
  
  # Stop all joint actuations:Make sure Pioneer1 is stationary:
  res = sim.simxSetJointTargetVelocity(client_ID, pioneer_left_motor_handle1, 0, sim.simx_opmode_streaming)
  res = sim.simxSetJointTargetVelocity(client_ID, pioneer_right_motor_handle1, 0, sim.simx_opmode_streaming)

  # Stop all joint actuations:Make sure Pioneer2 is stationary:
  res = sim.simxSetJointTargetVelocity(client_ID, pioneer_left_motor_handle2, 0, sim.simx_opmode_streaming)
  res = sim.simxSetJointTargetVelocity(client_ID, pioneer_right_motor_handle2, 0, sim.simx_opmode_streaming)
  
  #These two lines spawn the robot at the random points range given below, at the start of simulation
  # x=np.random.randint(12.5,20,size=1)
  # y=np.random.randint(19,21,size=1)
  # sim.simxSetObjectPosition(client_ID, goal_handle, -1 ,[x,y,0.35], sim.simx_opmode_streaming)
  
  # Get the position of the goal for the first time in streaming mode
  # res , goalPosition = sim.simxGetObjectPosition(client_ID, goal_handle, -1 , sim.simx_opmode_streaming)

  print ("Succesfully obtained handles")

  return

def start_simulation():
  global sim
  global client_ID

  ###Start the Simulation: Keep printing out status messages!!!
  res = sim.simxStartSimulation(client_ID, sim.simx_opmode_oneshot_wait)

  if res == sim.simx_return_ok:
    print ("---!!! Started Simulation !!! ---")
    return True
  else:
    return False

def localize_robot1():
  #Function that will return the current location of Pioneer
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
  global sim
  global client_ID
  global pioneer_handle1
  
  res , pioneer_Position = sim.simxGetObjectPosition(client_ID, pioneer_handle1, -1 , sim.simx_opmode_buffer)
  res , pioneer_Orientation = sim.simxGetObjectOrientation(client_ID, pioneer_handle1, -1 , sim.simx_opmode_buffer)
  
  x = pioneer_Position[0]
  y = pioneer_Position[1]
  theta  =pioneer_Orientation[2]

  return [x,y,theta]

def localize_robot2():
  #Function that will return the current location of Pioneer
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
  global sim
  global client_ID
  global pioneer_handle1
  
  res , pioneer_Position = sim.simxGetObjectPosition(client_ID, pioneer_handle2, -1 , sim.simx_opmode_buffer)
  res , pioneer_Orientation = sim.simxGetObjectOrientation(client_ID, pioneer_handle2, -1 , sim.simx_opmode_buffer)
  
  x = pioneer_Position[0]
  y = pioneer_Position[1]
  theta  =pioneer_Orientation[2]

  return [x,y,theta]        

def sim_timer():
    global sim
    global client_ID
    global prev_time
    
    new_time = sim.simxGetLastCmdTime(client_ID)
    delta_sim_time = (new_time - prev_time)/1000.0
    prev_time = new_time
    return delta_sim_time
    
def change_pioneer1_pose(x,y,theta):
  #Function to change goal pose 
  global sim
  global client_ID
  global pioneer_handle1
  
  
  sim.simxSetObjectPosition(client_ID, pioneer_handle1, -1 ,[x,y,0.35], sim.simx_opmode_streaming)

  sim.simxSetObjectOrientation(client_ID, pioneer_handle1, -1, [0, 0, theta], sim.simx_opmode_streaming)
  
  return

def change_pioneer2_pose(x,y,theta):
  #Function to change goal pose 
  global sim
  global client_ID
  global pioneer_handle2
  
  
  sim.simxSetObjectPosition(client_ID, pioneer_handle2, -1 ,[x,y,0.35], sim.simx_opmode_streaming)

  sim.simxSetObjectOrientation(client_ID, pioneer_handle2, -1, [0, 0, theta], sim.simx_opmode_streaming)
  
  return
          
def get_goal_pose():
  #Function that will return the goal pose
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
  global sim
  global client_ID
  global goal_handle
    
  #Obtain goal position
  res , goalPosition = sim.simxGetObjectPosition(client_ID, goal_handle, -1 , sim.simx_opmode_streaming)
  res , goalPosition = sim.simxGetObjectPosition(client_ID, goal_handle, -1 , sim.simx_opmode_buffer)
  res , goalOrientation = sim.simxGetObjectOrientation(client_ID, goal_handle, -1 , sim.simx_opmode_buffer)
    
  x = goalPosition[0]
  y = goalPosition[1]
  theta  = goalOrientation[2]
  print("goal", x,y,theta)
  
  return [x,y,theta]    

def setvel_pioneers1(V, W):
  #Function to set the linear and rotational velocity of pioneers
  global sim
  global client_ID
  global pioneer_left_motor_handle1
  global pioneer_right_motor_handle1

  # Limit v,w from controller to +/- of their max
  w = max(min(W, robot_params.pioneer_max_W), -1.0*robot_params.pioneer_max_W)
  v = max(min(V, robot_params.pioneer_max_V), -1.0*robot_params.pioneer_max_V)
          
  # Compute desired vel_r, vel_l needed to ensure w
  Vr = ((2.0*v) + (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
  Vl = ((2.0*v) - (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
                      
  # Set velocity
  sim.simxSetJointTargetVelocity(client_ID, pioneer_left_motor_handle1, Vl, sim.simx_opmode_oneshot_wait)
  sim.simxSetJointTargetVelocity(client_ID, pioneer_right_motor_handle1, Vr, sim.simx_opmode_oneshot_wait)
  
  return 

def setvel_pioneers2(V, W):
  #Function to set the linear and rotational velocity of pioneers
  global sim
  global client_ID
  global pioneer_left_motor_handle2
  global pioneer_right_motor_handle2

  # Limit v,w from controller to +/- of their max
  w = max(min(W, robot_params.pioneer_max_W), -1.0*robot_params.pioneer_max_W)
  v = max(min(V, robot_params.pioneer_max_V), -1.0*robot_params.pioneer_max_V)
          
  # Compute desired vel_r, vel_l needed to ensure w
  Vr = ((2.0*v) + (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
  Vl = ((2.0*v) - (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
                      
  # Set velocity
  sim.simxSetJointTargetVelocity(client_ID, pioneer_left_motor_handle2, Vl, sim.simx_opmode_oneshot_wait)
  sim.simxSetJointTargetVelocity(client_ID, pioneer_right_motor_handle2, Vr, sim.simx_opmode_oneshot_wait)
  
  return 

def sim_shutdown():
  #Gracefully shutdown simulation

  global sim
  global client_ID

  #Stop simulation
  res = sim.simxStopSimulation(client_ID, sim.simx_opmode_oneshot_wait)
  if res == sim.simx_return_ok:
    print ("---!!! Stopped Simulation !!! ---")

  # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
  sim.simxGetPingTime(client_ID)

  # Now close the connection to CoppeliaSim:
  sim.simxFinish(client_ID)      

  return            
