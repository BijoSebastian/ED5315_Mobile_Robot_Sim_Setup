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


def sim_init():
  global sim
  global client_ID
  
  #Initialize sim interface
  sim.simxFinish(-1) # just in case, close all opened connections
  client_ID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim    
  if client_ID!=-1:
    print ('Connected to remote API server')
    return True
  else:
    return False

def get_handles():
  #Get the handles to the sim items

  global pioneer_handle
  global pioneer_left_motor_handle
  global pioneer_right_motor_handle
  global goal_handle

  # Handle to Pioneer1:
  res , pioneer_handle = sim.simxGetObjectHandle(client_ID, "/Pioneer1", sim.simx_opmode_blocking)
  res,  pioneer_left_motor_handle = sim.simxGetObjectHandle(client_ID, "/Pioneer1/Pioneer1_left", sim.simx_opmode_blocking)
  res,  pioneer_right_motor_handle = sim.simxGetObjectHandle(client_ID, "/Pioneer1/Pioneer1_right", sim.simx_opmode_blocking)

  # Get the position of the Pioneer1 for the first time in streaming mode
  res , pioneer_1_Position = sim.simxGetObjectPosition(client_ID, pioneer_handle, -1 , sim.simx_opmode_streaming)
  res , pioneer_1_Orientation = sim.simxGetObjectOrientation(client_ID, pioneer_handle, -1 , sim.simx_opmode_streaming)
  
  # Stop all joint actuations:Make sure Pioneer1 is stationary:
  res = sim.simxSetJointTargetVelocity(client_ID, pioneer_left_motor_handle, 0, sim.simx_opmode_streaming)
  res = sim.simxSetJointTargetVelocity(client_ID, pioneer_right_motor_handle, 0, sim.simx_opmode_streaming)
  
  # Handle to the goal:
  res , goal_handle = sim.simxGetObjectHandle(client_ID, "/Sphere", sim.simx_opmode_blocking)
  
  # Get the position of the goal for the first time in streaming mode
  res , goalPosition = sim.simxGetObjectPosition(client_ID, goal_handle, -1 , sim.simx_opmode_streaming)

  print ("Succesfully obtained handles")

  return

def get_wall_seg(centerPos, length, zAngle):
  #Function to get the end points of a single wall segment
  
  cosA = np.cos(zAngle)
  sinA = np.sin(zAngle)

  # Start point:
  xs = centerPos[0]  - sinA*(length/2)
  ys = centerPos[1]  + cosA*(length/2)

  # End point:
  xe = centerPos[0]  + sinA*(length/2)
  ye = centerPos[1]  - cosA*(length/2)

  return [xs, ys, xe, ye]

def get_maze_segments():
  # Function to get the maze sgements from the wall handles list:
  # Format: Returns a list called mazeSegments:
  # mazeSegments is a list of segments -> Each segment is a list of two tuples corresponding to start and end points -> Each tuple contains (x,y) co-ods  of the points
  
  global sim
  global client_ID
  
  ## Get handles to:
  # The Maze collection:
  res , mazeHandle = sim.simxGetCollectionHandle(client_ID, "Maze", sim.simx_opmode_blocking)
  # Get the handles associated with each wall and the absolute position of the center of each wall:
  res, wallHandles , intData, absPositions, stringData = sim.simxGetObjectGroupData(client_ID, mazeHandle, 3,sim.simx_opmode_blocking)
  mazeSegments = []
  if res == sim.simx_return_ok:
    count = 1
    for wall in wallHandles:
      res, wallCenterAbsPos = sim.simxGetObjectPosition(client_ID, wall, -1, sim.simx_opmode_oneshot_wait)
      res, wallMinY = sim.simxGetObjectFloatParameter(client_ID,wall, sim.sim_objfloatparam_objbbox_min_y , sim.simx_opmode_oneshot_wait)
      res, wallMaxY = sim.simxGetObjectFloatParameter(client_ID,wall, sim.sim_objfloatparam_objbbox_max_y , sim.simx_opmode_oneshot_wait)
      wallLength = abs(wallMaxY - wallMinY)
      # Get the orientation of the wall: Third euler angle is the angle around z-axis:
      res , wallOrient = sim.simxGetObjectOrientation(client_ID, wall, -1 , sim.simx_opmode_oneshot_wait)
      # Get the end points of the maze wall: A list containing two tuples: [ (xs,ys) , (xe,ye)]
      wallSeg = get_wall_seg(wallCenterAbsPos, wallLength, wallOrient[2])    # Assuming all walls are on the ground and nearly flat:
      print ("Wall #" , count, " -> " , wallSeg)
      mazeSegments.append(wallSeg)
      count+=1
  else:
    print (" Failed to get individual wall handles!")

  return np.array(mazeSegments)

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

def localize_robot():
  #Function that will return the current location of Pioneer
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
  global sim
  global client_ID
  global pioneer_handle
  
  res , pioneer_Position = sim.simxGetObjectPosition(client_ID, pioneer_handle, -1 , sim.simx_opmode_buffer)
  res , pioneer_Orientation = sim.simxGetObjectOrientation(client_ID, pioneer_handle, -1 , sim.simx_opmode_buffer)
  
  x = pioneer_Position[0]
  y = pioneer_Position[1]
  theta  =pioneer_Orientation[2]
  print("robot", x,y,theta)

  return [x,y,theta]       

def get_goal_pose():
  #Function that will return the goal pose
  #PS. THE ORIENTATION WILL BE RETURNED IN RADIANS        
  global sim
  global client_ID
  global goal_handle
  
  #Obtain goal position
  res , goalPosition = sim.simxGetObjectPosition(client_ID, goal_handle, -1 , sim.simx_opmode_buffer)
  res , goalOrientation = sim.simxGetObjectOrientation(client_ID, goal_handle, -1 , sim.simx_opmode_buffer)
  
  x = goalPosition[0]
  y = goalPosition[1]
  theta  = goalOrientation[2]
  print("goal", x,y,theta)

  return [x,y,theta]    

def setvel_pioneers(V, W):
  #Function to set the linear and rotational velocity of pioneers
  global sim
  global client_ID
  global pioneer_left_motor_handle
  global pioneer_right_motor_handle

  # Limit v,w from controller to +/- of their max
  w = max(min(W, robot_params.pioneer_max_W), -1.0*robot_params.pioneer_max_W)
  v = max(min(V, robot_params.pioneer_max_V), -1.0*robot_params.pioneer_max_V)
          
  # Compute desired vel_r, vel_l needed to ensure w
  Vr = ((2.0*v) + (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
  Vl = ((2.0*v) - (w*robot_params.pioneer_track_width))/(2*robot_params.pioneer_wheel_radius)
                      
  # Set velocity
  sim.simxSetJointTargetVelocity(client_ID, pioneer_left_motor_handle, Vl, sim.simx_opmode_oneshot_wait)
  sim.simxSetJointTargetVelocity(client_ID, pioneer_right_motor_handle, Vr, sim.simx_opmode_oneshot_wait)
  
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
