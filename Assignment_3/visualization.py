import os
import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

# Global variables for plotting and odom pose
robot_odom_pose = [4, 4, 0.0]  # x, y position and yaw angle
robot_real_pose = [4, 4, 0.0]  # x, y position and yaw angle
# scale_x, scale_y, x_offset, y_offset = 60.30, 60.30, 93, 200
#scale_x, scale_y, x_offset, y_offset = 0.6030, 0.6030, 93, 200
scale_x, scale_y, x_offset, y_offset = 1,1,0,0
heading_line_length = 0.5
robot_odom_point_plot_handle = None
robot_real_point_plot_handle = None
heading_line_plot_handle_odom = None
heading_line_plot_handle_real = None
prev_robot_odom_x_plot, prev_robot_odom_y_plot = 4, 4
prev_robot_real_x_plot, prev_robot_real_y_plot = 4, 4
error_ellipse_plot = None
ax = None

def plot_initialization():
    global robot_odom_point_plot_handle,robot_real_point_plot_handle, heading_line_plot_handle_odom,heading_line_plot_handle_real, error_ellipse_plot
    global prev_robot_odom_x_plot,prev_robot_real_x_plot, prev_robot_odom_y_plot,prev_robot_real_y_plot, ax
    
    #cwd_path = os.path.dirname(os.path.abspath(__file__))
    plt.ion()
    
    # Initialize figure and axis
    fig,ax = plt.subplots(figsize=(8,6))

    ax.set_xticks([i for i in range(0,21)],minor= False)
    ax.set_yticks([i for i in range(0,21)],minor= False)
    ax.set_xticks([i for i in range(0,41)],minor=True)
    ax.set_yticks([i for i in range(0,41)],minor=True)

    ax.grid(which='minor',color='gray',linestyle='--',linewidth=0.5)
    ax.grid(which='major',color='black',linestyle='-',linewidth=1)

    ax.set_xlim(0,20)
    ax.set_ylim(0,20)

    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    
    # Initial robot_odom position on the plot
    robot_odom_x_plot = (robot_odom_pose[0]/scale_x) + x_offset
    robot_odom_y_plot = (robot_odom_pose[1]/scale_y) + y_offset
    prev_robot_odom_x_plot = robot_odom_x_plot
    prev_robot_odom_y_plot = robot_odom_y_plot
    robot_odom_point_plot_handle = ax.scatter(robot_odom_x_plot, robot_odom_y_plot, alpha=1.0, s=50, color='green')

    # Initial robot_real position on the plot
    robot_real_x_plot = (robot_real_pose[0]/scale_x) + x_offset
    robot_real_y_plot = (robot_real_pose[1]/scale_y) + y_offset
    prev_robot_real_x_plot = robot_real_x_plot
    prev_robot_real_y_plot = robot_real_y_plot
    robot_real_point_plot_handle = ax.scatter(robot_real_x_plot, robot_real_y_plot, alpha=1.0, s=50, color='red')
    
    # Plot initial heading
    heading_x_plot_odom = [robot_odom_x_plot, robot_odom_x_plot + heading_line_length*np.cos(robot_odom_pose[2])]
    heading_y_plot_odom = [robot_odom_y_plot, robot_odom_y_plot + heading_line_length*np.sin(robot_odom_pose[2])]
    heading_line_plot_handle_odom, = ax.plot(heading_x_plot_odom, heading_y_plot_odom,'g-')

    # Plot initial heading for real
    heading_x_plot_real = [robot_real_x_plot, robot_real_x_plot + heading_line_length*np.cos(robot_real_pose[2])]
    heading_y_plot_real = [robot_real_y_plot, robot_real_y_plot + heading_line_length*np.sin(robot_real_pose[2])]
    heading_line_plot_handle_real, = ax.plot(heading_x_plot_real, heading_y_plot_real,'g-')

    
    # Initial error ellipse
    error_ellipse_plot = Ellipse(xy=(robot_odom_x_plot, robot_odom_y_plot), width=1.0, height=1.0, angle=0.0, edgecolor='y', fc='None', lw=2)
    ax.add_patch(error_ellipse_plot)
    
    # Set legend
    #ax.legend()

    #plt.savefig('test.png')

def plot_updation(pose, covariance,realpose):
    """A callback function which plots the updated EKF position and error ellipse."""
    global error_ellipse_plot, prev_robot_odom_x_plot, prev_robot_odom_y_plot,prev_robot_real_x_plot, prev_robot_real_y_plot
    
    # Update the robot_odom pose
    robot_odom_pose[0], robot_odom_pose[1], robot_odom_pose[2] = pose

    # Update the robot_real pose
    robot_real_pose[0], robot_real_pose[1], robot_real_pose[2] = realpose
    
    # Update robot_odom position plot
    robot_odom_x_plot = (robot_odom_pose[0]/scale_x) + x_offset  # 1 was replaced with 0
    robot_odom_y_plot = (robot_odom_pose[1]/scale_y) + y_offset # 0 was replaced with 1
    robot_odom_point_plot_handle.set_offsets([robot_odom_x_plot, robot_odom_y_plot])

    # Update robot_real position plot
    robot_real_x_plot = (robot_real_pose[0]/scale_x) + x_offset  # 1 was replaced with 0
    robot_real_y_plot = (robot_real_pose[1]/scale_y) + y_offset # 0 was replaced with 1
    robot_real_point_plot_handle.set_offsets([robot_real_x_plot, robot_real_y_plot])
    
    # Update robot_odom trail
    ax.plot([robot_odom_x_plot, prev_robot_odom_x_plot], [robot_odom_y_plot, prev_robot_odom_y_plot],'k--')
    prev_robot_odom_x_plot = robot_odom_x_plot
    prev_robot_odom_y_plot = robot_odom_y_plot
    
    # Update robot_real trail
    ax.plot([robot_real_x_plot, prev_robot_real_x_plot], [robot_real_y_plot, prev_robot_real_y_plot],'k--')
    prev_robot_real_x_plot = robot_real_x_plot
    prev_robot_real_y_plot = robot_real_y_plot

    # Update heading plot
    heading_x_plot = [robot_odom_x_plot, robot_odom_x_plot + heading_line_length*np.cos(robot_odom_pose[2])] #This had to be changed to cos
    heading_y_plot = [robot_odom_y_plot, robot_odom_y_plot + heading_line_length*np.sin(robot_odom_pose[2])] #This had to be changed to sin
    heading_line_plot_handle_odom.set_xdata(heading_x_plot)
    heading_line_plot_handle_odom.set_ydata(heading_y_plot)

    # Update heading plot real
    heading_x_plot = [robot_real_x_plot, robot_real_x_plot + heading_line_length*np.cos(robot_real_pose[2])] #This had to be changed to cos
    heading_y_plot = [robot_real_y_plot, robot_real_y_plot + heading_line_length*np.sin(robot_real_pose[2])] #This had to be changed to sin
    heading_line_plot_handle_real.set_xdata(heading_x_plot)
    heading_line_plot_handle_real.set_ydata(heading_y_plot)
    
    # Update the error covariance
    covariance_np_array = np.array(covariance).reshape(3, 3)
    
    # Eigen decomposition for the covariance matrix
    eigenvals, eigenvects = np.linalg.eig(covariance_np_array[0:2, 0:2])
    error_angle = np.arctan2(eigenvects[1, 0], eigenvects[0, 0])
    error_x = np.sqrt(eigenvals[0])
    error_y = np.sqrt(eigenvals[1])
    
    # Remove old ellipse and add a new one
    ellipsex=6
    ellipsey=6
    error_ellipse_plot.remove()
    error_ellipse_plot = Ellipse(xy=(robot_odom_x_plot, robot_odom_y_plot), width=error_x/scale_x, height=error_y/scale_y,
                                 angle=np.rad2deg(robot_odom_pose[2]+error_angle), edgecolor='y', fc='None', lw=2)
    ax.add_patch(error_ellipse_plot)

#Here errory was replaced with errorx
#Here errorx was replaced with errory

#plot_initialization()