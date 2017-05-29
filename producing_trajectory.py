#trajectorysampled.msg
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sympy
from sympy.solvers import solve
import math
from pprint import pprint as pp

import rospy
from std_msgs.msg import String

from trajectory_msgs.msg import MultiDOFJointTrajectory
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Pose, Vector3, Transform, Quaternion
from geometry_msgs.msg import Twist

from trajectory_sample import TrajectorySample

import os
import time

trajectory = []
replanned_trajectory = []

def define_coeffs(x0, xk, t, dx0=0, d2x0=0, d3x0=0, d4x0=0, dxk=0, d2xk=0, d3xk=0, d4xk=0):
    """
    FUnction that defines finished expressions for polynome coefficients.
    """
    b0 = x0
    b1 = dx0
    b2 = d2x0/2
    b3 = d3x0/6
    b4 = d3x0/24
    b5 = (t**4*(-5*d4x0 + d4xk) - 12*t**3*(5*d3x0 + 2*d3xk) + 84*t**2*(-5*d2x0 + 3*d2xk) - 336*t*(5*dx0 + 4*dxk) - 3024*x0 + 3024*xk)/(24*t**5)
    b6 = (t**4*(5*d4x0 - 2*d4xk) + 2*t**3*(40*d3x0 + 23*d3xk) + 42*t**2*(15*d2x0 - 11*d2xk) + 336*t*(8*dx0 + 7*dxk) + 5040*x0 - 5040*xk)/(12*t**6)
    b7 = (t**4*(-5*d4x0 + 3*d4xk) - 6*t**3*(15*d3x0 + 11*d3xk) + 12*t**2*(-63*d2x0 + 53*d2xk) - 240*t*(14*dx0 + 13*dxk) - 6480*x0 + 6480*xk)/(12*t**7)
    b8 = (t**4*(5*d4x0 - 4*d4xk) + 12*t**3*(8*d3x0 + 7*d3xk) + 60*t**2*(14*d2x0 - 13*d2xk) + 120*t*(32*dx0 + 31*dxk) + 7560*x0 - 7560*xk)/(24*t**8)
    b9 = (t**4*(-d4x0 + d4xk) - 20*t**3*(d3x0 + d3xk) + 180*t**2*(-d2x0 + d2xk) - 840*t*(dx0 + dxk) - 1680*x0 + 1680*xk)/(24*t**9)

    coeffs = [b9, b8, b7, b6, b5, b4, b3, b2, b1, b0]

    return coeffs

def publisher(newMsg, pub):
    
    time.sleep(0.5)
    rate = rospy.Rate(10) # 10hz
    print "tu sam"
    pub.publish(newMsg)


def calulating_trajectory(x, y, z, t, v_max, a_max, vx=(0,0), vy=(0,0), vz=(0,0), 
                        ax=(0,0), ay=(0,0), az=(0,0), 
                        jx=(0,0), jy=(0,0), jz=(0,0), 
                        sx=(0,0), sy=(0,0), sz=(0,0),
                        optimizing=True):
    """
    Calculating trajectory.
    args:

        x: tuple (x_beginning, x_end)
        y: tuple (y_beginning, y_end)
        z: tuple (z_beginning, z_end)
        t: float (estimated time)
        v_max: float (desired maximum velocity of the drone)
        a_max: float (desired maximum acceleration of the drone)

    return: 

        Sa, Sv: if Sa and Sv != 1 (needed more trajectory calculations)
        newMsg: MultiDOFJointTrajectory -- message to be published to '/firefly/command/trajectory'
    """
    newMsg = MultiDOFJointTrajectory()

    # positions
    x0, x1 = x
    y0, y1 = y
    z0, z1 = z

    # speed
    vx0, vx1 = vx
    vy0, vy1 = vy
    vz0, vz1 = vz

    # accelerations
    ax0, ax1 = ax
    ay0, ay1 = ay
    az0, az1 = az

    # snap
    sx0, sx1 = sx
    sy0, sy1 = sy
    sz0, sz1 = sz

    # jerk
    jx0, jx1 = jx
    jy0, jy1 = jy
    jz0, jz1 = jz


    #defining coefficients for x, y, z axis
    x_coeffs = define_coeffs(x0, x1, t, dx0=vx0, d2x0=ax0, d3x0=jx0, d4x0=sx0, dxk=vx1, d2xk=ax1, d3xk=jx1, d4xk=sx1)
    y_coeffs = define_coeffs(y0, y1, t, dx0=vy0, d2x0=ay0, d3x0=jy0, d4x0=sy0, dxk=vy1, d2xk=ay1, d3xk=jy1, d4xk=sy1)
    z_coeffs = define_coeffs(z0, z1, t, dx0=vz0, d2x0=az0, d3x0=jz0, d4x0=sz0, dxk=vz1, d2xk=az1, d3xk=jz1, d4xk=sz1)
    
    xb9, xb8, xb7, xb6, xb5, xb4, xb3, xb2, xb1, xb0 = x_coeffs
    yb9, yb8, yb7, yb6, yb5, yb4, yb3, yb2, yb1, yb0 = y_coeffs
    zb9, zb8, zb7, zb6, zb5, zb4, zb3, zb2, zb1, zb0 = z_coeffs

    #time sampling
    t_disc = np.linspace(0, t, num=int(100.0*t))

    x = np.zeros(len(t_disc))
    y = np.zeros(len(t_disc))
    z = np.zeros(len(t_disc))

    v_x = np.zeros(len(t_disc))
    v_y = np.zeros(len(t_disc))
    v_z = np.zeros(len(t_disc))

    a_x = np.zeros(len(t_disc))
    a_y = np.zeros(len(t_disc))
    a_z = np.zeros(len(t_disc))

    s_x = np.zeros(len(t_disc))
    s_y = np.zeros(len(t_disc))
    s_z = np.zeros(len(t_disc))

    j_x = np.zeros(len(t_disc))
    j_y = np.zeros(len(t_disc))
    j_z = np.zeros(len(t_disc))

    a_real = np.zeros(len(t_disc))
    v_real = np.zeros(len(t_disc))

    #calculatng speed and acceleration for every time sample
    for idx, t in enumerate(t_disc):

        #position, speed and acceleration for x direction
        x[idx] = xb0 + xb1*t + xb2*(t**2) + xb3*(t**3) + xb4*(t**4) + xb5*(t**5) + xb6*(t**6) + xb7*(t**7) + xb8*(t**8) + xb9*(t**9)
        v_x[idx] = xb1 + 2*xb2*t + 3*xb3*t**2 + 4*xb4*t**3 + 5*xb5*t**4 + 6*xb6*t**5 + 7*xb7*t**6 + 8*xb8*t**7 + 9*xb9*t**8
        a_x[idx] = 2*xb2 + 6*xb3*t + 12*xb4*t**2 + 20*xb5*t**3 + 30*xb6*t**4 + 42*xb7*t**5 + 56*xb8*t**6 + 72*xb9*t**7
        j_x[idx] = 6*xb3 + 24*xb4*t + 60*xb5*t**2 + 120*xb6*t**3 + 210*xb7*t**4 + 336*xb8*t**5 + 504*xb9*t**6
        s_x[idx] = 24*xb4 + 120*xb5*t + 360*xb6*t**2 + 840*xb7*t**3 + 1680*xb8*t**4 + 3024*xb9*t**5

        #position, speed and acceleration for y direction
        y[idx] = yb0 + yb1*t + yb2*(t**2) + yb3*(t**3) + yb4*(t**4) + yb5*(t**5) + yb6*(t**6) + yb7*(t**7) + yb8*(t**8) + yb9*(t**9)
        v_y[idx] = yb1 + 2*yb2*t + 3*yb3*t**2 + 4*yb4*t**3 + 5*yb5*t**4 + 6*yb6*t**5 + 7*yb7*t**6 + 8*yb8*t**7 + 9*yb9*t**8
        a_y[idx] = 2*yb2 + 6*yb3*t + 12*yb4*t**2 + 20*yb5*t**3 + 30*yb6*t**4 + 42*yb7*t**5 + 56*yb8*t**6 + 72*yb9*t**7
        j_y[idx] = 6*yb3 + 24*yb4*t + 60*yb5*t**2 + 120*yb6*t**3 + 210*yb7*t**4 + 336*yb8*t**5 + 504*yb9*t**6
        s_y[idx] = 24*yb4 + 120*yb5*t + 360*yb6*t**2 + 840*yb7*t**3 + 1680*yb8*t**4 + 3024*yb9*t**5

        #position, speed and acceleration for z direction
        z[idx] = zb0 + zb1*t + zb2*(t**2) + zb3*(t**3) + zb4*(t**4) + zb5*(t**5) + zb6*(t**6) + zb7*(t**7) + zb8*(t**8) + zb9*(t**9)        
        v_z[idx] = zb1 + 2*zb2*t + 3*zb3*t**2 + 4*zb4*t**3 + 5*zb5*t**4 + 6*zb6*t**5 + 7*zb7*t**6 + 8*zb8*t**7 + 9*zb9*t**8
        a_z[idx] = 2*zb2 + 6*zb3*t + 12*zb4*t**2 + 20*zb5*t**3 + 30*zb6*t**4 + 42*zb7*t**5 + 56*zb8*t**6 + 72*zb9*t**7      
        j_z[idx] = 6*zb3 + 24*zb4*t + 60*zb5*t**2 + 120*zb6*t**3 + 210*zb7*t**4 + 336*zb8*t**5 + 504*zb9*t**6
        s_z[idx] = 24*zb4 + 120*zb5*t + 360*zb6*t**2 + 840*zb7*t**3 + 1680*zb8*t**4 + 3024*zb9*t**5

        v_real[idx] = math.sqrt(v_x[idx]**2 + v_y[idx]**2 + v_z[idx]**2)
        a_real[idx] = math.sqrt(a_x[idx]**2 + a_y[idx]**2 + a_z[idx]**2)

    pos = (x, y, z)
    vel = (v_x, v_y, v_z)
    acc = (a_x, a_y, a_z)

    newMsg = producing_message(pos, vel, acc, t_disc, newMsg)

    if not optimizing:
        return newMsg
    
    v_real_max = np.amax(v_real)
    a_real_max = np.amax(a_real)

    Sv = v_real_max/v_max
    Sa = math.sqrt(a_real_max/a_max)

    if round(Sa, 2) == 1.0 or round(Sv, 2) == 1.0:

        vels = (v_x, v_y, v_z)
        accs = (a_x, a_y, a_z)
        pos = (x, y, z)
        jerks = (j_x, j_y, j_z)
        snaps = (s_x, s_y, s_z)

        appending_to_trajectory(t_disc, vels, accs, pos, jerks, snaps)
        return newMsg

    return (Sa, Sv)

def producing_message(pos, vel, acc, t_disc, newMsg):
    """
    This function produces message to publish 
    from pos, vel, acc. 

    pos - tuple of lists (x, y, z) x: list of all x positions in trajectory
    vel - tuple of lists (v_x, v_y, v_z) v_x: list of all x positions in trajectory
    acc - tuple of lists (a_x, a_y, a_z) a_x: list of all x accelerations in trajectory
    t_disc - list (time)
    newMsg - already created (empty?) message
    """

    x, y, z = pos
    v_x, v_y, v_z = vel
    a_x, a_y, a_z = acc

    for idx, t in enumerate(t_disc):

        # initialization of message components
        newPoint = MultiDOFJointTrajectoryPoint()

        newTransform = Transform()
        newQuaternion = Quaternion()
        newVelocities = Twist()
        newAccelerations = Twist()
        
        try:
            newTransform.translation.x = x[idx]
            newTransform.translation.y = y[idx]
            newTransform.translation.z = z[idx]
        except:
            import pdb; pdb.set_trace()

        # appending position in this point
        newPoint.transforms.append(newTransform)

        newVelocities.linear.x = v_x[idx]
        newVelocities.linear.y = v_y[idx]
        newVelocities.linear.z = v_z[idx]

        # appending linear velocities in this point
        newPoint.velocities.append(newVelocities)

        newAccelerations.linear.x = a_x[idx]
        newAccelerations.linear.y = a_y[idx]
        newAccelerations.linear.z = a_z[idx]

        # appending linear accelarations in this point
        newPoint.accelerations.append(newAccelerations)

        # time from start in seconds
        newPoint.time_from_start = rospy.Duration(t)

        newMsg.points.append(newPoint)

    return newMsg

def appending_to_trajectory(t_disc, vels, accs, pos, jerks, snaps):

    v_x, v_y, v_z = vels
    a_x, a_y, a_z = accs
    x, y, z = pos
    j_x, j_y, j_z = jerks
    s_x, s_y, s_z = snaps

    trajSample = TrajectorySample()

    # making of trajSample object

    # saving time to object trajSample
    trajSample.time = t_disc
    
    # saving velocities to object trajSample
    trajSample.velocities["v_x"] = v_x
    trajSample.velocities["v_y"] = v_y
    trajSample.velocities["v_z"] = v_z

    # saving accelerations to object trajSample
    trajSample.accelerations["a_x"] = a_x
    trajSample.accelerations["a_y"] = a_y
    trajSample.accelerations["a_z"] = a_z

    # saving positions to object trajSample
    trajSample.positions["x"] = x
    trajSample.positions["y"] = y
    trajSample.positions["z"] = z

    # saving jerks to object trajSample
    trajSample.jerk["j_x"] = j_x
    trajSample.jerk["j_y"] = j_y
    trajSample.jerk["j_z"] = j_z

    # saving snaps to object trajSample
    trajSample.snap["s_x"] = s_x
    trajSample.snap["s_y"] = s_y
    trajSample.snap["s_z"] = s_z

    trajectory.append(trajSample)


def trajectory_two_points(point0, point1, v_max, a_max, pub, speedp0 = (0,0,0), speedp1=(0,0,0), \
                            accp0=(0,0,0), accp1=(0,0,0), jerkp0=(0,0,0), jerkp1=(0,0,0), snapp0=(0,0,0), snapp1=(0,0,0), optimizing = True):
    """
    Function that generates trajectory between two points.

    args:

        point0: tuple (x0, y0, z0)
        point1: tuple (x1, y1, z1)
        v_max: float -- maximal wanted speed
        a_max: float -- maximal wanted acceleration
        pub: Publisher to which you want to publish your trajectory
    
    return:

        newMsg: MultiDOFJointTrajectory -- message to be published to '/firefly/command/trajectory'

    """

    x0, y0, z0 = point0
    x1, y1, z1 = point1
    vx0, vy0, vz0 = speedp0
    vx1, vy1, vz1 = speedp1
    ax0, ay0, az0 = accp0
    ax1, ay1, az1 = accp1
    sx0, sy0, sz0 = snapp0
    sx1, sy1, sz1 = snapp1
    jx0, jy0, jz0 = jerkp0
    jx1, jy1, jz1 = jerkp1

    #calculating estimated t
    t = math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)/v_max
    
    trajectory_between_two_points = calulating_trajectory((x0, x1), (y0, y1), (z0, z1), t, v_max, a_max, vx=(vx0, vx1), vy=(vy0, vy1), vz=(vz0, vz1), 
                        ax=(ax0, ax1), ay=(ay0, ay1), az=(az0, az1), jx=(jx0, jx1), jy=(jy0, jy1), 
                        jz=(jz0, jz1), sx=(sx0, sx1), sy=(sy0, sy1), sz=(sz0, sz1), optimizing = optimizing)
    
    if not optimizing:
        return trajectory_between_two_points

    # while Sa != 1 and Sv != 1 (need for more calculating trajectory)
    while isinstance(trajectory_between_two_points, tuple):
        Sa, Sv = trajectory_between_two_points
        factor = max(Sa, Sv)
        t = factor*t

        trajectory_between_two_points = calulating_trajectory((x0, x1), (y0, y1), (z0, z1), t, v_max, a_max, vx=(vx0, vx1), vy=(vy0, vy1), vz=(vz0, vz1), 
                        ax=(ax0, ax1), ay=(ay0, ay1), az=(az0, az1), jx=(jx0, jx1), jy=(jy0, jy1), 
                        jz=(jz0, jz1), sx=(sx0, sx1), sy=(sy0, sy1), sz=(sz0, sz1), optimizing = optimizing)
    
    return trajectory_between_two_points

def multiple_points(points, v_max, a_max, pub):
    """
    Function that publishes trajectory of multiple points to publisher pub.

    args:

        points: list of tuples (x, y, z) -- (multiple points that we want to add to trajectory)
        v_max: float -- maximal wanted speed
        a_max: float -- maximal wanted acceleration
        pub: Publisher to which you want to publish your trajectory

    """
    traj_2_points = MultiDOFJointTrajectory()
    for i, point in enumerate(points):
        try:
            traj_2_points.points.extend(trajectory_two_points(point, points[i+1], v_max, a_max, pub).points)
        except IndexError:
            pass
    #publisher(traj_2_points, pub)

def derivations_of_small_traj(small_trajectory, idx_for_replan):
    # getting positions in points which we want to replan trajectory
    # for
    pos = (small_trajectory.positions["x"][idx_for_replan],
                small_trajectory.positions["y"][idx_for_replan],
                small_trajectory.positions["z"][idx_for_replan])


    # getting velocities in points which we want to replan trajectory
    # for
    vel = (small_trajectory.velocities["v_x"][idx_for_replan],
                small_trajectory.velocities["v_y"][idx_for_replan],
                small_trajectory.velocities["v_z"][idx_for_replan])


    # getting accelerations in points which we want to replan trajectory
    # for
    acc = (small_trajectory.accelerations["a_x"][idx_for_replan], 
            small_trajectory.accelerations["a_y"][idx_for_replan],
            small_trajectory.accelerations["a_z"][idx_for_replan])

    # getting jerk in points which we want to replan trajectory
    # for
    jerk = (small_trajectory.jerk["j_x"][idx_for_replan], 
            small_trajectory.jerk["j_y"][idx_for_replan],
            small_trajectory.jerk["j_z"][idx_for_replan])

    # getting snap in points which we want to replan trajectory
    # for
    snap = (small_trajectory.snap["s_x"][idx_for_replan],
            small_trajectory.snap["s_y"][idx_for_replan],
            small_trajectory.snap["s_z"][idx_for_replan])

    return [pos, vel, acc, jerk, snap]

def run():

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10) # your node is publishing to the chatter topic usting the msg type String

    v_max = 5
    a_max = 3

    points = []
    with open("trajectory_points.txt", 'r+') as traj_file:
        for line in traj_file:
            point = eval(line.strip(os.linesep))
            points.append(point)

    multiple_points(points, v_max, a_max, pub)

    # defining percentage, begin point of trajectory,
    # and initialization of final message to be published
    percentage = 0.7
    begin_point = 0
    finalMsg = MultiDOFJointTrajectory()

    last_traj_flag = False

    # for every part of trajectory (between two points)
    for i, small_trajectory in enumerate(trajectory):

        # lentgh of part of trajectory
        small_traj_len = len(small_trajectory.time)

        # index of part of trajectory in which you want to replan 
        # trajectory you already have
        idx_for_replan = int(round(small_traj_len*percentage))

        # next trajectory in line 
        try:
            next_traj = trajectory[i+1]
        except IndexError:
            idx_for_replan = small_traj_len - 1
            last_traj_flag = True

        # slicing trajectory from begin_point to point in which 
        # we want to replan trajectory
        pos = (small_trajectory.positions["x"][begin_point:idx_for_replan],
                small_trajectory.positions["y"][begin_point:idx_for_replan],
                small_trajectory.positions["z"][begin_point:idx_for_replan])

        vel = (small_trajectory.velocities["v_x"][begin_point:idx_for_replan],
                small_trajectory.velocities["v_y"][begin_point:idx_for_replan],
                small_trajectory.velocities["v_z"][begin_point:idx_for_replan])

        acc = (small_trajectory.accelerations["a_x"][begin_point:idx_for_replan],
                small_trajectory.accelerations["a_y"][begin_point:idx_for_replan],
                small_trajectory.accelerations["a_z"][begin_point:idx_for_replan])

        t_disc = small_trajectory.time[begin_point:idx_for_replan]
        finalMsg = producing_message(pos, vel, acc, t_disc, finalMsg)

        if last_traj_flag:
            continue

        # length of the next piece of trajectory
        next_traj_len = len(next_traj.time)

        # index in which you want to land in next part of trajectory
        next_idx_for_replan = int(round(next_traj_len*(1.0 - percentage)))

        # calculating values of derivations in points from and to which 
        # we want to replan
        [point0, vp0, ap0, jp0, sp0] = derivations_of_small_traj(small_trajectory, idx_for_replan)
        [point1, vp1, ap1, jp1, sp1] = derivations_of_small_traj(next_traj, next_idx_for_replan)
        
        newMsg_extend = trajectory_two_points(point0, point1,  v_max, a_max, pub, speedp0 = vp0, speedp1=vp1, \
                            accp0=ap0, accp1=ap1, jerkp0=jp0, jerkp1=jp1, snapp0=sp0, snapp1=sp1, optimizing = False)

        finalMsg.points.extend(newMsg_extend.points)

        begin_point = next_idx_for_replan

    publisher(finalMsg, pub)

if __name__ == "__main__":
    run()
