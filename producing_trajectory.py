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

import os
import time

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

def calculate_coeffs():
    """
    Functions that calculates coefficients 
    for list of sympy equations.
    """

    #defining symbols used for further calculation
    t = sympy.Symbol('t')
    b0, b1, b2, b3, b4 = sympy.symbols('b0 b1 b2 b3 b4')
    b5, b6, b7, b8, b9 = sympy.symbols('b5 b6 b7 b8 b9')
    x0, dx0, d2x0, d3x0, d4x0, d5x0, d6x0, d7x0, d8x0, d9x0 = sympy.symbols('x0 dx0 d2x0 d3x0 d4x0 d5x0 d6x0 d7x0 d8x0 d9x0')
    xk, dxk, d2xk, d3xk, d4xk, d5xk, d6xk, d7xk, d8xk, d9xk = sympy.symbols('xk dxk d2xk d3xk d4xk d5xk d6xk d7xk d8xk d9xk')

    #defining polinomial trajectory
    x = b0 + b1*t + b2*(t**2) + b3*(t**3) + b4*(t**4)\
         + b5*(t**5) + b6*(t**6) + b7*(t**7) + b8*(t**8) + b9*(t**9)


    #defining derivations of this polinomial
    dx = sympy.diff(x, t)
    d2x = sympy.diff(dx, t) #second derivation
    d3x = sympy.diff(d2x, t) #third derivation
    d4x = sympy.diff(d3x, t) #fourth derivation
    d5x = sympy.diff(d4x, t) #fifth derivation
    d6x = sympy.diff(d5x, t) #sixth derivation
    d7x = sympy.diff(d6x, t) #seventh derivation

    equations = [

        sympy.Eq(x0 - b0),
        sympy.Eq(dx0 - b1),
        sympy.Eq(d2x0 - 2*b2),
        sympy.Eq(d3x0 - 6*b3),
        sympy.Eq(d4x0 - 24*b4),
        sympy.Eq(xk - x),
        sympy.Eq(dxk - dx),
        sympy.Eq(d2xk - d2x),
        sympy.Eq(d3xk - d3x),
        sympy.Eq(d4xk - d4x),
    ]

    # coeffs = solve(equations, [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9])
    # for key, item in coeffs.iteritems(): print "\ncoeff ", key,": ", item



def publisher(newMsg, pub):

    
    time.sleep(0.5)
    rate = rospy.Rate(10) # 10hz
    #while not rospy.is_shutdown(): 
    print "tu sam"
    pub.publish(newMsg)
        #rate.sleep()

def calulating_trajectory(x, y, z, t, v_max, a_max):
    """
    Calculating trajectory.
    args:

        x: tuple (x_beginning, x_end)
        y: tuple (y_beginning, y_end)
        z: tuple (z_beginning, z_end)
        t: float (estimated time)
        v_max: float (desired maximum velocity of the drone)
        a_max: float (desired maximum acceleration of the drone)
    """
    newMsg = MultiDOFJointTrajectory()
    x0, x1 = x
    y0, y1 = y
    z0, z1 = z

    #defining coefficients for x, y, z axis
    x_coeffs = define_coeffs(x0, x1, t)
    y_coeffs = define_coeffs(y0, y1, t)
    z_coeffs = define_coeffs(z0, z1, t)
    
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

    a_real = np.zeros(len(t_disc))
    v_real = np.zeros(len(t_disc))

    #calculatng speed and acceleration for every time sample
    for idx, t in enumerate(t_disc):
        pp((idx, t))

        #position, speed and acceleration for x direction
        x[idx] = xb0 + xb1*t + xb2*(t**2) + xb3*(t**3) + xb4*(t**4) + xb5*(t**5) + xb6*(t**6) + xb7*(t**7) + xb8*(t**8) + xb9*(t**9)
        v_x[idx] = xb1 + 2*xb2*t + 3*xb3*t**2 + 4*xb4*t**3 + 5*xb5*t**4 + 6*xb6*t**5 + 7*xb7*t**6 + 8*xb8*t**7 + 9*xb9*t**8
        a_x[idx] = 2*xb2 + 6*xb3*t + 12*xb4*t**2 + 20*xb5*t**3 + 30*xb6*t**4 + 42*xb7*t**5 + 56*xb8*t**6 + 72*xb9*t**7

        #position, speed and acceleration for y direction
        y[idx] = yb0 + yb1*t + yb2*(t**2) + yb3*(t**3) + yb4*(t**4) + yb5*(t**5) + yb6*(t**6) + yb7*(t**7) + yb8*(t**8) + yb9*(t**9)
        v_y[idx] = yb1 + 2*yb2*t + 3*yb3*t**2 + 4*yb4*t**3 + 5*yb5*t**4 + 6*yb6*t**5 + 7*yb7*t**6 + 8*yb8*t**7 + 9*yb9*t**8
        a_y[idx] = 2*yb2 + 6*yb3*t + 12*yb4*t**2 + 20*yb5*t**3 + 30*yb6*t**4 + 42*yb7*t**5 + 56*yb8*t**6 + 72*yb9*t**7

        #position, speed and acceleration for z direction
        z[idx] = zb0 + zb1*t + zb2*(t**2) + zb3*(t**3) + zb4*(t**4) + zb5*(t**5) + zb6*(t**6) + zb7*(t**7) + zb8*(t**8) + zb9*(t**9)        
        v_z[idx] = zb1 + 2*zb2*t + 3*zb3*t**2 + 4*zb4*t**3 + 5*zb5*t**4 + 6*zb6*t**5 + 7*zb7*t**6 + 8*zb8*t**7 + 9*zb9*t**8
        a_z[idx] = 2*zb2 + 6*zb3*t + 12*zb4*t**2 + 20*zb5*t**3 + 30*zb6*t**4 + 42*zb7*t**5 + 56*zb8*t**6 + 72*zb9*t**7      

        v_real[idx] = math.sqrt(v_x[idx]**2 + v_y[idx]**2 + v_z[idx]**2)
        a_real[idx] = math.sqrt(a_x[idx]**2 + a_y[idx]**2 + a_z[idx]**2)

        # initialization of message components
        newPoint = MultiDOFJointTrajectoryPoint()

        newTransform = Transform()
        newQuaternion = Quaternion()
        newVelocities = Twist()
        newAccelerations = Twist()

        newTransform.translation.x = x[idx]
        newTransform.translation.y = y[idx]
        newTransform.translation.z = z[idx]

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

    v_real_max = np.amax(v_real)
    a_real_max = np.amax(a_real)

    Sv = v_real_max/v_max
    Sa = math.sqrt(a_real_max/a_max)

    if round(Sa, 2) == 1.0 or round(Sv, 2) == 1.0:
        return newMsg

    return (Sa, Sv)

def publishing_trajectory_between_two_points(point0, point1, v_max, a_max, pub):

    x0, y0, z0 = point0
    x1, y1, z1 = point1

    #calculating estimated t
    t = math.sqrt((x1-x0)**2 + (y1-y0)**2 + (z1-z0)**2)/v_max

    result = calulating_trajectory((x0, x1), (y0, y1), (z0, z1), t, v_max, a_max)
    while isinstance(result, tuple):
        Sa, Sv = result
        # print "Sa: ", Sa
        # print "Sv: ", Sv

        factor = max(Sa, Sv)
        t = factor*t

        result = calulating_trajectory((x0, x1), (y0, y1), (z0, z1), t, v_max, a_max)
    
    return result   
    #publisher(result, pub)

def multiple_points(points, v_max, a_max, pub):
    for i, point in enumerate(points):
        try:
            publishing_trajectory_between_two_points(point, points[i+1], v_max, a_max, pub)
        except IndexError:
            pass

def run():

    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('/firefly/command/trajectory', MultiDOFJointTrajectory, queue_size=10) # your node is publishing to the chatter topic usting the msg type String

    v_max = 3
    a_max = 1.5

    points = []
    with open("trajectory_points.txt", 'r+') as traj_file:
        for line in traj_file:
            point = eval(line.strip(os.linesep))
            points.append(point)


    #multiple_points(points, v_max, a_max, pub)
#    publishing_trajectory_between_two_points(x0, x1, y0, y1, z0, z1, t, v_max, a_max, pub)


if __name__ == "__main__":

    run()
    #calculate_coeffs()
