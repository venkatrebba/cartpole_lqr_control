#!/usr/bin/env python3
"""
LQR Controller for cart-pole problem
"""

import numpy as np
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
import control
import rospy
from scipy.signal import place_poles

class LQRController:
    def __init__(self):
        rospy.init_node('lqrcontroller', anonymous=True)
        self.pend_states_sub = rospy.Subscriber('/invpend/joint_states', JointState, self.state_check)
        self.join1_velocity_pub = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
        self.cart_postition = 0
        self.cart_velocity = 0
        self.pole_position = 0
        self.pole_velocity = 0
       
    def state_check(self, data):
        self.cart_postition = data.position[1]
        self.cart_velocity = data.velocity[1]
        self.pole_position = data.position[0]
        self.pole_velocity = data.velocity[0]

    def input(self,k):
        target = np.array([0,0,0,0])
        original = np.array([self.cart_postition, self.cart_velocity, self.pole_position, self.pole_velocity])
        u = np.dot(k,(target - original))
        self.join1_velocity_pub.publish(u)
        print("Force applied on cart : {} ".format(u) )
        

g = 9.8  # Gravity
l = 0.5  # Lenthg of pole
m = 2    # Mass of pole
M = 20   # Mass of cart

# Defining all matrices
A = np.matrix([[0,1,0,0], [0,0,(-12*m*g)/((13*M)+ m),0], [0,0,0,1],[0,0,(12*g*(M + m))/(l * ((13*M) + m)),0]])
B = np.matrix([ [0], [13 / ((13 * M) + m)], [0],  [-12 / (l * ((13 * M) + m))]])
Q = np.diag([1,1,10,100])
R = np.diag([0.01])


def getK():
    """
    Applying LQR controller
    """
    k,S,E = control.lqr(A,B,Q,R)
    return control.place(A, B, E)
    
    
if __name__ == '__main__':
    try:
        model = LQRController()
        k=getK()
        while not rospy.is_shutdown():
            model.input(k)
    except rospy.ROSInterruptException:
        pass




