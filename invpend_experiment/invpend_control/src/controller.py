#!/usr/bin/env python3

import numpy as np
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point
#import controlpy
import control
import rospy
from scipy.signal import place_poles


class controllerpole:
    def __init__(self):
        rospy.init_node('controllerpole', anonymous=True)
        
        self.sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.state_check)
        self.pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=10)
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
       
    def state_check(self, data):
        self.pos_cart = data.position[1]
        self.vel_cart = data.velocity[1]
        self.pos_pole = data.position[0]
        self.vel_pole = data.velocity[0]


    def input(self,k):
        desired=np.array([0,0,0,0])
        actual=np.array([self.pos_cart,self.vel_cart,self.pos_pole,self.vel_pole])
        u=np.dot(k,(desired - actual))
        self.pub_vel_cmd.publish(u)
        print("force applied on cart : {} ".format(u) )
        
g = 9.8
l = 0.5
m = 2
M = 20
A = np.matrix([[0,1,0,0], [0,0,(-12*m*g)/((13*M)+ m),0], [0,0,0,1],[0,0,(12*g*(M + m))/(l * ((13*M) + m)),0]])

#A = np.matrix([[0,1,0,0], [0,0,(-12*m*g)/((13*M)+ m),0], [0,0,0,1],[0,0,(12*g*(M + m))/(l * ((13*M) + m)),0]])
B = np.matrix([ [0], [13 / ((13 * M) + m)], [0],  [-12 / (l * ((13 * M) + m))]])

def K(Q = np.diag([1,1,10,100]), R = np.diag([0.01])):
    k,S,E = control.lqr(A,B,Q,R)
    #k,S,E = controlpy.synthesis.controller_lqr(A,B,Q,R)
    #return place_poles(A, B, E).gain_matrix
    #return k,S,E
    return (control.place(A, B, E))
    


if __name__ == '__main__':
    try:
        model = controllerpole()
        k=K()
        while not rospy.is_shutdown():
            model.input(k)
    except rospy.ROSInterruptException:
        pass




