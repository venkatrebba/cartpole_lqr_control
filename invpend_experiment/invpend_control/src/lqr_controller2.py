#!/usr/bin/env python3
import rospy
import numpy as np
import control as cs 
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

# Contrains are picked from Gazebo directly
mass_pole = 2 #mass of pole
Mass_cart = 20 #mass of cart
gravity = 9.8 #gravity
length = 0.5 #heigth of the pole

a = (-12 * mass_pole * gravity)/((13 * Mass_cart) + mass_pole)
a1 = (12 * gravity * (Mass_cart + mass_pole))/(length * ((13 * Mass_cart) + mass_pole))
b = 13 / ((13 * Mass_cart) + Mass_cart)
b1 = -12 / (length * ((13 * Mass_cart) + Mass_cart))

pc =np.array([0., 0., 0., 0.])  
finPose = np.array([0., 0., 0., 0.])



def blanace(K):
    rospy.init_node('LQR_bal')
    pose_pub = rospy.Publisher("/invpend/joint1_velocity_controller/command", Float64, queue_size=50)
    pose_messgae  = Float64()
    pose_messgae.data = -np.matmul(K,(pc-finPose))   
    print(pose_messgae.data)
    pose_pub.publish(pose_messgae)
    
    
def pAngle(angle):
    pc[2] = angle.process_value
    pc[3] = angle.process_value_dot
    
    
def pPose(Ppose):
    pc[0] = Ppose.position[1]
    pc[1] = Ppose.velocity[1]
    
    
if __name__ == '__main__':
    # A and B matrix from lecture
    A = np.matrix([ [0, 1, 0, 0],
                [0, 0, a, 0],
                [0, 0, 0, 1],
                [0, 0, a1, 0]
                ])
    B = np.matrix([ [0],
                [b],
                [0],
                [b1]
                ])


    Q = Q = np.diag( [1.,1.,1.,1.] ) *90

    R = np.diag( [0.001])
    K, S, E = cs.lqr( A, B, Q, R )
    theta_sub = rospy.Subscriber("/invpend/joint2_position_controller/state",JointControllerState, pAngle)
    Position_sub = rospy.Subscriber("/invpend/joint_states",JointState, pPose) 
    while not rospy.is_shutdown():
        blanace(K)
