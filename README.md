# cartpole_lqr_control

Execution instrictions

1 cd catkin_ws/src  
2. git clone https://github.com/venkatrebba/cartpole_lqr_control/  
3. cd catkin_ws  
4. catkin build  
5. chmod +x catkin_ws/src/cartpole_lqr_control/invpend_control/src/controller.py  
6. Run the below command in a terminal  
   roslaunch invpend_gazebo invpend_world.launch  
7. Run the below command in a new terminal  
    roslaunch invpend_control invpend_control.launch   
8. Run the below command in a new terminal  
    rosrun invpend_control controller.py  
   
