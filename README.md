# ROS_Roboclaw_C-_integration_with_position-control_PID_node
1. To run this project first connect and configure the roboclaw using ion motion studio available for windows
2. make sure encoders and motors work perfectly
3. install github repository and unzip it into catkin_ws/src
4. Make the files using catkin_make
5. go to roboclaw/launch/roboclaw_single.launch
6. change the parameters according to the ones you set in ion motion studio
7. then go to the command line and do $ roslaunch roboclaw roboclaw_single.launch
8. If it does not run correctly reset/check the parameters of the launch file again
9. Set the parameters of kp,ki,kd in the motor_controller motor_controller_with_subscriber_topic.cpp according to your motor and controllers
9. if it runs then perform this command in another terminal $ rosrun motor_controller motor_controller_with_subscriber_topic.cpp
10. S rostopic pub /position <press tab 2 times> <press tab 2 times> you will get a option to enter position_1:
                                                                                                    position_2:
                                                                                                    position_3:
11.Enter some values to make the motor go to that position in encoder steps.
                                                                                                                                                                              
                                                                                                                                                                                                      
