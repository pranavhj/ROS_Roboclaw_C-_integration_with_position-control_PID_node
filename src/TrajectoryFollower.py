#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from withRPMs_new_maze import*

velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

odom=Odometry()


def quaternion_to_euler(x, y, z, w):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw*180/3.142, pitch*180/3.142, roll*180/3.142]



def odom_callback(message):
    
    global odom
    
    odom=message
    
    








def go_to_point(goal):
    goal_flag=0
    position=odom.pose.pose.position
    angles=odom.pose.pose.orientation
    angles_euler=quaternion_to_euler(angles.x,angles.y,angles.z,angles.w)
    #print(np.sqrt(np.square(goal[0]-position.x)+np.square(goal[1]-position.y)))
    goal_k=0.015
    
    while np.sqrt(np.square(goal[0]-position.x)+np.square(goal[1]-position.y))>0.05:
        

        position=odom.pose.pose.position
        angles=odom.pose.pose.orientation
        angles_euler=quaternion_to_euler(angles.x,angles.y,angles.z,angles.w)
        #print("gotowhile")
        #print(angles_euler[0])
        rospy.sleep(0.01)
        angles_euler[0]=angles_euler[0]+180
        goal_angle=180.0/3.142*np.arctan2(goal[1]-position.y,goal[0]-position.x)
        goal_angle=goal_angle+180

        
        if abs(goal_angle-angles_euler[0])>250:
            goal_k=-0.001
        else:
            goal_k=0.015
        velocity_msg_=Twist()
        print([goal_angle,angles_euler[0]])#,position.x,position.y,angles_euler[0]])
        #print(np.sqrt(np.square(goal[0]-position.x)+np.square(goal[0]-position.y)))
        #print([velocity_msg_.linear.x,velocity_msg_.angular.z])
        
        if abs(goal_angle-angles_euler[0])>35    and    abs(goal_angle-angles_euler[0])<305 :
            
            while abs(goal_angle-angles_euler[0])>3:
                position=odom.pose.pose.position
                angles=odom.pose.pose.orientation
                angles_euler=quaternion_to_euler(angles.x,angles.y,angles.z,angles.w)
                #print("gotowhile")
                angles_euler[0]=angles_euler[0]+180
                goal_angle=180.0/3.142*np.arctan2(goal[1]-position.y,goal[0]-position.x)
                goal_angle=goal_angle+180
                
                if abs(goal_angle-angles_euler[0])>250:
                    goal_k=-0.001
                else:
                    goal_k=0.015
                velocity_msg_.angular.z=(goal_angle-angles_euler[0])*goal_k

                #if abs(velocity_msg_.angular.z)>0.5:
                    #velocity_msg_.angular.z=0.5*abs(velocity_msg_.angular.z)/velocity_msg_.angular.z
                
                velocity_msg_.linear.x=0
                velocity_publisher.publish(velocity_msg_)
                rospy.sleep(0.01)
                print("Angle While  ",goal_angle,angles_euler[0])

        else:
            #velocity_msg_.angular.z=0

            speed=np.sqrt(np.square(goal[0]-position.x)+np.square(goal[0]-position.y))
            if speed>0.2:
                speed=0.2

            velocity_msg_.linear.x=speed
            if abs(goal_angle-angles_euler[0])>250:
                goal_k=-0.001
            else:
                goal_k=0.015
            velocity_msg_.angular.z=(goal_angle-angles_euler[0])*(goal_k)

            velocity_publisher.publish(velocity_msg_)
            rospy.sleep(0.01)
      
        flag=0
        if np.sqrt(np.square(goal[0]-position.x)+np.square(goal[0]-position.y))<0.05:
            flag=1
        
        
        

    #return velocity_msg_   This is follower.cpp




def talker():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    


    #we need to initialize the node
    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node 
    goal_list=maze_solver_Astar()
    #print(goal_list)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.init_node('moveto', anonymous=True)

    #set the loop rate
    print("Execution Started")
    velocity_msg=Twist()
    velocity_msg.linear.x=-0.0
    velocity_msg.linear.y=-0.0
    velocity_msg.angular.z=0.0

    rate = rospy.Rate(1) # 1hz
    
    goal_list.reverse()
    goal_list.pop(0)
    
    for p in range(len(goal_list)):
        goal_list[p][0]=-(goal_list[p][0]-5000.0)/1000.0
        goal_list[p][1]=(goal_list[p][1]-5000.0)/1000.0


    print(goal_list)
    i=0
    j=0
    while not rospy.is_shutdown():
               
    
        

        

        
        if j>0:

            if goal_list[j][0]<goal_list[j-1][0]:
        
                go_to_point(goal_list[j])
            

        

        
        j=j+1

        velocity_publisher.publish(velocity_msg)
        if j==len(goal_list)-1:
            break
        rospy.sleep(0.1)
        i=i+1

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        velocity_msg=Twist()
        velocity_msg.linear.x=0.0
        velocity_publisher.publish(velocity_msg)

        pass