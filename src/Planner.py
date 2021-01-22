#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import roslib
import geometry_msgs.msg

import multiprocessing

from rrt import *


import thread

import math

class Planner():
    def __init__(self):
        rospy.init_node('Planner')
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom=Odometry()


        self.robot_pose=None

        self.lc_pose=None

        self.rc_pose=None

        self.robot_radius=400

        self.threshold=10


        self.saftey_distance_for_controller=400    #is radius

        self.Maze_eqns=[(000,000,self.saftey_distance_for_controller)]  # x y r of obstacle
        self.Maze=MazeMaker(10,[10000,10000],self.Maze_eqns)


        self.pose_list=[]


        self.goal_pose=[(0,0,1),(0,0,0,1)]

        self.goal_change=False

        self.drawingThreadStarted=False


        self.current_index=0

        self.tracker_pose_subsriber=rospy.Subscriber("/trackerPose", PoseStamped, self.trackerPosecallback)
        self.lc_pose_subcriber=rospy.Subscriber("/leftControllerPose", PoseStamped, self.LCPosecallback)
        self.rc_pose_subscriber=rospy.Subscriber("/rightControllerPose", PoseStamped, self.RCPosecallback)
        self.robot_pose_subscriber=rospy.Subscriber("/robot_pose", Pose, self.RobotPosecallback)


        f = open(r"calibration2.txt", "r")
        transformation_tracker_robot_str=(f.read())
        transformation_tracker_robot_str=transformation_tracker_robot_str[1:]
        transformation_tracker_robot_str=transformation_tracker_robot_str[0:-2]
        strs=transformation_tracker_robot_str.split(',')
        # print(strs)
        self.transformation_tracker_robot=[]
        for s in strs:
            self.transformation_tracker_robot.append(float(s))

        # print(transformation_tracker_robot)
        
       
        
       

    def rot2quat(self,R):
        """ROT2QUAT - Transform Rotation matrix into normalized quaternion.
        
        Usage: q = rot2quat(R)
        
        Input:
        R - 3-by-3 Rotation matrix
        
        Output:
        q - 4-by-1 quaternion, with form [w x y z], where w is the scalar term.
        """
        # By taking certain sums and differences of the elements
        # of R we can obtain all products of pairs a_i a_j with
        # i not equal to j. We then get the squares a_i^2 from
        # the diagonal of R.
        a2_a3 = (R[0,1] + R[1,0]) / 4
        a1_a4 = (R[1,0] - R[0,1]) / 4
        a1_a3 = (R[0,2] - R[2,0]) / 4
        a2_a4 = (R[0,2] + R[2,0]) / 4
        a3_a4 = (R[1,2] + R[2,1]) / 4
        a1_a2 = (R[2,1] - R[1,2]) / 4
      
        D = np.array([[+1, +1, +1, +1],
                   [+1, +1, -1, -1],
                   [+1, -1, +1, -1],
                   [+1, -1, -1, +1]]) * 0.25
                   
        aa = np.dot(D, np.r_[np.sqrt(np.sum(R**2) / 3), np.diag(R)])
      
        # form 4 x 4 outer product a \otimes a:
        a_a = np.array([[aa[0], a1_a2, a1_a3, a1_a4],
                     [a1_a2, aa[1], a2_a3, a2_a4],
                     [a1_a3, a2_a3, aa[2], a3_a4],
                     [a1_a4, a2_a4, a3_a4, aa[3]]])
        
        # use rank-1 approximation to recover a, up to sign.
        U, S, V = np.linalg.svd(a_a)
        q = U[:, 0] 
        # q = _n.dot(_math.sqrt(S[0]), U[:, 0]) # Use this if you want unnormalized quaternions 
        
        return q





    def LCPosecallback(self,message):
    

        
        temp=message.pose

        self.lc_pose=[(temp.position.x,temp.position.y,temp.position.z),(temp.orientation.x,temp.orientation.y,temp.orientation.z,temp.orientation.w)]



    def RCPosecallback(self,message):
    
        temp=message.pose
        self.rc_pose=[(temp.position.x,temp.position.y,temp.position.z),(temp.orientation.x,temp.orientation.y,temp.orientation.z,temp.orientation.w)]

    def RobotPosecallback(self,message):
    
        temp=message
        self.robot_pose=[(temp.position.x,temp.position.y,temp.position.z),(temp.orientation.x,temp.orientation.y,temp.orientation.z,temp.orientation.w)]


    def trackerPosecallback(self,message):
    
        temp=message.pose

        ###################
        # convert to robot pose 
        ###################

        # print("Tracker pose is",message.pose)
        # print("##########################")
        # trackerpose=message.pose
        # rotn_matrix_tracker=self.quat2rot((temp.orientation.x,temp.orientation.y,temp.orientation.z,temp.orientation.w))
        # # print(rotn_matrix_tracker)

        # Htracker=np.eye(4)
        # Htracker[0:3,0:3]=rotn_matrix_tracker
        # Htracker[0:3,3]=np.array([temp.position.x,temp.position.y,temp.position.z])
        # # print(Htracker)



        # orirpy=(3.14159/2,0,3.14159/2)
        # rotn_matrix_tracker_robot=self.rpy2rot(orirpy)
        # # rotn_matrix_robot_tracker=self.quat2rot((self.transformation_tracker_robot[3],self.transformation_tracker_robot[4],self.transformation_tracker_robot[5],self.transformation_tracker_robot[6]))
        # H_tracker_robot=np.eye(4)
        # H_tracker_robot[0:3,0:3]=rotn_matrix_tracker_robot
        # H_tracker_robot[0:3,3]=np.array([self.transformation_tracker_robot[0],self.transformation_tracker_robot[1],self.transformation_tracker_robot[2]])

        # # print("Transform is from txt file")
        # # print(Hrobot_tracker[0:3,3],self.rot2quat(Hrobot_tracker[0:3,0:3]))
        # # print("Transform is from tf")

        # # print("Tracker pose is", trackerpose)
        # r_p=np.matmul((Htracker),(H_tracker_robot))
        # # print("pos ",r_p[0:3,3])
        # # print("quat ",self.rot2quat(r_p[0:3,0:3]))
        # pos=r_p[0:3,3]
        # quat=self.rot2quat(r_p[0:3,0:3])


        # # assert 1==0

        # self.robot_pose=[(pos[0],pos[1],pos[2]),(quat[0],quat[1],quat[2],quat[3])]
        # print(self.robot_pose)


    def quaternion_rotation_matrix(self,Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
     
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
     
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                 This rotation matrix converts a point in the local reference 
                 frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
         
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
         
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
         
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
         
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])
                                
        return rot_matrix


    def quat2rot(self,q):
        """QUAT2ROT - Transform quaternion into rotation matrix
        
        Usage: R = quat2rot(q)
        
        Input:
        q - 4-by-1 quaternion, with form [w x y z], where w is the scalar term.
        
        Output:
        R - 3-by-3 Rotation matrix
        """    
        
        q = q / np.linalg.norm(q)
             
        w = q[0]; x = q[1];  y = q[2];  z = q[3]
        
        x2 = x*x;  y2 = y*y;  z2 = z*z;  w2 = w*w
        xy = 2*x*y;  xz = 2*x*z;  yz = 2*y*z
        wx = 2*w*x;  wy = 2*w*y;  wz = 2*w*z
        
        R = np.array([[w2+x2-y2-z2, xy-wz, xz+wy],
                   [xy+wz, w2-x2+y2-z2, yz-wx],
                   [xz-wy, yz+wx, w2-x2-y2+z2]])
        return R


    def rpy2rot(self,rpy):
        """ROT2RPY - Transform [roll, pitch, yaw] to rotation matrix
        Rotations are around fixed-axes. Roll is around x-axis, pitch is around 
        y-axis, yaw is around z-axis. 
        Rotations' order is: first roll, then pitch, then yaw.
        
        Usage: R = rpy2rot(rpy)
        
        Input:
        rpy - 3-by-1 array with [roll, pitch, yaw] values
        
        Output:
        R - 3-by-3 rotation matrix
        """
        r = rpy[0]; p = rpy[1]; y = rpy[2]
        s = math.sin
        c = math.cos
         
        R = np.array([
            [c(y)*c(p),   c(y)*s(p)*s(r)-s(y)*c(r),   c(y)*s(p)*c(r)+s(y)*s(r)],
            [s(y)*c(p),   s(y)*s(p)*s(r)+c(y)*c(r),   s(y)*s(p)*c(r)-c(y)*s(r)],
            [-s(p),       c(p)*s(r),                  c(p)*c(r)]])

        return R


    def quaternion_to_euler(self,x, y, z, w):

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


    def UpdatePosesThread(self,threadName,delay):
        import tf
        print("Started Thread")
        listener=tf.TransformListener()
        
        

        #listener.waitForTransform("/headset", "/origin", rospy.Time(), rospy.Duration(4.0))
        while not rospy.is_shutdown():
            try:
                now=rospy.Time.now()
                listener.waitForTransform("/origin","/robot_frame", now, rospy.Duration(0.5))
                (trans,rot) = listener.lookupTransform('/origin','/robot_frame', now)
                # print(trans,rot)
                self.robot_pose=(trans,rot)




                # now=rospy.Time.now()
                # listener.waitForTransform("/origin","/left_controller",  now, rospy.Duration(0.5)) 
                # (trans,rot) = listener.lookupTransform('/origin','/left_controller', now)
                # # print(trans,rot)
                # self.lc_pose=(trans,rot)




                # now=rospy.Time.now()
                # listener.waitForTransform("/origin","/right_controller",  now, rospy.Duration(0.5))
                # (trans,rot) = listener.lookupTransform('/origin','/right_controller', now)
                # # print(trans,rot)
                # self.rc_pose=(trans,rot)



                # left_controller_pose_map=self.ConvertPoseToMapCoordinates(self.lc_pose)
                





            except:
                pass



        print("THREAD ENDED")


    


    



    def odom_callback(self,message):
        #get_caller_id(): Get fully resolved name of local node
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s")#, message.data)
        

        self.goal_change=True
        
        ##############

        #  Get A B

        ###############
        
        #print("callback executed")
        #print(odom.pose)


    def detectGoalchange(self):
        

        if self.goal_change:
            self.goal_change=False
            return True
        return False


    def ConvertPoseToMapCoordinates(self,robot_pose):
        # print("In convert ",robot_pose)
        x=robot_pose[0][0]
        y=robot_pose[0][1]

        return (5000+(x*1000), 5000+ (-y*1000))



    def ConvertMapCoordinatesToPose(self,robot_pose):
        # print("In convert ",robot_pose)
        x=robot_pose[0]
        y=robot_pose[0]
        x_dash=((x-5000)/1000)  
        y_dash=(-y+5000)/1000

        p=Pose2D()

        p.x=x_dash
        p.y=y_dash

        ###############Give angle as heading angle


        return 


    def CheckObstruction(self,current_index):
        #check for pts ahead of robot pose
        pose_list_sub=self.pose_list[current_index:]

        for i in range(1,len(self.pose_list)):
            p1=self.pose_list[i-1]
            p2=self.pose_list[i]
            point_=Node([p1[0],p1[1]])
            point_neighbour_=Node([p2[0],p2[1]])
            obs = pathinObstacleFast(point_,point_neighbour_,self.Maze_eqns,self.robot_radius )
            if obs==True:
                return True

        return False



    def EulerDistance(self,robot_pose_map,waypoint_map):
        return np.sqrt((robot_pose_map[0]-waypoint_map[0])**2 + (robot_pose_map[1]-waypoint_map[1])**2)

    def NormalDistance(self,robot_pose_map,current_index):

        global pose_list



        x1,y1=self.pose_list[current_index-1]
        x2,y2=self.pose_list[current_index]

        x,y=robot_pose_map

        a,b,c=giveabcofLine(x1,y1,x2,y2)

        d= np.abs( (a*x)+ (b*y) + c )/np.sqrt(a**2 + b**2)


        print("abc is ",a,b,c, "normal dis is ",d)

        return d

    def drawingThread(self,threadName,delay):
        while True:

            robot_pose_map=self.ConvertPoseToMapCoordinates(self.robot_pose)




            # print("Drawing from Pose_list ", len(self.pose_list)," current_index:",self.current_index)
            # print("mz shape is ",Maze.shape)
            cv2.line(self.Maze,(0,0)  ,(100,0) ,(0,0,255),10)

            cv2.line(self.Maze,(0,0)  ,(0,100) ,(255,0,0),10)


            origin_O=[(0,0,0),(0,0,0,1)]
            origin_X=[(1,0,0),(0,0,0,1)]
            origin_Y=[(0,1,0),(0,0,0,1)]


            convO=self.ConvertPoseToMapCoordinates(origin_O)
            convX=self.ConvertPoseToMapCoordinates(origin_X)
            convY=self.ConvertPoseToMapCoordinates(origin_Y)

            eul=self.quaternion_to_euler(self.robot_pose[1][0],self.robot_pose[1][1],self.robot_pose[1][2],self.robot_pose[1][3])
            # print(eul)
            theta=eul[0]*3.14159/180.0
            headingXx=int(robot_pose_map[0] + 1000*np.cos(theta))
            headingXy=int(robot_pose_map[1] + 1000*np.sin(theta))


            headingYx=int(robot_pose_map[0] + 1000*np.sin(theta))
            headingYy=int(robot_pose_map[1] - 1000*np.cos(theta))

            # print(robot_pose_map, headingXx,headingXy,headingYx,headingYy,theta)

            cv2.circle(self.Maze,(int(robot_pose_map[0]/self.threshold),int(robot_pose_map[1]/self.threshold)),int(self.robot_radius/self.threshold),(0,255,0),-1)

            cv2.line(self.Maze,(int(convO[0]/self.threshold),int(convO[1]/self.threshold))  ,(int(convX[0]/self.threshold),int(convX[1]/self.threshold) ) ,(0,0,255),10)

            cv2.line(self.Maze,(int(convO[0]/self.threshold),int(convO[1]/self.threshold))  ,(int(convY[0]/self.threshold),int(convY[1]/self.threshold) ) ,(255,0,0),10)



            cv2.line(self.Maze,(int(robot_pose_map[0]/self.threshold),int(robot_pose_map[1]/self.threshold))  ,(int(headingXx/self.threshold),int(headingXy/self.threshold) ) ,(0,0,255),10)
            cv2.line(self.Maze,(int(robot_pose_map[0]/self.threshold),int(robot_pose_map[1]/self.threshold))  ,(int(headingYx/self.threshold),int(headingYy/self.threshold) ) ,(255,0,0),10)





            for p in range(self.current_index-1,len(self.pose_list)-1):
                # print((int(pose_list[p][0]/threshold),int(pose_list[p][1]/threshold))  ,(int(pose_list[p+1][0]/threshold),int(pose_list[p+1][1]/threshold)))
                cv2.line(self.Maze,(int(self.pose_list[p][0]/self.threshold),int(self.pose_list[p][1]/self.threshold))  ,(int(self.pose_list[p+1][0]/self.threshold),int(self.pose_list[p+1][1]/self.threshold)) ,(250,0,255),10)

            

        print("Drawing THREAD ENDED")


    def main(self):
        


        # thread.start_new_thread(self.UpdatePosesThread,("thread1",0) )
        self.current_index=0
       

        while True:
            pass
            # Get robot pose and controller pose

           


            if self.robot_pose is None:
                # print("robot pose is None")
                rospy.sleep(0.02)
                continue

            if self.lc_pose is None:
                # print("lc pose is None")
                rospy.sleep(0.02)
                continue

            if self.rc_pose is None:
                rospy.sleep(0.02)
                self.rc_pose=([0,0,0],[0,0,0,1])



            if self.goal_pose is None:
                self.goal_pose=robot_pose  #########so that robot does not go away running


            ##############################       remapping update only over here


            if self.drawingThreadStarted==False:
                thread.start_new_thread(self.drawingThread,("drawingthread1",0) )
                self.drawingThreadStarted=True


            #update Maze and Maze_eqns accordingly  
            robot_pose_map=self.ConvertPoseToMapCoordinates(self.robot_pose)
            

            left_controller_pose_map=self.ConvertPoseToMapCoordinates(self.lc_pose)
            right_controller_pose_map=self.ConvertPoseToMapCoordinates(self.rc_pose)

            goal_pose_map=self.ConvertPoseToMapCoordinates(self.goal_pose)

            

            self.Maze_eqns=[(left_controller_pose_map[0],left_controller_pose_map[1],self.saftey_distance_for_controller)]
            self.Maze=MazeMaker(10,[10000,10000],self.Maze_eqns)
            


            print("Map update done:",self.current_index, self.robot_pose)


            

            #  Publish current robot pose to robot so that it stopos while remapping in progress
            #  maybe only give if obstruction far ahead

            if len(self.pose_list)==0   or   self.detectGoalchange()    or    self.NormalDistance(robot_pose_map,self.current_index)>self.robot_radius:
                
                print("Finding Path from ",[robot_pose_map[0],robot_pose_map[1]], " to this ",[goal_pose_map[0],goal_pose_map[1]]) 
                
                if self.EulerDistance(goal_pose_map,left_controller_pose_map)>self.robot_radius:
                    self.pose_list=RRT(self.Maze,self.Maze_eqns,[robot_pose_map[0],robot_pose_map[1]],[goal_pose_map[0],goal_pose_map[1]],800,800,10,[10000,10000],400)
                else:
                    print("Cannot do path planning, obstacle very near goal")
                    pose_list=[]
                    continue


                #   Publish pose_list[0] to  along with goal angle   maybe give angle wrt heading direction follower.cpp
                self.current_index=0
                


            else:

                obstruction=self.CheckObstruction(self.current_index)

                if obstruction==True:
                    print("obstruction found")
                    self.pose_list=[]
                    continue

                
            print("Distance away from current_index ",self.EulerDistance(robot_pose_map,self.pose_list[self.current_index]))

            if self.EulerDistance(robot_pose_map,self.pose_list[self.current_index])<300 and self.current_index!=len(self.pose_list)-1:
                self.current_index+=1


            if(EulerDistance(robot_pose_map,self.pose_list[self.current_index])<robot_radius):
                Publish(self.pose_list[self.current_index])

            else:
                Publish(robot_pose)

            resizedMaze=cv2.resize(self.Maze,(300,300),interpolation = cv2.INTER_AREA)
            cv2.imshow("mazePlanner",resizedMaze)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                    break

            rospy.sleep(0.0002)




                ################################

        cv2.destroyAllWindows()







if __name__ == '__main__':
   
    # main()
    p= Planner()
    p.main() 