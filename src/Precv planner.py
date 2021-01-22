############################# Precv planner.py

rospy.init_node('Planner')
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
odom=Odometry()


robot_pose=None

lc_pose=None

rc_pose=None

robot_radius=400

threshold=10


saftey_distance_for_controller=400    #is radius

Maze_eqns=[(000,000,saftey_distance_for_controller)]  # x y r of obstacle
Maze=MazeMaker(10,[10000,10000],Maze_eqns)


pose_list=[]


goal_pose=[(4.5,4.5,1),(0,0,0,1)]

goal_change=False




#do update of these outside

# goal_list=RRT(Maze,Maze_eqns,[550,9550],[9500,500],800,800,10,[10000,10000],400)
# print(goal_list)









def UpdatePosesThread(threadName,delay):
    import tf
    print("Started Thread")
    listener=tf.TransformListener()
    global robot_pose
    global lc_pose
    global rc_pose
    

    #listener.waitForTransform("/headset", "/origin", rospy.Time(), rospy.Duration(4.0))
    while not rospy.is_shutdown():
        try:
            now=rospy.Time.now()
            listener.waitForTransform("/origin","/robot_frame", now, rospy.Duration(0.5))
            (trans,rot) = listener.lookupTransform('/origin','/robot_frame', now)
            # print(trans,rot)
            robot_pose=(trans,rot)




            now=rospy.Time.now()
            listener.waitForTransform("/origin","/left_controller",  now, rospy.Duration(0.5)) 
            (trans,rot) = listener.lookupTransform('/origin','/left_controller', now)
            # print(trans,rot)
            lc_pose=(trans,rot)




            now=rospy.Time.now()
            listener.waitForTransform("/origin","/right_controller",  now, rospy.Duration(0.5))
            (trans,rot) = listener.lookupTransform('/origin','/right_controller', now)
            # print(trans,rot)
            rc_pose=(trans,rot)















                




        except:
            pass
    print("THREAD ENDED")





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
    #get_caller_id(): Get fully resolved name of local node
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s")#, message.data)
    global odom

    goal_change=True
    
    ##############

    #  Get A B

    ###############
    
    #print("callback executed")
    #print(odom.pose)






def detectGoalchange():
    global goal_change

    if goal_change:
        goal_change=False
        return True
    return False


def ConvertPoseToMapCoordinates(robot_pose):
    # print("In convert ",robot_pose)
    x=robot_pose[0][0]
    y=robot_pose[0][1]

    return (5000+(x*1000), 5000+ (-y*1000))


def CheckObstruction(Maze_eqns,pose_list,current_index):
    #check for pts ahead of robot pose
    pose_list_sub=pose_list[current_index:]

    for i in range(1,len(pose_list)):
        p1=pose_list[i-1]
        p2=pose_list[i]
        point_=Node([p1[0],p1[1]])
        point_neighbour_=Node([p2[0],p2[1]])
        obs = pathinObstacleFast(point_,point_neighbour_,Maze_eqns,robot_radius )
        if obs==True:
            return True

    return False



def EulerDistance(robot_pose_map,waypoint_map):
    return np.sqrt((robot_pose_map[0]-waypoint_map[0])**2 + (robot_pose_map[1]-waypoint_map[1])**2)

def NormalDistance(robot_pose_map,current_index):

    global pose_list



    x1,y1=pose_list[current_index-1]
    x2,y2=pose_list[current_index]

    x,y=robot_pose_map

    a,b,c=giveabcofLine(x1,y1,x2,y2)

    d= np.abs( (a*x)+ (b*y) + c )/np.sqrt(a**2 + b**2)


    print("abc is ",a,b,c, "normal dis is ",d)

    return d













    













def main():
    


    thread.start_new_thread(UpdatePosesThread,("thread1",0) )

    global goal_pose
    global lc_pose
    global rc_pose
    global pose_list
    global goal_change
    global robot_pose
    global threshold
    current_index=0

    while True:
        pass
        # Get robot pose and controller pose

        


        if robot_pose is None:
            print("robot pose is None")
            continue

        if lc_pose is None:
            print("lc pose is None")
            continue

        if rc_pose is None:
            rc_pose=([0,0,0],[0,0,0,1])



        if goal_pose is None:
            goal_pose=robot_pose  #########so that robot does not go away running


        ##############################       remapping update only over here        

        #update Maze and Maze_eqns accordingly  
        robot_pose_map=ConvertPoseToMapCoordinates(robot_pose)
        

        left_controller_pose_map=ConvertPoseToMapCoordinates(lc_pose)
        right_controller_pose_map=ConvertPoseToMapCoordinates(rc_pose)

        goal_pose_map=ConvertPoseToMapCoordinates(goal_pose)

        Maze_eqns=[(left_controller_pose_map[0],left_controller_pose_map[1],saftey_distance_for_controller)]
        Maze=MazeMaker(10,[10000,10000],Maze_eqns)


        


        print("Map update done:",current_index)


        

        #  Publish current robot pose to robot so that it stopos while remapping in progress
        #  maybe only give if obstruction far ahead

        if len(pose_list)==0   or   detectGoalchange()    or    NormalDistance(robot_pose_map,current_index)>robot_radius:
            
            print("Finding Path from ",[robot_pose_map[0],robot_pose_map[1]], " to this ",[goal_pose_map[0],goal_pose_map[1]]) 
            pose_list=RRT(Maze,Maze_eqns,[robot_pose_map[0],robot_pose_map[1]],[goal_pose_map[0],goal_pose_map[1]],800,800,10,[10000,10000],400)

            #   Publish pose_list[0] to  along with goal angle   maybe give angle wrt heading direction follower.cpp
            current_index=0
            


        else:

            obstruction=CheckObstruction(Maze_eqns,pose_list,current_index)

            if obstruction==True:
                print("obstruction found")
                pose_list=[]
                continue

            
        print("Distance away from current_index ",EulerDistance(robot_pose_map,pose_list[current_index]))

        if EulerDistance(robot_pose_map,pose_list[current_index])<300 and current_index!=len(pose_list)-1:
            current_index+=1



        print("Drawing from Pose_list ", len(pose_list)," current_index:",current_index)
        # print("mz shape is ",Maze.shape)
        cv2.line(Maze,(0,0)  ,(100,0) ,(0,0,255),10)

        cv2.line(Maze,(0,0)  ,(0,100) ,(255,0,0),10)


        origin_O=[(0,0,0),(0,0,0,1)]
        origin_X=[(1,0,0),(0,0,0,1)]
        origin_Y=[(0,1,0),(0,0,0,1)]


        convO=ConvertPoseToMapCoordinates(origin_O)
        convX=ConvertPoseToMapCoordinates(origin_X)
        convY=ConvertPoseToMapCoordinates(origin_Y)

        eul=quaternion_to_euler(robot_pose[1][0],robot_pose[1][1],robot_pose[1][2],robot_pose[1][3])
        # print(eul)
        theta=eul[0]*3.14159/180.0
        headingXx=int(robot_pose_map[0] + 1000*np.cos(theta))
        headingXy=int(robot_pose_map[1] + 1000*np.sin(theta))


        headingYx=int(robot_pose_map[0] + 1000*np.sin(theta))
        headingYy=int(robot_pose_map[1] - 1000*np.cos(theta))

        print(robot_pose_map, headingXx,headingXy,headingYx,headingYy,theta)

        cv2.circle(Maze,(int(robot_pose_map[0]/threshold),int(robot_pose_map[1]/threshold)),int(robot_radius/threshold),(0,255,0),-1)

        cv2.line(Maze,(int(convO[0]/threshold),int(convO[1]/threshold))  ,(int(convX[0]/threshold),int(convX[1]/threshold) ) ,(0,0,255),10)

        cv2.line(Maze,(int(convO[0]/threshold),int(convO[1]/threshold))  ,(int(convY[0]/threshold),int(convY[1]/threshold) ) ,(255,0,0),10)



        cv2.line(Maze,(int(robot_pose_map[0]/threshold),int(robot_pose_map[1]/threshold))  ,(int(headingXx/threshold),int(headingXy/threshold) ) ,(0,0,255),10)
        cv2.line(Maze,(int(robot_pose_map[0]/threshold),int(robot_pose_map[1]/threshold))  ,(int(headingYx/threshold),int(headingYy/threshold) ) ,(255,0,0),10)





        for p in range(current_index-1,len(pose_list)-1):
            # print((int(pose_list[p][0]/threshold),int(pose_list[p][1]/threshold))  ,(int(pose_list[p+1][0]/threshold),int(pose_list[p+1][1]/threshold)))
            cv2.line(Maze,(int(pose_list[p][0]/threshold),int(pose_list[p][1]/threshold))  ,(int(pose_list[p+1][0]/threshold),int(pose_list[p+1][1]/threshold)) ,(250,0,255),10)

        resizedMaze=cv2.resize(Maze,(300,300),interpolation = cv2.INTER_AREA)
        cv2.imshow("mazePlanner",resizedMaze)

        if cv2.waitKey(10) & 0xFF == ord('q'):
                break



            ################################

    cv2.destroyAllWindows()

















        # print(robot_pose)
        #rospy.sleep(0.0002)

    # listener=tf.TransformListener()
    # listener.waitForTransform("/headset", "/origin", rospy.Time(), rospy.Duration(4.0))
    # while not rospy.is_shutdown():
        
    #         now=rospy.Time.now()
    #         listener.waitForTransform("/headset", "/origin", now, rospy.Duration(0.5))

        
    #         (trans,rot) = listener.lookupTransform('/headset', '/origin',now)
    #         print(trans,rot)
        

        

      

