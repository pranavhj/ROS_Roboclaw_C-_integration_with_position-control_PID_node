
# -*- coding: utf-8 -*-
"""
Created on Sun Feb  2 12:30:54 2020
111111
@author: prana
"""
import numpy as np
from random import seed
from random import randint
import matplotlib.pyplot as plt
import time
import cv2
import math
import copy


class Node():
    def __init__(self,co_ord):
        self.co_ord=co_ord
        self.neighbours=[]
        self.obstacle=None
        self.cost=0
        self.cost_togo=0




def findDistance(p1,p2):
    return np.sqrt(np.square(p1.co_ord[0]-p2.co_ord[0])+np.square(p1.co_ord[1]-p2.co_ord[1]))


def MazeMaker(threshold, block_dimensions,Maze_eqns):
    #r=66
    #L=354
    frame= np.zeros( (int(block_dimensions[0]/threshold)+3,int(block_dimensions[1]/threshold)+3,3), np.uint8 )
    
    # c1=np.array([[int(250/threshold),int(4250/threshold)],[int(1750/threshold),int(4250/threshold)],[int(1750/threshold),int(5750/threshold)],[int(250/threshold),int(5750/threshold)]])
    # c2=np.array([[int(2250/threshold),int(1250/threshold)],[int(3750/threshold),int(1250/threshold)],[int(3750/threshold),int(2750/threshold)],[int(2250/threshold),int(2750/threshold)]])
    # c3=np.array([[int(8250/threshold),int(4250/threshold)],[int(9750/threshold),int(4250/threshold)],[int(9750/threshold),int(5750/threshold)],[int(8250/threshold),int(5750/threshold)]])
    
    #cv2.circle(frame, (int(goal[0]/threshold),int(goal[1]/threshold)), 10 , (255,255,255))
    for cir in Maze_eqns:
        cv2.circle(frame,(int(cir[0]/threshold),int(cir[1]/threshold)),int(cir[2]/threshold),(255,255,255),-1)
    # cv2.circle(frame,(int(5000/threshold),int(5000/threshold)),int(1000/threshold),(255,255,255),-1)
    # cv2.circle(frame,(int(7000/threshold),int(2000/threshold)),int(1000/threshold),(255,255,255),-1)
    # cv2.circle(frame,(int(7000/threshold),int(8000/threshold)),int(1000/threshold),(255,255,255),-1)
    
    # cv2.drawContours(frame,[c1],-1,(255,255,255),-1)
    # cv2.drawContours(frame,[c3],-1,(255,255,255),-1)
    # cv2.drawContours(frame,[c2],-1,(255,255,255),-1)
    
    return frame
            
            

def closestPoint(point_,Nodes):
    d_smallest=float('inf')
    point_smallest_=Node([150000,150000])          ######################      was 1500 might be wrong
    for node in Nodes:
        #print(point_)
        #print(node)
        #print(" ")
        d=findDistance(point_,node)
        #print(d)
        if d<d_smallest:
            d_smallest=d
            point_smallest_=node
        #print([d_smallest,point_smallest_.co_ord])
    return point_smallest_

def isGoal(node_,goal_,delta,Maze_eqns,robot_radius):
    if euler_dist(node_,goal_)<=delta   and    pathinObstacleFast(node_,goal_,Maze_eqns,robot_radius) ==False :
        return True
    return False


    # def contourIntersect(original_image, contour1, contour2):
    #     # Two separate contours trying to check intersection on
    #     contours = [contour1, contour2]

    #     # Create image filled with zeros the same size of original image
    #     blank = np.zeros(original_image.shape[0:2])

    #     # Copy each contour into its own image and fill it with '1'
    #     image1 = cv2.drawContours(blank.copy(), contours, 0, 1)
    #     image2 = cv2.drawContours(blank.copy(), contours, 1, 1)

    #     # Use the logical AND operation on the two images
    #     # Since the two images had bitwise and applied to it,
    #     # there should be a '1' or 'True' where there was intersection
    #     # and a '0' or 'False' where it didnt intersect
    #     intersection = np.logical_and(image1, image2)

    #     # Check if there was a '1' in the intersection
    #     return intersection.any()


def CheckSurroundingObstacle(node_,R,Maze,threshold,block_dimensions):
        
    
    
    for i in range(8):
        theta=i*360/8
        x_new=node_.co_ord[0]+(np.cos(np.deg2rad(theta))*R)
        y_new=node_.co_ord[1]+(np.sin(np.deg2rad(theta))*R)
        if x_new>block_dimensions[0]:
            x_new=block_dimensions[0]-R-5
        if y_new>block_dimensions[1]:
            y_new=block_dimensions[1]-R-5
        if x_new<0:
            x_new=R+5
        if y_new<0:
            y_new=R+5
        new_node_=Node([x_new,y_new])
        if Maze[int(y_new/threshold)][int(x_new/threshold)][0]==255   and  Maze[int(y_new/threshold)][int(x_new/threshold)][1]==255   and   Maze[int(y_new/threshold)][int(x_new/threshold)][2]==255 :
            return True
    return False

def RRT(Maze,Maze_eqns,start,goal,counter_,delta,threshold,block_dimensions,robot_radius):  
    R=254+20     #radius
    
    
    Nodes=[]
    
    start_=Node([start[0],start[1]])
    #goal=[10000,10000]
    goal_=Node([goal[0],goal[1]])
    w=block_dimensions[0]
    h=block_dimensions[1]
        
    Nodes.append(start_)
    counter=0
    flag=0
    start_.cost=0
    # radius=delta+20
    radius=robot_radius

    print("algo started")

    start_time=time.time()
    maze_solved_time=0
    
    while 1:#counter<counter_:
        counter=counter+1 
        #Nodes_list=[]
        for i in range(1):
            x=randint(0,w-R)
            y=randint(0,h-R)
            point_=Node([x,y])
            point_neighbour_=closestPoint(point_,Nodes)
            #if Maze[int(x/threshold)][int(y/threshold)][0]==255:         #obst
                #continue
            #write heurestic part here
            
            #interpolate function
            theta=np.arctan2(point_.co_ord[1]-point_neighbour_.co_ord[1],point_.co_ord[0]-point_neighbour_.co_ord[0])
            new_x=x
            new_y=y
            if findDistance(point_,point_neighbour_)>delta:
                
                new_x=point_neighbour_.co_ord[0]+(delta*np.cos(theta))
                new_y=point_neighbour_.co_ord[1]+(delta*np.sin(theta))
                #if new_x<x    or  new_y<y:
                    #continue
                if new_x>w:
                    new_x=w-R
                if new_x<0:
                    new_x=0
                if new_y>h:
                    new_y=h-R
                if new_y<0:
                    new_y=0
                point_=Node([new_x,new_y])
            if time.time()-start_time>2.5:
                print("Time exceded to solver")
                flag=0
                return []
                
            


             ###################################
            #    RRT* for next comment block
            #######################################    
            # costs=[]
            # Nodes_list_nearest_=FindNeighbourswithDistance(Nodes,point_,radius)
            # #print(len(Nodes_list_nearest_))
            # for n_ in Nodes_list_nearest_:
            #     costs.append(n_.cost+euler_dist(n_,point_))
            
            # min_cost=min(costs)
            # index=costs.index(min_cost)
            # #point_neighbour_dash_=Nodes_list_nearest_[index]
            
                
            # point_neighbour_=Nodes_list_nearest_[index]
            # point_.cost=point_neighbour_.cost+euler_dist(point_,point_neighbour_)
            
            for cir in Maze_eqns:
                if euler_dist(goal_,Node([cir[0],cir[1]]))<robot_radius+100:
                    return None
            
            

        
        if pathinObstacleFast(point_,point_neighbour_,Maze_eqns,robot_radius)==True:
            counter=counter-1
            #print("hi")
            #print([point_neighbour_.co_ord,point_.co_ord])
            continue
        
        point_neighbour_.neighbours.append(point_)
    
        Nodes.append(point_)
        
        

        if isGoal(point_,goal_,delta,Maze_eqns,robot_radius)==True:#if point_.co_ord[0]>w   and  point_.co_ord[0]<w  and point_.co_ord[1]>h   and  point_.co_ord[1]<h:
            flag=1
            # print("Final cost is ....",point_.cost+euler_dist(point_,goal_))
            # print(Nodes[len(Nodes)-1].co_ord)
            break
        
        
    
        if flag==1:
            # print("Maze Solved")
            # print("Iterations= ")
            maze_solved_time=time.time()
            # print("counter:",counter)
            # print("time taken to solve maze: ",maze_solved_time- start_time)

        elif time.time()-start_time>4.5:
            print("Time exceded to solver")
            flag=0
            return []

        else:
            #print("Could not solve")
            pass
            
        
    
    
    
    
    
    
    
    print(Nodes[0].co_ord)             #Start is start
    print(Nodes[len(Nodes)-1].co_ord)    #last is goal
    Nodes[len(Nodes)-1].neighbours.append(goal_)
    Nodes.append(goal_)


    maze_solver_bfs_start_time=time.time()
    drawnMaze,path_=maze_solver_bfs(Nodes,Nodes[len(Nodes)-1],Maze,threshold)  

    maze_solver_bfs_time=time.time()
    # print("Maze solver bfs took ",maze_solver_bfs_start_time- maze_solver_bfs_time) 
    path_=[ele for ele in reversed(path_)] 
    path_optimized=FindOptimizedPath(path_,Maze,Maze_eqns,threshold,R,block_dimensions,robot_radius)
    path_optimized.append(path_[len(path_)-1])
    path_optimized_co_ord=[]
    optimized_path_time=time.time()
    # print("optimized path took ",optimized_path_time- maze_solver_bfs_time)

    for p in path_optimized:
        # print(p.co_ord)
        path_optimized_co_ord.append(p.co_ord)

    
    # print(drawnMaze.shape)


    for p in range(len(path_optimized)-1):
        cv2.line(drawnMaze,(int(path_optimized[p].co_ord[0]/threshold),int(path_optimized[p].co_ord[1]/threshold)),(int(path_optimized[p+1].co_ord[0]/threshold),int(path_optimized[p+1].co_ord[1]/threshold)),(250,0,255),10)

    resizedMaze=cv2.resize(drawnMaze,(300,300),interpolation = cv2.INTER_AREA)
    # cv2.imshow("maze",resizedMaze)
    # print("Last kachra took ",time.time()-optimized_path_time)
    
    
    # while 1:
    #     if cv2.waitKey(10) & 0xFF == ord('q'):
    #         break
    
    # plt.show()
    # cv2.destroyAllWindows()
    print("RRT DONE")

    return path_optimized_co_ord
#    for n in range (1,len(Nodes)):
#        
#        for n1 in Nodes[n].neighbours:
#            #print(n1.co_ord)
            

def FindNeighbourswithDistance(Nodes,point_,radius):
    Nodes_list_=[]
    for n_ in Nodes:
        if euler_dist(point_,n_)<radius:
            Nodes_list_.append(n_)
    return Nodes_list_




def FindOptimizedPath(path_,Maze,Maze_eqns,threshold,R,block_dimensions,robot_radius):
    print("Finding optimized path")
    first_=path_[0]
    second_=0
    i=0
    path_optimized_=[]
    while(1):
        #first_=path_[0]
        second_=path_[len(path_)-1-i]
        
        i=i+1
        #result=CheckObstruction(first_,second_,Maze,threshold)
        # result=pathinObstacleOptimum(first_,second_,Maze, threshold,R,block_dimensions)
        result=pathinObstacleFast(first_,second_,Maze_eqns,robot_radius)

        if result==True:
            continue
        else:
            if second_==path_[len(path_)-1]:
                path_optimized_.append(first_)
                return path_optimized_ 
            else:
                
                #print("'''''''''''''''''''''''")
                i=0
                path_optimized_.append(first_)
                first_=second_
                
                
def CheckObstruction(first_,second_,Maze,threshold):
    x1,y1=first_.co_ord
    x2,y2=second_.co_ord
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
    if x1!=x2:    
        for d in np.arange(x1,x2,100).tolist():
            if x2-x1!=0:
                y=(y2-y1)*(d-x1)/(x2-x1)+y1
    
                if Maze[int(y/threshold)][int(d/threshold)][0]==255   and   Maze[int(y/threshold)][int(d/threshold)][1]==255    and    Maze[int(y/threshold)][int(d/threshold)][2]==255:
                    return True
    y_dash=[y1,y2]    
    if x1==x2:
        #print("hi")
        for d in np.arange(min(y_dash),max(y_dash)):
            if Maze[int(d/threshold)][int(x1/threshold)][0]==255   and   Maze[int(d/threshold)][int(x1/threshold)][1]==255    and    Maze[int(d/threshold)][int(x1/threshold)][2]==255:
                    return True
    return False






#pass the last node and it will go to the first node
def maze_solver_bfs(Nodes,node,frame,threshold):
    print("Sol of BFS")
    
    
    
    frame_=copy.copy(frame)
    path=[]
    
    print("loop started")
    
    start=node
    path=[]
    path.append(start)
    while 1:
        start=SearchinNeighbour(start,Nodes)
        path.append(start)
        #print(start.co_ord)
        if start==Nodes[0]:       #i.e.start point
            break
    print("path is")
    #for c in path:
        #print(c.cost)
    for i in range(len(path)-1):
        cv2.line(frame_,(int(path[i].co_ord[0]/threshold),int(path[i].co_ord[1]/threshold)),(int(path[i+1].co_ord[0]/threshold),int(path[i+1].co_ord[1]/threshold)),(0,0,255),10)
    return frame_,path
    



       
def SearchinNeighbour(start,Nodes):
    for c in Nodes:
        for d in c.neighbours:
            if d==start:
                return c
    


def euler_dist(node1_,node2_):
    x1,y1=node1_.co_ord
    x2,y2=node2_.co_ord
    return (((y2-y1)**2)+((x2-x1)**2))**(0.5)



        
        
    
    
   
def findIndex(n,Nodes):
    for i in range(len(Nodes)):
        if Nodes[i]==n:
            return i
    return -1
def presentIndex(n,Nodes):
    for node in Nodes:
        if node==n:
            return True
    return False

def plotLine(x1,y1,x2,y2,color):#,Xpath,Ypath):
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
    if x1!=x2:    
        for d in np.arange(x1,x2,0.25).tolist():
            if x2-x1!=0:
                y=(y2-y1)*(d-x1)/(x2-x1)+y1
    
                plt.plot(d,y,color,markersize=2)
        
    if x1==x2:
        print("hi")
        if y1>y2:
            [x1,y1,x2,y2]=[x2,y2,x1,y1]
        for d in np.arange(y1,y2,0.25).tolist():
            plt.plot(x1,d,color,markersize=2)
        #print("hi")
    #time.sleep(0.05)
        #Xpath.append(d)
        #Ypath.append(y)
        #print(Xpath)
        #print(Ypath)
        #return [[Xpath],[Ypath]]
    
def printPointneighbours(point_neighbour_):
    lists=[]
    for c in point_neighbour_.neighbours:
        lists.append(c.co_ord)
    print([point_neighbour_.co_ord,lists])
    
def pathinObstacle(point_,point_neighbour_,Maze, threshold,R,block_dimensions):
    [x1,y1]=point_.co_ord
    [x2,y2]=point_neighbour_.co_ord
    
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
        
    x1=x1/threshold
    x2=x2/threshold
    y1=y1/threshold
    y2=y2/threshold
    for d in np.arange(int(x1),int(x2)).tolist():
        if x2-x1>1:
            y=(y2-y1)*(d-x1)/(x2-x1)+y1
        #if y<block_dimensions[1]/threshold
        
        else:
            y=y1
        
        p=Maze[int(y)][int(d)]
        if p[0]==255   and p[1]==255    and   p[2]==255:
            return True
        
        if CheckSurroundingObstacle(Node([d*threshold,y*threshold]),R,Maze,threshold,block_dimensions )==True   :
            return True
            
    return False



def pathinObstacleOptimum(point_,point_neighbour_,Maze, threshold,R,block_dimensions):
    [x1,y1]=point_.co_ord
    [x2,y2]=point_neighbour_.co_ord
    
    if x1>x2   and x1-x2!=0:
        [x1,y1,x2,y2]=[x2,y2,x1,y1]
  
    for d in np.arange(int(x1),int(x2)).tolist():
        if x2-x1>1:
            y=(y2-y1)*(d-x1)/(x2-x1)+y1
        #if y<block_dimensions[1]/threshold
        
        else:
            y=y1
        
        p=Maze[int(y/threshold)][int(d/threshold)]
        if p[0]==255   and p[1]==255    and   p[2]==255:
            return True
        
        if CheckSurroundingObstacle(Node([d,y]),R,Maze,threshold,block_dimensions )==True   :
            return True
            
    return False





def pathinObstacleFast(point_,point_neighbour_,Maze_eqns,r ):  #Mazeeqns has [(xc,yc,R)] r in robot radius
    [x1,y1]=point_.co_ord
    [x2,y2]=point_neighbour_.co_ord
    
    if x1>x2   and x1-x2!=0:       ########smaller x2 bigger x1
        [x1,y1,x2,y2]=[x2,y2,x1,y1]

    for cir in Maze_eqns:
        xc,yc,R=cir
        
        # print(xc,yc,R)
        #### checking center line
        ans=checkCollision(x1,y1,x2,y2,xc,yc,R)
        if ans==True:
            return True


        # print("Not in center")

        
        if x2!=x1:
            # print("not eq")
            tantheta=(y2-y1)/(x2-x1)
            theta=np.arctan2(tantheta,1)

            # print("th:",theta)
            upperline=(x1-(r*np.sin(theta)),  y1+(r*np.cos(theta))  ,   x2-(r*np.sin(theta)),  y2+(r*np.cos(theta)) )
            #x1 y1 x2 y2
            upperline=upperline
            # print("upp line is",upperline)
            nx1, ny1, nx2, ny2=upperline
            ans=checkCollision(nx1,ny1,nx2,ny2,xc,yc,R)
            if ans==True:
                return True




            lowerline=(x1+(r*np.sin(theta)),  y1-(r*np.cos(theta))  ,   x2+(r*np.sin(theta)),  y2-(r*np.cos(theta)) )
            lowerline=lowerline
            #x1 y1 x2 y2
            nx1, ny1, nx2, ny2=lowerline
            ans=checkCollision(nx1,ny1,nx2,ny2,xc,yc,R)
            if ans==True:
                return True

        else:

            nx1,ny1,nx2,ny2= x1-r, y1, x2-r, y2
            upperline=(nx1,ny1,nx2,ny2)
            ans=checkCollision(nx1,ny1,nx2,ny2,xc,yc,R)
            if ans==True:
                return True


            nx1,ny1,nx2,ny2= x1+r, y1, x2+r, y2
            lowerline=(nx1,ny1,nx2,ny2)
            ans=checkCollision(nx1,ny1,nx2,ny2,xc,yc,R)
            if ans==True:
                return True






    return False


def giveabcofLine(x1,y1,x2,y2):
    a,b,c=(0,0,0)
        

    if x1==x2  and y1==y2:
        a,b,c=(0,0,0)

    elif x2==x1:
        a=y2-y1
        b=0
        c=-x1*(y2-y1)

    elif y2==y1:
        a=0
        b=x2-x1
        c=-y1*(x2-x1)

    else:
        a=-1/(x2-x1)
        b=1/(y2-y1)
        c=(-y1/(y2-y1))+(x1/(x2-x1))

    return (a,b,c)


def checkRange(x1,y1,x2,y2, xin,yin):
    xbig=max(x1,x2)
    xsmall=min(x1,x2)

    ybig=max(y1,y2)
    ysmall=min(y1,y2)


    if xin>xbig   or   xin<xsmall   or    yin>ybig    or   yin<ysmall:
        return False
    return True



def checkCollision(x1,y1,x2,y2, p, q, r): #a b c are ax+by+c=0     x,y,radi ...
      
    # Finding the distance of line  
    # from center.
    dist=0
    a,b,c=giveabcofLine(x1,y1,x2,y2)
    if a==b  and b==c:
        dist=0
    else:
        dist = ((abs(a * p + b * q + c)) /
            math.sqrt(a * a + b * b)) 
  
    # Checking if the distance is less  
    # than, greater than or equal to radius. 
    if (r == dist)  or  (r > dist) : 
        ### find pts and check if in range
        if x1==x2:  ## veri line
            k=x1
            A=1
            B=-2*q
            C= p**2 + q**2 - r**2 - (2*k*p) + k**2
            yin1,yin2=np.roots([A,B,C])
            xin1,xin2=k,k
            # print(xin1,yin1,xin2,yin2)
            if checkRange(x1,y1,x2,y2,xin1,yin1)   or  checkRange(x1,y1,x2,y2,xin2,yin2):
                return True
        else:
            m=(y2-y1)/(x2-x1)
            c=y1 - m*x1
            A=m**2 +1
            B=2*((m*c)-(m*q)-p)
            C=(q**2 - r**2 +p**2 -(2*c*q) + c**2)
            xin1,xin2=np.roots([A,B,C])
            yin1= (m*xin1) + c
            yin2= (m*xin2) +c
            # print(xin1,yin1,xin2,yin2)

            if checkRange(x1,y1,x2,y2,xin1,yin1)   or  checkRange(x1,y1,x2,y2,xin2,yin2):
                return True

        return False


    else: 
        return False




# #do update of these outside
# saftey_distance_for_controller=400    #is radius
# block_dimensions=[3200,3200]
# Maze_eqns=[(1600,1600,saftey_distance_for_controller)]  # x y r of obstacle
# Maze=MazeMaker(10,[block_dimensions[0],block_dimensions[1]],Maze_eqns)

# robot_pose_map=[0,0]
# goal_pose_map=[2500,2500]
# pose_list=RRT(Maze,Maze_eqns,(robot_pose_map[0],robot_pose_map[1]),(goal_pose_map[0],goal_pose_map[1]),800,800,10,block_dimensions,400)
                    
# print(goal_list)



