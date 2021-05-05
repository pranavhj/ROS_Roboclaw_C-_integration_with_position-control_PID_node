

###################################################################################################
# Very imp
# Python Version := 2.7.12
# Numpy version := 1.16.6
# Scipy version := 1.2.3
###################################################################################################



import rospy
import sys
from geometry_msgs.msg           import Pose, PoseStamped, PoseArray

from numpy import *
from matplotlib.pyplot import *
from mpl_toolkits.mplot3d.axes3d import Axes3D

import numpy as np

import time


datapoints1=[]
datapoints2=[]

collecting=True

def TrackerDataCallback1(data):
    if collecting:
    
        pt=data.pose.position
        datapoints1.append((pt.x,pt.y,pt.z))

def TrackerDataCallback2(data):
    if collecting:

        pt=data.pose.position
        datapoints2.append((pt.x,pt.y,pt.z))



rospy.init_node('PosePublisher',anonymous=True, disable_signals=True)
publc=rospy.Publisher('leftControllerPose',PoseStamped,queue_size=100)
pubrc=rospy.Publisher('rightControllerPose',PoseStamped,queue_size=100)
pubhead=rospy.Publisher('headPose',PoseStamped,queue_size=100)
pubtrac1=rospy.Publisher('trackerPose1',PoseStamped,queue_size=100)
pubtrac2=rospy.Publisher('trackerPose2',PoseStamped,queue_size=100)

rospy.Subscriber("/trackerPose1", PoseStamped, TrackerDataCallback1)
rospy.Subscriber("/trackerPose2", PoseStamped, TrackerDataCallback2)



#-------------------------------------------------------------------------------
# Generate points on circle
# P(t) = r*cos(t)*u + r*sin(t)*(n x u) + C
#-------------------------------------------------------------------------------
def generate_circle_by_vectors(t, C, r, n, u):
    n = n/linalg.norm(n)
    u = u/linalg.norm(u)
    P_circle = r*cos(t)[:,newaxis]*u + r*sin(t)[:,newaxis]*cross(n,u) + C
    return P_circle

def generate_circle_by_angles(t, C, r, theta, phi):
    # Orthonormal vectors n, u, <n,u>=0
    n = array([cos(phi)*sin(theta), sin(phi)*sin(theta), cos(theta)])
    u = array([-sin(phi), cos(phi), 0])
    
    # P(t) = r*cos(t)*u + r*sin(t)*(n x u) + C
    P_circle = r*cos(t)[:,newaxis]*u + r*sin(t)[:,newaxis]*cross(n,u) + C
    return P_circle

#-------------------------------------------------------------------------------
# Generating circle
#-------------------------------------------------------------------------------
r = 2.5               # Radius
C = array([3,3,4])    # Center
theta = 45/180*pi     # Azimuth
phi   = -30/180*pi    # Zenith

t = linspace(0, 2*pi, 100)
# P_gen = generate_circle_by_angles(t, C, r, theta, phi)
# print(P_gen)


#-------------------------------------------------------------------------------
# Cluster of points
#-------------------------------------------------------------------------------
t = linspace(-pi, -0.25*pi, 100)
n = len(t)
P = generate_circle_by_angles(t, C, r, theta, phi)

# Add some random noise to the points
P+= random.normal(size=P.shape) * 0.1

print(P)





while True:
    try:
        print 'Help! Im stuck in a while loop! Press Ctrl+C to save me!'
        time.sleep(0.5)
    except KeyboardInterrupt:
        collecting=False
        break


print(len(datapoints1))
datapoints1=np.asarray(datapoints1)
P1=datapoints1

print(len(datapoints2))
datapoints2=np.asarray(datapoints2)
P2=datapoints2


#-------------------------------------------------------------------------------
# Plot
#-------------------------------------------------------------------------------
f1, ax1 = subplots(1, 3, figsize=(15,5))
alpha_pts = 0.5
i = 0
# ax[i].plot(P_gen[:,0], P_gen[:,1], 'y-', lw=3, label='Generating circle')
ax1[i].scatter(P1[:,0], P1[:,1], alpha=alpha_pts, label='Cluster points P1')
ax1[i].set_title('View X-Y')
ax1[i].set_xlabel('x'); ax1[i].set_ylabel('y')
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1)
ax1[i].grid()
i = 1
# ax[i].plot(P_gen[:,0], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax1[i].scatter(P1[:,0], P1[:,2], alpha=alpha_pts, label='Cluster points P1')
ax1[i].set_title('View X-Z')
ax1[i].set_xlabel('x'); ax1[i].set_ylabel('z')
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1)
ax1[i].grid()
i = 2
# ax[i].plot(P_gen[:,1], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax1[i].scatter(P1[:,1], P1[:,2], alpha=alpha_pts, label='Cluster points P1')
ax1[i].set_title('View Y-Z')
ax1[i].set_xlabel('y'); ax1[i].set_ylabel('z')
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1)
ax1[i].legend()
ax1[i].grid()

f2, ax2 = subplots(1, 3, figsize=(15,5))
alpha_pts = 0.5
i = 0
# ax[i].plot(P_gen[:,0], P_gen[:,1], 'y-', lw=3, label='Generating circle')
ax2[i].scatter(P2[:,0], P2[:,1], alpha=alpha_pts, label='Cluster points P2')
ax2[i].set_title('View X-Y')
ax2[i].set_xlabel('x'); ax2[i].set_ylabel('y')
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1)
ax2[i].grid()
i = 1
# ax[i].plot(P_gen[:,0], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax2[i].scatter(P2[:,0], P2[:,2], alpha=alpha_pts, label='Cluster points P2')
ax2[i].set_title('View X-Z')
ax2[i].set_xlabel('x'); ax2[i].set_ylabel('z')
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1)
ax2[i].grid()
i = 2
# ax[i].plot(P_gen[:,1], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax2[i].scatter(P2[:,1], P2[:,2], alpha=alpha_pts, label='Cluster points P2')
ax2[i].set_title('View Y-Z')
ax2[i].set_xlabel('y'); ax2[i].set_ylabel('z')
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1)
ax2[i].legend()
ax2[i].grid()

#-------------------------------------------------------------------------------
# FIT CIRCLE 2D
# - Find center [xc, yc] and radius r of circle fitting to set of 2D points
# - Optionally specify weights for points
#
# - Implicit circle function:
#   (x-xc)^2 + (y-yc)^2 = r^2
#   (2*xc)*x + (2*yc)*y + (r^2-xc^2-yc^2) = x^2+y^2
#   c[0]*x + c[1]*y + c[2] = x^2+y^2
#
# - Solution by method of least squares:
#   A*c = b, c' = argmin(||A*c - b||^2)
#   A = [x y 1], b = [x^2+y^2]
#-------------------------------------------------------------------------------
def fit_circle_2d(x, y, w=[]):
    
    A = array([x, y, ones(len(x))]).T
    b = x**2 + y**2
    
    # Modify A,b for weighted least squares
    if len(w) == len(x):
        W = diag(w)
        A = dot(W,A)
        b = dot(W,b)
    
    # Solve by method of least squares
    c = linalg.lstsq(A,b,rcond=None)[0]
    
    # Get circle parameters from solution c
    xc = c[0]/2
    yc = c[1]/2
    r = sqrt(c[2] + xc**2 + yc**2)
    return xc, yc, r


#-------------------------------------------------------------------------------
# RODRIGUES ROTATION
# - Rotate given points based on a starting and ending vector
# - Axis k and angle of rotation theta given by vectors n0,n1
#   P_rot = P*cos(theta) + (k x P)*sin(theta) + k*<k,P>*(1-cos(theta))
#-------------------------------------------------------------------------------
def rodrigues_rot(P, n0, n1):
    
    # If P is only 1d array (coords of single point), fix it to be matrix
    if P.ndim == 1:
        P = P[newaxis,:]
    
    # Get vector of rotation k and angle theta
    n0 = n0/linalg.norm(n0)
    n1 = n1/linalg.norm(n1)
    k = cross(n0,n1)
    k = k/linalg.norm(k)
    theta = arccos(dot(n0,n1))
    
    # Compute rotated points
    P_rot = zeros((len(P),3))
    for i in range(len(P)):
        P_rot[i] = P[i]*cos(theta) + cross(k,P[i])*sin(theta) + k*dot(k,P[i])*(1-cos(theta))

    return P_rot


#-------------------------------------------------------------------------------
# ANGLE BETWEEN
# - Get angle between vectors u,v with sign based on plane with unit normal n
#-------------------------------------------------------------------------------
def angle_between(u, v, n=None):
    if n is None:
        return arctan2(linalg.norm(cross(u,v)), dot(u,v))
    else:
        return arctan2(dot(n,cross(u,v)), dot(u,v))

    
#-------------------------------------------------------------------------------
# - Make axes of 3D plot to have equal scales
# - This is a workaround to Matplotlib's set_aspect('equal') and axis('equal')
#   which were not working for 3D
#-------------------------------------------------------------------------------
def set_axes_equal_3d(ax):
    limits = array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    spans = abs(limits[:,0] - limits[:,1])
    centers = mean(limits, axis=1)
    radius = 0.5 * max(spans)
    ax.set_xlim3d([centers[0]-radius, centers[0]+radius])
    ax.set_ylim3d([centers[1]-radius, centers[1]+radius])
    ax.set_zlim3d([centers[2]-radius, centers[2]+radius])







fig1 = figure(figsize=(15,11))
alpha_pts = 0.5
figshape = (2,3)
ax1 = [None]*4
ax1[0] = subplot2grid(figshape, loc=(0,0), colspan=2)
ax1[1] = subplot2grid(figshape, loc=(1,0))
ax1[2] = subplot2grid(figshape, loc=(1,1))
ax1[3] = subplot2grid(figshape, loc=(1,2))
i = 0
ax1[i].set_title('Fitting circle in 2D coords projected onto fitting plane')
ax1[i].set_xlabel('x'); ax1[i].set_ylabel('y')
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1); ax1[i].grid()
i = 1
# ax[i].plot(P_gen[:,0], P_gen[:,1], 'y-', lw=3, label='Generating circle')
ax1[i].scatter(P1[:,0], P1[:,1], alpha=alpha_pts, label='Cluster points P1')
ax1[i].set_title('View X-Y')
ax1[i].set_xlabel('x'); ax1[i].set_ylabel('y');
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1); ax1[i].grid()
i = 2
# ax[i].plot(P_gen[:,0], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax1[i].scatter(P1[:,0], P1[:,2], alpha=alpha_pts, label='Cluster points P1')
ax1[i].set_title('View X-Z')
ax1[i].set_xlabel('x'); ax1[i].set_ylabel('z');
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1); ax1[i].grid()
i = 3
# ax[i].plot(P_gen[:,1], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax1[i].scatter(P1[:,1], P1[:,2], alpha=alpha_pts, label='Cluster points P1')
ax1[i].set_title('View Y-Z')
ax1[i].set_xlabel('y'); ax1[i].set_ylabel('z');
ax1[i].set_aspect('equal', 'datalim'); ax1[i].margins(.1, .1); ax1[i].grid()



fig2 = figure(figsize=(15,11))
alpha_pts = 0.5
figshape = (2,3)
ax2 = [None]*4
ax2[0] = subplot2grid(figshape, loc=(0,0), colspan=2)
ax2[1] = subplot2grid(figshape, loc=(1,0))
ax2[2] = subplot2grid(figshape, loc=(1,1))
ax2[3] = subplot2grid(figshape, loc=(1,2))
i = 0
ax2[i].set_title('Fitting circle in 2D coords projected onto fitting plane')
ax2[i].set_xlabel('x'); ax2[i].set_ylabel('y');
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1); ax2[i].grid()
i = 1
# ax[i].plot(P_gen[:,0], P_gen[:,1], 'y-', lw=3, label='Generating circle')
ax2[i].scatter(P2[:,0], P2[:,1], alpha=alpha_pts, label='Cluster points P2')
ax2[i].set_title('View X-Y')
ax2[i].set_xlabel('x'); ax2[i].set_ylabel('y');
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1); ax2[i].grid()
i = 2
# ax2[i].plot(P_gen[:,0], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax2[i].scatter(P2[:,0], P2[:,2], alpha=alpha_pts, label='Cluster points P2')
ax2[i].set_title('View X-Z')
ax2[i].set_xlabel('x'); ax2[i].set_ylabel('z');
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1); ax2[i].grid()
i = 3
# ax2[i].plot(P_gen[:,1], P_gen[:,2], 'y-', lw=3, label='Generating circle')
ax2[i].scatter(P2[:,1], P2[:,2], alpha=alpha_pts, label='Cluster points P2')
ax2[i].set_title('View Y-Z')
ax2[i].set_xlabel('y'); ax2[i].set_ylabel('z');
ax2[i].set_aspect('equal', 'datalim'); ax2[i].margins(.1, .1); ax2[i].grid()


#-------------------------------------------------------------------------------
# (1) Fitting plane by SVD for the mean-centered data
# Eq. of plane is <p,n> + d = 0, where p is a point on plane and n is normal vector
#-------------------------------------------------------------------------------
P_mean1 = P1.mean(axis=0)
P_centered1 = P1 - P_mean1
U1,s1,V1 = linalg.svd(P_centered1)

# Normal vector of fitting plane is given by 3rd column in V
# Note linalg.svd returns V^T, so we need to select 3rd row from V^T
normal1 = V1[2,:]
d1 = -dot(P_mean1, normal1)  # d = -<p,n>


P_mean2 = P2.mean(axis=0)
P_centered2 = P2 - P_mean2
U2,s2,V2 = linalg.svd(P_centered2)

# Normal vector of fitting plane is given by 3rd column in V
# Note linalg.svd returns V^T, so we need to select 3rd row from V^T
normal2 = V2[2,:]
d2 = -dot(P_mean2, normal2)  # d = -<p,n>

#-------------------------------------------------------------------------------
# (2) Project points to coords X-Y in 2D plane
#-------------------------------------------------------------------------------
P_xy1 = rodrigues_rot(P_centered1, normal1, [0,0,1])

ax1[0].scatter(P_xy1[:,0], P_xy1[:,1], alpha=alpha_pts, label='Projected points')

P_xy2 = rodrigues_rot(P_centered2, normal2, [0,0,1])

ax2[0].scatter(P_xy2[:,0], P_xy2[:,1], alpha=alpha_pts, label='Projected points')


#-------------------------------------------------------------------------------
# (3) Fit circle in new 2D coords
#-------------------------------------------------------------------------------
xc1, yc1, r1 = fit_circle_2d(P_xy1[:,0], P_xy1[:,1])

xc2, yc2, r2 = fit_circle_2d(P_xy2[:,0], P_xy2[:,1])

#--- Generate circle points in 2D
t1 = linspace(0, 2*pi, 100)
xx1 = xc1 + r1*cos(t1)
yy1 = yc1 + r1*sin(t1)

#--- Generate circle points in 2D
t2 = linspace(0, 2*pi, 100)
xx2 = xc2 + r2*cos(t2)
yy2 = yc2 + r2*sin(t2)


ax1[0].plot(xx1, yy1, 'k--', lw=2, label='Fitting circle')
ax1[0].plot(xc1, yc1, 'k+', ms=10)
ax1[0].legend()

ax2[0].plot(xx2, yy2, 'k--', lw=2, label='Fitting circle')
ax2[0].plot(xc2, yc2, 'k+', ms=10)
ax2[0].legend()

#-------------------------------------------------------------------------------
# (4) Transform circle center back to 3D coords
#-------------------------------------------------------------------------------
C1 = rodrigues_rot(array([xc1,yc1,0]), [0,0,1], normal1) + P_mean1
C1 = C1.flatten()

C2 = rodrigues_rot(array([xc2,yc2,0]), [0,0,1], normal2) + P_mean2
C2 = C2.flatten()




#--- Generate points for fitting circle
t1 = linspace(0, 2*pi, 100)
u1 = P1[0] - C1
P_fitcircle1 = generate_circle_by_vectors(t1, C1, r1, normal1, u1)

#--- Generate points for fitting circle
t2 = linspace(0, 2*pi, 100)
u2 = P2[0] - C2
P_fitcircle2 = generate_circle_by_vectors(t2, C2, r2, normal2, u2)



ax1[1].plot(P_fitcircle1[:,0], P_fitcircle1[:,1], 'k--', lw=2, label='Fitting circle1')
ax1[2].plot(P_fitcircle1[:,0], P_fitcircle1[:,2], 'k--', lw=2, label='Fitting circle1')
ax1[3].plot(P_fitcircle1[:,1], P_fitcircle1[:,2], 'k--', lw=2, label='Fitting circle1')
ax1[3].legend()

ax2[1].plot(P_fitcircle2[:,0], P_fitcircle2[:,1], 'k--', lw=2, label='Fitting circle2')
ax2[2].plot(P_fitcircle2[:,0], P_fitcircle2[:,2], 'k--', lw=2, label='Fitting circle2')
ax2[3].plot(P_fitcircle2[:,1], P_fitcircle2[:,2], 'k--', lw=2, label='Fitting circle2')
ax2[3].legend()

#--- Generate points for fitting arc
u1 = P1[0] - C1
v1 = P1[-1] - C1
theta1 = angle_between(u1, v1, normal1)

#--- Generate points for fitting arc
u2 = P2[0] - C2
v2 = P2[-1] - C2
theta2 = angle_between(u2, v2, normal2)



t1 = linspace(0, theta1, 100)
P_fitarc1 = generate_circle_by_vectors(t1, C1, r1, normal1, u1)

t2 = linspace(0, theta2, 100)
P_fitarc2 = generate_circle_by_vectors(t2, C2, r2, normal2, u2)


ax1[1].plot(P_fitarc1[:,0], P_fitarc1[:,1], 'k-', lw=3, label='Fitting arc1')
ax1[2].plot(P_fitarc1[:,0], P_fitarc1[:,2], 'k-', lw=3, label='Fitting arc1')
ax1[3].plot(P_fitarc1[:,1], P_fitarc1[:,2], 'k-', lw=3, label='Fitting arc1')
ax1[1].plot(C1[0], C1[1], 'k+', ms=10)
ax1[2].plot(C1[0], C1[2], 'k+', ms=10)
ax1[3].plot(C1[1], C1[2], 'k+', ms=10)
ax1[3].legend()

ax2[1].plot(P_fitarc2[:,0], P_fitarc2[:,1], 'k-', lw=3, label='Fitting arc')
ax2[2].plot(P_fitarc2[:,0], P_fitarc2[:,2], 'k-', lw=3, label='Fitting arc')
ax2[3].plot(P_fitarc2[:,1], P_fitarc2[:,2], 'k-', lw=3, label='Fitting arc')
ax2[1].plot(C2[0], C2[1], 'k+', ms=10)
ax2[2].plot(C2[0], C2[2], 'k+', ms=10)
ax2[3].plot(C2[1], C2[2], 'k+', ms=10)
ax2[3].legend()

print('Fitting plane: n = %s' % array_str(normal1, precision=4))
print('Fitting circle: center = %s, r = %.4g' % (array_str(C1, precision=4), r1))

print('Fitting plane: n = %s' % array_str(normal2, precision=4))
print('Fitting circle: center = %s, r = %.4g' % (array_str(C2, precision=4), r2))




print("################Center and rad is ################# ",C)


# Python program to illustrate 
# Append vs write mode 
file1 = open(r"calibration1.txt","w")
L1 = str((C1[0],C1[1],C1[2],r1))
file1.write(L1)
file1.close()

file2 = open(r"calibration2.txt","w")
L2 = str((C2[0],C2[1],C2[2],r2))
file2.write(L2)
file2.close()
  
# # Append-adds at last 
# file1 = open("myfile.txt","a")#append mode 
# file1.write("Today \n") 
# file1.close() 
  
# file1 = open("calibration.txt","r") 
# print "Output of Readlines after appending"
# print file1.readlines() 
# print
# file1.close() 






fig1 = figure(figsize=(15,15))
ax1 = fig1.add_subplot(1,1,1,projection='3d')
# ax.plot(*P_gen.T, color='y', lw=3, label='Generating circle')
ax1.plot(*P1.T, ls='', marker='o', alpha=0.5, label='Cluster points P1')

#--- Plot fitting plane
xx1, yy1 = meshgrid(linspace(0,6,11), linspace(0,6,11))
zz1 = (-normal1[0]*xx1 - normal1[1]*yy1 - d1) / normal1[2]
ax1.plot_surface(xx1, yy1, zz1, rstride=2, cstride=2, color='y' ,alpha=0.2, shade=False)

#--- Plot fitting circle
ax1.plot(*P_fitcircle1.T, color='k', ls='--', lw=2, label='Fitting circle')
ax1.plot(*P_fitarc1.T, color='k', ls='-', lw=3, label='Fitting arc')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
ax1.legend()

ax1.set_aspect('equal', 'datalim')
set_axes_equal_3d(ax1)


print("Starting Receiving")
matplotlib.pyplot.show()


print()




fig2 = figure(figsize=(15,15))
ax2 = fig2.add_subplot(1,1,1,projection='3d')
# ax.plot(*P_gen.T, color='y', lw=3, label='Generating circle')
ax2.plot(*P2.T, ls='', marker='o', alpha=0.5, label='Cluster points P2')

#--- Plot fitting plane
xx2, yy2 = meshgrid(linspace(0,6,11), linspace(0,6,11))
zz2 = (-normal2[0]*xx2 - normal2[1]*yy2 - d2) / normal2[2]
ax2.plot_surface(xx2, yy2, zz2, rstride=2, cstride=2, color='y' ,alpha=0.2, shade=False)

#--- Plot fitting circle
ax2.plot(*P_fitcircle2.T, color='k', ls='--', lw=2, label='Fitting circle')
ax2.plot(*P_fitarc2.T, color='k', ls='-', lw=3, label='Fitting arc')
ax2.set_xlabel('X')
ax2.set_ylabel('Y')
ax2.set_zlabel('Z')
ax2.legend()

ax2.set_aspect('equal', 'datalim')
set_axes_equal_3d(ax2)


print("Starting Receiving")
matplotlib.pyplot.show()


print()

