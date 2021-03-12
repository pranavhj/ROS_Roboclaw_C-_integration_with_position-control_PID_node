import socket 
import struct
import rospy
import sys
from geometry_msgs.msg           import Pose, PoseStamped, PoseArray
UDP_IP = "127.0.0.1"
UDP_PORT = 60010
 
sock = socket.socket(socket.AF_INET, # Internet
              socket.SOCK_DGRAM) # UDP

sock.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

sock.bind(('', UDP_PORT))
#sock.connect(('192.168.0.243',UDP_PORT))


#sock.send('0:0',('192.168.43.212',UDP_PORT)) 


rospy.init_node('PosePublisher',anonymous=True)
publc=rospy.Publisher('leftControllerPose',PoseStamped,queue_size=100)
pubrc=rospy.Publisher('rightControllerPose',PoseStamped,queue_size=100)
pubhead=rospy.Publisher('headPose',PoseStamped,queue_size=100)
pubtrac1=rospy.Publisher('trackerPose1',PoseStamped,queue_size=100)
pubtrac2=rospy.Publisher('trackerPose2',PoseStamped,queue_size=100)



print("Starting Receiving")


print()
while True :
	print("Trying to recv")

	raw_data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
	#print(raw_data,addr)
	#print(sys.getsizeof(raw_data))
	raw_data=struct.unpack('fffffffi', raw_data)
	print( raw_data )

	poseStamped=PoseStamped()
	poseStamped.header.frame_id=str(raw_data[7])

	poseStamped.pose.position.x=(raw_data[0])
	poseStamped.pose.position.y=(raw_data[1])
	poseStamped.pose.position.z=(raw_data[2])

	poseStamped.pose.orientation.x=(raw_data[3])
	poseStamped.pose.orientation.y=(raw_data[4])
	poseStamped.pose.orientation.z=(raw_data[5])
	poseStamped.pose.orientation.w=(raw_data[6])

	frame_id=raw_data[7]
	if frame_id==0:
		pubhead.publish(poseStamped)

	if frame_id==1:
		publc.publish(poseStamped)

	if frame_id==2:
		pubrc.publish(poseStamped)

	if frame_id==3:
		pubtrac1.publish(poseStamped)

	if frame_id==4:
		pubtrac2.publish(poseStamped)




