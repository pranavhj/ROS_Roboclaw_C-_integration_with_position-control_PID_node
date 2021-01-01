import socket
import struct 

UDP_IP = "192.168.43.212"
UDP_PORT = 50006
MESSAGE = b"Hello, World!"



values = (1.2, 2.3, 2.7, 1.2, 2.3, 2.7)
packer = struct.Struct('ffffff')
packed_data = packer.pack(*values)
MESSAGE=packed_data

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)
 
sock = socket.socket(socket.AF_INET, # Internet
                      socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
