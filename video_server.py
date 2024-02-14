# This is server code to send video frames over UDP
import cv2, imutils, socket
import numpy as np
import time
import base64
from realsense_depth import *

BUFF_SIZE = 65536
server_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,BUFF_SIZE)
host_name = socket.gethostname()
host_ip = '172.17.104.180'#  socket.gethostbyname(host_name)
print(host_ip)
port = 9999
socket_address = (host_ip,port)
server_socket.bind(socket_address)
print('Listening at:',socket_address)

#Camer Frames Part
dc = DepthCamera()
# Create mouse event
# cv2.namedWindow("color_frame")

fps,st,frames_to_count,cnt = (0,0,20,0)

while True:
    msg,client_addr = server_socket.recvfrom(BUFF_SIZE)
    print('GOT connection from ',client_addr)
    WIDTH=400
    while(1):
        ret, depth_frame, color_frame = dc.get_frame()
        color_frame = imutils.resize(color_frame,width=WIDTH)
        encoded,buffer = cv2.imencode('.jpg',color_frame,[cv2.IMWRITE_JPEG_QUALITY,80])
        message = base64.b64encode(buffer)
        server_socket.sendto(message,client_addr)
        color_frame = cv2.putText(color_frame,'FPS: '+str(fps),(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        # cv2.imshow('TRANSMITTING VIDEO',color_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            server_socket.close()
            break
        if cnt == frames_to_count:
            try:
                fps = round(frames_to_count/(time.time()-st))
                st=time.time()
                cnt=0
            except:
                pass
        cnt+=1
