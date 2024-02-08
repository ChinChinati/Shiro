from sync_motor import Motor
from math import *
import numpy as np
from numpy import interp
from time import time
from pynput import keyboard



#Robot Pose
x,y,z,roll,pitch,yaw = 0,0,0,0,0,0

LEG_PIECE_1 = 210.67
LEG_PIECE_2 = 200

REST =  [0.04,
        7.87,
        -5.85,
        -2.68,
        -7.6,
        3.74,
        -1.19,
        -8.57,
        3.56,
        0.66,
        4.44,
        -5.23]
def dict(arr):
    d = {}
    for i in range(1,13):
        d[f"{i}"] = arr[i-1]
    return d

def arr(n):
    array = {}
    for i in range(1,13):
        array[i] = i
    return array

def common_arr(n):
    array = {}
    for i in range(0,12):
        array[i] = n
    return array

# step_length = 170
# step_length /= 2
# alpha = radians(30)
# offset_x = 0
# offset_y = 0
# offset_z = 0

# def traj(t):
#     r = step_length/cos(alpha)
#     if t > 4:
#         t = t%4
#     X = 0
#     if t <= 3:
#         Z = 0
#         Y = interp(t,[0,3],[step_length,-step_length])
#     if t > 3 and t <= 4:
#         theta = interp(t,[3,4],[0,pi-(2*alpha)])
#         Y = -r*cos(alpha+theta)
#         Z = -r*sin(alpha+theta) +(r*sin(alpha))
#     return X,Y,Z

# def rotation(r,p,y):
#     r_x1,r_y1,r_z1 = 0,0,0
#     r_x2,r_y2,r_z2 = 0,0,0
#     r_x3,r_y3,r_z3 = 0,0,0
#     r_x4,r_y4,r_z4 = 0,0,0
#     b = 321.5
#     r,p,y = radians(r),radians(p),radians(y)
#     # pitch
#     p = interp(p,[-pi/2,pi/2],[-350,350])
#     r_z1 = -p
#     r_z2 = -p
#     r_z3 = p
#     r_z4 = p

#     # roll
#     h = offset_z - b*sin(r)/2
#     H = offset_z + b*sin(r)/2

#     r_x1 = -h*sin(r)
#     r_x2 = -H*sin(r)
#     r_x3 = -h*sin(r)
#     r_x4 = -H*sin(r)

#     r_z1 += h*cos(r) - offset_z
#     r_z2 += H*cos(r) - offset_z
    # r_z3 += h*cos(r) - offset_z
    # r_z4 += H*cos(r) - offset_z

    # # yaw
    # # Use only when X = 0 | Y = 0
    # theta = 1.079933399
    # X = 341*cos(1.079933399+y) - (321.5/2)
    # Y = 341*sin(1.079933399+y) - (601.5/2)
    # r_x2 += X
    # r_y2 += Y
    # r_z2 += 0
    # r_x3 += -X
    # r_y3 += -Y
    # r_z3 += 0
    # X = 341*cos(1.079933399-y) - (321.5/2)
    # Y = 341*sin(1.079933399-y) - (601.5/2)
    # r_x1 += -X
    # r_y1 += Y
    # r_z1 += 0
    # r_x4 += X
    # r_y4 += -Y
    # r_z4 += 0
    # return r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4

motors = arr(12)

safe = dict([[-45, 20],
          [-178, 10],
          [-3, 178],

          [-20, 45],
          [-10, 178],
          [-178, 3],

          [-20, 45],
          [-10, 178],
          [-178, 3],

          [-45, 20],
          [-178, 10],
          [-3, 178]])




class Doggy(object):

    def __init__(self):
        self.x1,self.y1,self.z1,self.x2,self.y2,self.z2,self.x3,self.y3,self.z3,self.x4,self.y4,self.z4 = 0,0,0,0,0,0,0,0,0,0,0,0
    
    def connect(self):
        self.motor = Motor()
        self.motor.open_port()
        ids = self.motor.scan()
        print(ids)
        if len(ids) == 12:
            print("Connection successful")

    def enable_torque(self):
        self.motor.torque_enable(motors)

    def disable_torque(self):
        self.motor.torque_disable(motors)

    def get_positions(self):
        return self.motor.get_position_angles(motors)

    def move(self,arr):
        # self.motor.torque_enable(motors)
        dis = dict(arr)
        for id in dis:
            if not (safe[id][0]) < dis[id] < (safe[id][1]):
                max = abs((safe[id][1]) - dis[id])
                min = abs((safe[id][0]) - dis[id])
                if max <= min:
                    dis[id] = (safe[id][1])
                else:
                    dis[id] = (safe[id][0])
                print(f"Angle {id} exceeded limit")
        # print(dis)
        self.motor.move_angles(dis)

     # /////////////////////////////////////////////////////////////////////////////////////////////////////////
    def ikin(self,arr):
        x = arr[0]
        z = arr[2]
        y = arr[1]
        
        if x==0:
            theta = 0
        else:
            theta = atan(x/z)
        r = sqrt(z**2 + x**2)
        b = acos((-(LEG_PIECE_1**2) - (LEG_PIECE_2**2) + (r**2) + (y**2))/(2*LEG_PIECE_1*LEG_PIECE_2))
        a = atan(y/r) + acos(-((LEG_PIECE_1**2) + (r**2) + (y**2) - (LEG_PIECE_2**2))/(2*LEG_PIECE_1*(sqrt(z**2 + y**2 + x**2))))
        theta, a, b = degrees(theta), degrees(a),degrees(b)
        return theta, a, b

    def move_xyz(self,arr):
        self.x1,self.y1,self.z1,self.x2,self.y2,self.z2,self.x3,self.y3,self.z3,self.x4,self.y4,self.z4 = arr
        l1 = arr[0:3]
        l2 = arr[3:6]
        l3 = arr[6:9]
        l4 = arr[9:]
        # print(l1)
        l3[1] = -l3[1]
        l4[1] = -l4[1]
        
        l1 = np.array(self.ikin(l1))
        l2 = np.array(self.ikin(l2))
        l3 = np.array(self.ikin(l3))
        l4 = np.array(self.ikin(l4))
        l1[0],l1[1],l1[2] = -l1[0],-(l1[1]+(-25.59-90)),360-(l1[2]+25.59+180)
        l2[0],l2[1],l2[2] = -l2[0],244.41+l2[1]-360,l2[2]+180+25.59-360
        l3[0],l3[1],l3[2] = l3[0],-(25.59+90-l3[1]),25.59+l3[2]+180-360
        l4[0],l4[1],l4[2] = l4[0],-(-25.59-90+l4[1]),360-(l4[2]+25.59+180)
        l1 = np.hstack((l1,l2))
        l1 = np.hstack((l1,l3))
        l1 = np.hstack((l1,l4))
        # l1 = np.array(l1)
        return list(l1)


    # Speed = [0,170]
    def set_time(self, time):
        # self.motor.drive_mode_time_based_profile(motors)
        self.motor.profile_velocity(dict(common_arr(time)))
        # self.motor.profile_acceleration(dict(common_arr(50)))


    def pid(self,p,i,d):
        self.motor.p(p,motors)
        self.motor.i(i,motors)
        self.motor.d(d,motors)
    
    def get_current(self):
        return(list(self.motor.get_current(motors).values()))
    
    def total_current(self):
        _ = np.sum(self.get_current())
        print(f"{_} A")
        return(_)
    
    def get_forces(self):
        dxl_present_current = {}
        for id in motors: 
            bits = self.motor.packetHandler.read2ByteTxRx(self.motor.portHandler, id, 126)
            if bits[0] < 32767:
                dxl_present_current[id-1] = round(bits[0]*.00269 ,2)
            else:
                dxl_present_current[id-1] = round((65536 - bits[0])*.00269 ,2)
        T = dxl_present_current
        return [T[1]+T[2],T[4]+T[5],T[7]+T[8],T[10]+T[11]]
    
    def get_voltage(self):
        print(self.motor.get_voltage(motors))
        return(self.motor.get_voltage(motors))

    def sit(self):
        self.set_time(3000)
        self.move(REST)
    
    def stand(self,height=250):
        self.move(self.move_xyz([0,0,height,0,0,height,0,0,height,0,0,height]))

    # interval: seconds ;
    # def walk_old(self,interval,walkingHeight = 300,roll=0,pitch=0,STEP_TIME = 250, precision=0.1):
    #     self.enable_torque()
    #     self.pid(1900,4000,50)#new pid values
    #     i = walkingHeight
    #     self.set_time(2000)
    #     self.move(self.move_xyz([0,0,i,0,0,i,0,0,i,0,0,i]))

    #     t = 0
    #     global offset_x,offset_y,offset_z
    #     offset_x = 0
    #     offset_y = 0
    #     offset_z = walkingHeight

    #     time_changed = 0
    #     now = time()
    #     while (time() - now) < interval:

    #         x1,y1,z1 = traj(t)
    #         x2,y2,z2 = traj(t+2)
    #         x3,y3,z3 = traj(t+1)
    #         x4,y4,z4 = traj(t+3)
    #         if t%4 >= 0 and t%4 <= 1:
    #             r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4 = rotation(+roll,-pitch,0)
    #         elif t%4 > 1 and t%4 <= 2:
    #             r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4 = rotation(+roll,+pitch,0)
    #         elif t%4 > 2 and t%4 <= 3:
    #             r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4 = rotation(-roll,-pitch,0)
    #         elif t%4 > 3 and t%4 <= 4:
    #             r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4 = rotation(-roll,+pitch,0)

        #     self.move(self.move_xyz([offset_x+r_x1+x1,  offset_y+r_y1+y1,   offset_z+r_z1+z1,
        #                             offset_x+x2+r_x2,    offset_y+y2+r_y2,   offset_z+z2+r_z2,
        #                             offset_x+x3+r_x3,   -offset_y+y3+r_y3,   offset_z+z3+r_z3,
        #                             offset_x+x4+r_x4,   -offset_y+y4+r_y4,   offset_z+z4+r_z4]))
        #     if not time_changed:
        #         self.set_time(STEP_TIME)
        #         time_changed = 1
        #     # t = 0.1 threshold = 50 FAST STEP_TIME = 90 #fastest
        #     # t = 0.14 threshold = 50 FAST STEP_TIME = 150
        #     # t = 0.1  threshold = 20 SLOW STEP_TIME = 150
        #     print(250)
        #     t += precision
            
        # self.set_time(2000)
        # self.move(REST)
        # print("Done")
        # #self.disable_torque()
        
    def walk_trot(self,interval=10,walkingHeight = 300,step_time = 400, step_length = 50,step_height=50, tilt=-5):
        self.enable_torque()
        # self.pid(1900,4000,50)#new pid values
        self.set_time(3000)
        self.stand(walkingHeight)
        t = 0
        offset_x = 0
        offset_y = 0
        offset_z = walkingHeight
        def trajectory(t):
            X = 0
            if t >= 4:
                t = t%4
            if t == 0:
                Z = 0
                Y = step_length
            elif t == 1:
                Z = 0
                # Y = step_length
                Y = 0
                # Y = -step_length
            elif t == 2:
                Z = 0
                Y = -step_length
            elif t == 3:
                Z = -step_height
                Y = 0
            return X,Y,Z

        time_changed = 0
        now = time()
        
        x1,y1,z1 = 0,0,0
        x2,y2,z2 = 0,0,0
        x3,y3,z3 = 0,0,0
        x4,y4,z4 = 0,0,0
        while (time() - now) < interval:

            x1,y1,z1 = trajectory(t+2)
            x2,y2,z2 = trajectory(t)
            x3,y3,z3 = trajectory(t)
            x4,y4,z4 = trajectory(t+2)
            
            self.move(self.move_xyz([offset_x+x1,  offset_y+y1,   offset_z+z1+tilt,
                                    offset_x+x2,    offset_y+y2,   offset_z+z2-tilt,
                                    offset_x+x3,   -offset_y+y3,   offset_z+z3+tilt,
                                    offset_x+x4,   -offset_y+y4,   offset_z+z4-tilt]))
            if not time_changed:
                self.set_time(step_time)
                time_changed = 1
            # t = 0.1 threshold = 50 FAST STEP_TIME = 90 #fastest
            # t = 0.14 threshold = 50 FAST STEP_TIME = 150
            # t = 0.1  threshold = 20 SLOW STEP_TIME = 150
            t += 1
            
        self.set_time(3000)
        self.move(self.move_xyz([   offset_x+x1,    offset_y+y1,   offset_z,
                                    offset_x+x2,    offset_y+y2,   offset_z,
                                    offset_x+x3,   -offset_y+y3,   offset_z,
                                    offset_x+x4,   -offset_y+y4,   offset_z]))
        self.move(REST)
        print("Done")
    
    tilt = 0
    def controller(self,walkingHeight = 300,step_time = 100, step_length = 20,step_height=50, tilt=-1): 
        self.walkingHeight = walkingHeight
        step_length = 20
        step_height = 50
        step_time = 100
        self.tilt = tilt
        t = 0
        time_changed = 0
        
        x1,y1,z1 = 0,0,0
        x2,y2,z2 = 0,0,0
        x3,y3,z3 = 0,0,0
        x4,y4,z4 = 0,0,0
        
        self.MODE = 1 # sitting = 0
                 # standing = 1
                 # straffing = 2
                 # rpy while still standing only = 3
         
        self.MOVEMENT = 0 # still = 0
                     # forward = 1
                     # backward = 2
                     # left = 3
                     # right = 4
                     # right rotation = 5
                     # left rotation = 6

        self.ESC = 0
        offset_x = 0
        offset_y = 0
        offset_z = self.walkingHeight
        
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        def on_press(key):
            try:
                if key.char == '0':
                    self.MODE = 0
                    self.MOVEMENT = 0
                    # print(f'self.MODE:{self.MODE}')
                elif key.char == None:
                    self.MODE = 1
                    self.MOVEMENT = 0
                    self.roll = 0
                    self.pitch = 0
                    self.yaw = 0
                    # print(f'self.MODE:{self.MODE}')
                elif key.char == '8':
                    # self.stand(walkingHeight)
                    self.MODE = 2
                    self.MOVEMENT = 1
                    # print(f'self.MODE:{self.MODE}')
                elif key.char == '2':
                    # self.stand(walkingHeight)
                    self.MODE = 2
                    self.MOVEMENT = 2
                elif key.char == '6':
                    # self.stand(walkingHeight)
                    self.MODE = 2
                    self.MOVEMENT = 3
                elif key.char == '4':
                    # self.stand(walkingHeight)
                    self.MODE = 2
                    self.MOVEMENT = 4
                elif key.char == '7':
                    self.MODE = 2
                    self.MOVEMENT = 5
                elif key.char == '9':
                    self.MODE = 2
                    self.MOVEMENT = 6
                elif key.char == '+':
                    self.yaw += 2
                    self.MODE = 3
                elif key.char == '-':
                    self.yaw -= 2
                    self.MODE = 3
                else:
                    pass
            except AttributeError:
                if key == keyboard.Key.enter:
                    self.MODE = 2
                    self.MOVEMENT = 0
                    # print(f'self.MODE:{self.MODE}')
                elif key == keyboard.Key.up:
                    self.pitch += 2
                    self.MODE = 3
                elif key == keyboard.Key.down:
                    self.pitch -= 2
                    self.MODE = 3
                elif key == keyboard.Key.left:
                    self.roll += 2
                    self.MODE = 3
                elif key == keyboard.Key.right:
                    self.roll -= 2
                    self.MODE = 3
                else:
                    print('special key {0} pressed'.format(
                        key))
        def on_release(key):
            if key == keyboard.Key.esc:
                print("Listner STOPPED!")
                self.ESC = 1
                return False
        # Starts listening in a non-blocking fashion:
        listener = keyboard.Listener(
            on_press=on_press,
            on_release=on_release)
        listener.start()
        def trajectory(t):
            X = 0
            if t >= 4:
                t = t%4
            if t == 0:
                Z = 0
                Y = step_length
            elif t == 1:
                Z = 0
                # Y = step_length
                Y = 0
                # Y = -step_length
            elif t == 2:
                Z = 0
                Y = -step_length
            elif t == 3:
                Z = -step_height
                Y = 0
            return X,Y,Z
        
        self.enable_torque()
        while not self.ESC:
            # print(self.MODE)
            if self.MODE == 0:
                self.sit()
                time_changed = 0
                self.MODE = 4
            elif self.MODE == 1:
                self.set_time(300)
                self.stand(self.walkingHeight)
                time_changed = 0
                self.MODE = 4
            elif self.MODE == 2:
                if self.MOVEMENT == 1:
                    x1,y1,z1 = trajectory(t+2)
                    x2,y2,z2 = trajectory(t)
                    x3,y3,z3 = trajectory(t)
                    x4,y4,z4 = trajectory(t+2)
                elif self.MOVEMENT == 2:
                    x1,y1,z1 = trajectory(t+2)
                    x2,y2,z2 = trajectory(t)
                    x3,y3,z3 = trajectory(t)
                    x4,y4,z4 = trajectory(t+2)
                    y1,y2,y3,y4 = -y1,-y2,-y3,-y4
                elif self.MOVEMENT == 3:
                    y1,x1,z1 = trajectory(t+2)
                    y2,x2,z2 = trajectory(t)
                    y3,x3,z3 = trajectory(t)
                    y4,x4,z4 = trajectory(t+2)
                elif self.MOVEMENT == 4:
                    y1,x1,z1 = trajectory(t+2)
                    y2,x2,z2 = trajectory(t)
                    y3,x3,z3 = trajectory(t)
                    y4,x4,z4 = trajectory(t+2)
                    x1,x2,x3,x4 = -x1,-x2,-x3,-x4
                elif self.MOVEMENT == 0:
                    x1,y1,z1 = trajectory(t+2)
                    x2,y2,z2 = trajectory(t)
                    x3,y3,z3 = trajectory(t)
                    x4,y4,z4 = trajectory(t+2)
                    y1,y2,y3,y4 = 0,0,0,0
                elif self.MOVEMENT == 5:
                    y1,x1,z1 = trajectory(t+2)
                    y2,x2,z2 = trajectory(t)
                    y3,x3,z3 = trajectory(t)
                    y4,x4,z4 = trajectory(t+2)
                    x1,x2 = -x1,-x2
                elif self.MOVEMENT == 6:
                    y1,x1,z1 = trajectory(t+2)
                    y2,x2,z2 = trajectory(t)
                    y3,x3,z3 = trajectory(t)
                    y4,x4,z4 = trajectory(t+2)
                    x3,x4 = -x3,-x4
                self.move(self.move_xyz([offset_x+x1,  offset_y+y1,   offset_z+z1+self.tilt,
                                        offset_x+x2,    offset_y+y2,   offset_z+z2-self.tilt,
                                        offset_x+x3,   -offset_y+y3,   offset_z+z3+self.tilt,
                                        offset_x+x4,   -offset_y+y4,   offset_z+z4-self.tilt+2]))
                if not time_changed:
                    self.set_time(step_time)
                    time_changed = 1
                t += 1
            elif self.MODE == 3:
                self.set_time(300)
                r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4 = self.rotation(self.roll,self.pitch,self.yaw)
                offset_z = self.walkingHeight
                self.move(self.move_xyz([+r_x1,   +r_y1,  offset_z+r_z1,
                                +r_x2,   +r_y2,   offset_z+r_z2,
                                +r_x3,   +r_y3,   offset_z+r_z3,
                                +r_x4,   +r_y4,   offset_z+r_z4]))
                time_changed = 0
                self.MODE = 4
            else:
                pass
        self.sit()
        print("Done")

    def rotation(self, r = 0, p = 0, y = 0):
        r_x1,r_y1,r_z1 = 0,0,0
        r_x2,r_y2,r_z2 = 0,0,0
        r_x3,r_y3,r_z3 = 0,0,0
        r_x4,r_y4,r_z4 = 0,0,0
        height = 275
        b = 321.5
        l = 601.5
        r,p,y = radians(r),radians(p),-radians(y)
        # left +ve Yaw roll bot k respect mai
        # up +ve Pitch bot k respect mai
        
        
        # pitch
        h = height - l*sin(p)/2
        H = height + l*sin(p)/2

        r_y1 = -H*sin(p)
        r_y2 = -H*sin(p)
        r_y3 = -h*sin(p)
        r_y4 = -h*sin(p)

        r_z1 = H*cos(p) - height
        r_z2 = H*cos(p) - height
        r_z3 = h*cos(p) - height
        r_z4 = h*cos(p) - height

    # roll
        h = height - b*sin(r)/2
        H = height + b*sin(r)/2

        r_x1 += -h*sin(r)
        r_x2 += -H*sin(r)
        r_x3 += -h*sin(r)
        r_x4 += -H*sin(r)

        r_z1 += h*cos(r) - height
        r_z2 += H*cos(r) - height
        r_z3 += h*cos(r) - height
        r_z4 += H*cos(r) - height

    # yaw
    # Use only when X = 0 | Y = 0
        R = 341
        X = 341*cos(1.079933399+y) - (321.5/2)
        Y = 341*sin(1.079933399+y) - (601.5/2)
        r_x2 += X
        r_y2 += Y
        r_z2 += 0
        
        r_x3 += -X
        r_y3 += -Y
        r_z3 += 0
        X = 341*cos(1.079933399-y) - (321.5/2)
        Y = 341*sin(1.079933399-y) - (601.5/2)
        
        r_x1 += -X
        r_y1 += Y
        r_z1 += 0
        
        r_x4 += X
        r_y4 += -Y
        r_z4 += 0
        return r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4

    def moving_threshold(self, value):
        self.motor.dxl_moving_threshold(value)

    def equalize(self,null=0):
        Kp = 0.1
        T = self.get_forces()
        if not null == 0:
            T[null-1] = 0
            
        avg = np.mean(T)
        error = np.array(T) - avg
        if not null == 0:
            error[null-1] = 0
            
        delta = error*Kp
        self.move(self.move_xyz([self.x1,self.y1,self.z1-delta[0],
                                 self.x2,self.y2,self.z2-delta[1],
                                 self.x3,self.y3,self.z3-delta[2],
                                 self.x4,self.y4,self.z4-delta[3]]))
        print(self.x1,self.y1,self.z1,self.x2,self.y2,self.z2,self.x3,self.y3,self.z3,self.x4,self.y4,self.z4)

    
    


'''
[-50, 23]
[-178, 10]
[-3, 178]

[-23, 50],
[-10, 178],
[-178, 3],

7: [-23, 50]
8: [-10, 178]
9: [-178, 3]

10: [-50, 23]
11: [-178, 10]
12: [-3, 178]
'''

'''Motor failure at
{'1': 10.8, '2': 10.9, '3': 10.6, '4': 10.9, '5': 10.9, '6': 10.9, '7': 10.8, '8': 10.8, '9': 10.8, '10': 10.9, '11': 10.8, '12': 10.3}
{'1': 0.13, '2': 0.42, '3': 2.6, '4': 0.0, '5': 0.51, '6': 0.07, '7': 0.01, '8': 0.26, '9': 0.08, '10': 0.11, '11': 0.17, '12': 3.29}

{'1': 10.8, '2': 10.8, '3': 10.4, '4': 10.9, '5': 10.9, '6': 10.9, '7': 10.8, '8': 10.8, '9': 10.8, '10': 10.9, '11': 10.8, '12': 10.5}
{'1': 0.13, '2': 0.41, '3': 2.6, '4': 0.0, '5': 0.52, '6': 0.08, '7': 0.01, '8': 0.27, '9': 0.08, '10': 0.11, '11': 0.18, '12': 3.29}

{'1': 10.8, '2': 10.8, '3': 10.4, '4': 10.9, '5': 10.9, '6': 10.9, '7': 10.9, '8': 10.8, '9': 10.8, '10': 10.9, '11': 10.8, '12': 10.2}
{'1': 0.13, '2': 0.42, '3': 2.59, '4': 0.0, '5': 0.52, '6': 0.06, '7': 0.01, '8': 0.27, '9': 0.08, '10': 0.11, '11': 0.17, '12': 3.3}

{'1': 10.8, '2': 10.8, '3': 10.5, '4': 10.9, '5': 10.9, '6': 10.8, '7': 10.8, '8': 10.8, '9': 10.8, '10': 10.8, '11': 10.8, '12': 10.2}
{'1': 0.13, '2': 0.42, '3': 2.59, '4': 0.01, '5': 0.52, '6': 0.01, '7': 0.0, '8': 0.28, '9': 0.09, '10': 0.11, '11': 0.17, '12': 3.39}

{'1': 10.8, '2': 10.7, '3': 10.3, '4': 10.9, '5': 10.8, '6': 10.8, '7': 10.8, '8': 10.8, '9': 10.8, '10': 10.8, '11': 10.7, '12': 10.4}
{'1': 0.13, '2': 0.42, '3': 2.57, '4': 0.0, '5': 0.52, '6': 0.05, '7': 0.0, '8': 0.27, '9': 0.07, '10': 0.08, '11': 0.18, '12': 3.57}

{'1': 10.8, '2': 10.7, '3': 10.3, '4': 10.8, '5': 10.8, '6': 10.8, '7': 10.8, '8': 10.7, '9': 10.7, '10': 10.7, '11': 10.7, '12': 10.0}
{'1': 0.27, '2': 0.43, '3': 2.53, '4': 0.0, '5': 0.51, '6': 0.05, '7': 0.0, '8': 0.28, '9': 0.07, '10': 0.02, '11': 0.22, '12': 3.78}

{'1': 10.7, '2': 10.7, '3': 10.4, '4': 10.7, '5': 10.6, '6': 10.7, '7': 10.7, '8': 10.6, '9': 10.6, '10': 10.7, '11': 10.6, '12': 10.0}
{'1': 0.29, '2': 0.44, '3': 2.52, '4': 0.0, '5': 0.5, '6': 0.02, '7': 0.0, '8': 0.27, '9': 0.08, '10': 0.03, '11': 0.24, '12': 3.98}

{'1': 10.6, '2': 10.6, '3': 10.2, '4': 10.7, '5': 10.6, '6': 10.6, '7': 10.6, '8': 10.6, '9': 10.6, '10': 10.6, '11': 10.5, '12': 9.7}
{'1': 0.27, '2': 0.45, '3': 2.49, '4': 0.0, '5': 0.5, '6': 0.0, '7': 0.0, '8': 0.29, '9': 0.02, '10': 0.08, '11': 0.38, '12': 1.11}

{'1': 11.3, '2': 11.4, '3': 11.4, '4': 11.5, '5': 11.5, '6': 11.5, '7': 11.4, '8': 11.4, '9': 11.0, '10': 11.4, '11': 11.3, '12': 11.4}
{'1': 0.03, '2': 0.58, '3': 0.04, '4': 0.23, '5': 0.4, '6': 2.34, '7': 0.34, '8': 0.56, '9': 2.98, '10': 0.0, '11': 0.42, '12': 0.0}

Motor failed: 12
'''