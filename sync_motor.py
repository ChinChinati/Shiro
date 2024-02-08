#!/usr/bin/env python

import os
from numpy import interp
from dynamixel_sdk import *
import pty
import sys, tty, termios
pid = pty.fork()
fd = sys.stdin.fileno()
if not pid:
    old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# Control table address
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE          = 64
    ADDR_GOAL_POSITION          = 116
    LEN_GOAL_POSITION           = 4         # Data Byte Length
    ADDR_PRESENT_POSITION       = 132
    LEN_PRESENT_POSITION        = 4         # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = 2048        # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 2048      # Refer to the Maximum Position Limit of product eManual
    DXL_IS_MOVING               = 122
    DXL_MOVING_STATUS           = 123
    DXL_CHECK_SAFE              = 70
    DXL_GET_CURRENT             = 126
    DXL_GET_VOLTAGE             = 144
    BAUDRATE                    = 3000000
    ADDR_PROFIOLE_VELOCITY      = 112
    ADDR_PROFIOLE_ACCELERATION  = 108
    ADDR_DRIVE_MODE             = 10
    TIME_BASED_PROFILE          = 4
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    LEN_GOAL_POSITION           = 4
    ADDR_PRESENT_POSITION       = 611
    LEN_PRESENT_POSITION        = 4
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    LEN_GOAL_POSITION           = 4          # Data Byte Length
    ADDR_PRESENT_POSITION       = 580
    LEN_PRESENT_POSITION        = 4          # Data Byte Length
    DXL_MINIMUM_POSITION_VALUE  = -150000    # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000     # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600

# DYNAMIXEL Protocol Version (1.0 / 2.0)
PROTOCOL_VERSION            = 2.0

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB1'

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 30               # Dynamixel moving status threshold



class Motor(object):
    dxl_present_position = {}

    def __init__(self,
        baud_rate:int = BAUDRATE,
        port: str = DEVICENAME):

        self.portHandler = PortHandler(port)
        # except:
        #     self.portHandler = PortHandler('/dev/ttyUSB1')
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.baud_rate = baud_rate
        # Initialize self.groupSyncWrite instance
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Initialize self.groupSyncRead instace for Present Position
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        if self.portHandler.setBaudRate(self.baud_rate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()


    def open_port(self):
            # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def close_port(self):
        # Close port
        self.portHandler.closePort()


    def torque_enable(self, arr):
        # Clear syncread parameter storage
        self.groupSyncRead.clearParam()
        for i in arr:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d has been successfully connected" % i)
            # Add parameter storage for Dynamixel present position value
            dxl_addparam_result = self.groupSyncRead.addParam(i)
            if dxl_addparam_result != True:
                print("[ID:%03d] self.groupSyncRead addparam failed" % i)
                quit()


    def torque_disable(self, arr):
        for i in arr:
        # Disable Dynamixel Torque
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))


    def scan(self):
        found_ids = []
        for i in range(1, 13):
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, i)
            if dxl_comm_result == COMM_SUCCESS:
                found_ids.append(i)
        return found_ids
    

    #bits is a dict that stores particular motors id and postion 
    def move_bits(self, bits):
        # bits = [bits]
        for id in bits: 
            id = int(id)
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(bits[str(id)])), DXL_HIBYTE(DXL_LOWORD(bits[str(id)])), DXL_LOBYTE(DXL_HIWORD(bits[str(id)])), DXL_HIBYTE(DXL_HIWORD(bits[str(id)]))]

            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] self.groupSyncWrite addparam failed" % id)
                quit()
            # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        flag = []
        for i in range(len(bits)):
            flag.append(0)
        
        while 1:
             # Syncread present position
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                # print('ye')
            for id in bits: 
                id = int(id)
                # Check if self.groupSyncRead data of Dynamixel#1 is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                # print('k')
                if dxl_getdata_result != True:
                    print("[ID:%03d] self.groupSyncRead getdata failed" % id)
                    quit()
                # Get Dynamixel present position value
                self.dxl_present_position[str(id)]= self.groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

                # print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t" % (id, bits[str(id)]))
            i = 0
            for id in bits: 
                id = int(id)
                if not ((abs(bits[str(id)] - self.dxl_present_position[str(id)]) > DXL_MOVING_STATUS_THRESHOLD)):
                    flag[i] = 1
                    i+=1
            if flag[-1]  == 1:
                break
        
    def move_angles(self, angles):
        # angles = [angles]
        for id in angles:
            angles[id] = int(interp(angles[id],[-180,180],[0,4095]))
        for id in angles: 
            id = int(id)
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(angles[str(id)])), DXL_HIBYTE(DXL_LOWORD(angles[str(id)])), DXL_LOBYTE(DXL_HIWORD(angles[str(id)])), DXL_HIBYTE(DXL_HIWORD(angles[str(id)]))]

            # Add Dynamixel goal position value to the Syncwrite parameter storage
            dxl_addparam_result = self.groupSyncWrite.addParam(id, param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] self.groupSyncWrite addparam failed" % id)
                quit()
            # Syncwrite goal position
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        self.groupSyncWrite.clearParam()
        flag = []
        for i in range(len(angles)):
            flag.append(0)
        
        while 1:
             # Syncread present position
            dxl_comm_result = self.groupSyncRead.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            for id in angles: 
                id = int(id)
                # Check if self.groupSyncRead data of Dynamixel#1 is available
                dxl_getdata_result = self.groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                # print('k')
                if dxl_getdata_result != True:
                    print("[ID:%03d] self.groupSyncRead getdata failed" % id)
                    quit()
                # Get Dynamixel present position value
                self.dxl_present_position[str(id)]= self.groupSyncRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

                # print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t" % (id, angles[str(id)]))
            i = 0
            for id in angles: 
                id = int(id)
                if not ((abs(angles[str(id)] - self.dxl_present_position[str(id)]) > DXL_MOVING_STATUS_THRESHOLD)):
                    flag[i] = 1
                    i+=1
            if flag[-1]  == 1:
                break

    def get_position_bits(self):
            return self.dxl_present_position

    def get_position_angles(self,arr):
            dxl_present_position = self.get_position_bits()
            for key,value in dxl_present_position.items(): 
                dxl_present_position[str(key)]= round(interp(value,[0,4095],[-180,180]),4)
            return dxl_present_position
    
    def reset_all(self):
        bits = {}
        ids = self.scan()
        for id in ids:
            bits[str(id)] = 2040
            print(id)
        # self.torque_disable(ids)
        self.torque_enable(ids)
        # print(angles)
        self.move_bits(bits)
    
    def reset(self, arr):
        bits = {}
        for id in arr:
            bits[str(id)] = 2040
            print(id)
        # print(angles)
        self.torque_enable(arr)
        self.move_bits(bits)

    def moving(self,arr):
        dxl_moving = {}
        # print(f'position :', end = ' ')
        for id in arr: 
            # Get Dynamixel present position value
            bits = self.packetHandler.read4ByteTxRx(self.portHandler, id, DXL_IS_MOVING)
            if bits[0] == 256:
                dxl_moving[str(id)] = 0
            else:
                dxl_moving[str(id)] = 1
        return dxl_moving


    def moving_status(self,arr):
        dxl_moving = {}
        # print(f'position :', end = ' ')
        for id in arr: 
            # Get Dynamixel present position value
            bits = self.packetHandler.read4ByteTxRx(self.portHandler, id, DXL_MOVING_STATUS)
            if bits[0] == 256:
                dxl_moving[str(id)] = 0
            else:
                dxl_moving[str(id)] = 1
        return dxl_moving

# CHECK LATER: FUNCTION IS RUNNING, WE NEED TO CHECK FOR DIFFERENT ERROR VALUES
    def check_safe(self,arr):
        dxl_safe = {}
        # print(f'position :', end = ' ')
        for id in arr: 
            # Get Dynamixel present position value
            bits = self.packetHandler.read4ByteTxRx(self.portHandler, id, DXL_CHECK_SAFE)
            # if bits[0] == 256:
            #     dxl_safe[str(id)] = 0
            # else:
            #     dxl_safe[str(id)] = 1
            dxl_safe[str(id)]= bits
        return dxl_safe


# CHECK LATER: MAP CURRENT VALUES TO GET ACTUAL AMPERES
    def get_current(self, arr):
        dxl_present_current = {}
        # print(f'position :', end = ' ')
        for id in arr: 
            bits = self.packetHandler.read2ByteTxRx(self.portHandler, id, DXL_GET_CURRENT)
            if bits[0] < 32767:
                dxl_present_current[str(id)] = round(bits[0]*.00269 ,2)
            else:
                dxl_present_current[str(id)] = round((65536 - bits[0])*.00269 ,2)
        return dxl_present_current
    
    def get_voltage(self,arr):
        dxl_present_voltage = {}
        for id in arr: 
            bits = self.packetHandler.read2ByteTxRx(self.portHandler, id, DXL_GET_VOLTAGE)
            dxl_present_voltage[str(id)] = round(bits[0]*0.1 ,2)
        return dxl_present_voltage

    def profile_velocity(self, dict):
        for id in dict:
        # Set Dynamixel profile velocity
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, int(id), ADDR_PROFIOLE_VELOCITY, dict[id])
            if dxl_comm_result != COMM_SUCCESS:
                continue
                # print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                continue
                # print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def profile_acceleration(self, dict):
        for id in dict:
        # Set Dynamixel profile acceleration
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, int(id), ADDR_PROFIOLE_ACCELERATION, dict[id])
            if dxl_comm_result != COMM_SUCCESS:
                continue
                # print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                continue
                # print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    #
    def drive_mode_time_based_profile(self,arr):
            for id in arr:
            # Set Dynamixel profile velocity
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, int(id), ADDR_DRIVE_MODE, TIME_BASED_PROFILE)
                if dxl_comm_result != COMM_SUCCESS:
                    continue
                    # print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    continue
                    # print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def p(self,gain,arr):
            for id in arr:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, int(id), 84, gain)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def i(self,gain,arr):
            for id in arr:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, int(id), 82, gain)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def d(self,gain,arr):
            for id in arr:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, int(id), 80, gain)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % self.packetHandler.getRxPacketError(dxl_error))
    
    def dxl_moving_threshold(self, value=20):
        DXL_MOVING_STATUS_THRESHOLD = value

 
