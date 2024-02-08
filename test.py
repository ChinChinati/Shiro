# from doggy import Doggy
# import pygame
# from time import *
# shiro = Doggy()
# import pygame

# shiro.connect()
# shiro.pid(1500,800,500)#best till 03.12.2023
# shiro.enable_torque()
# shiro.set_time(2000)

# # #stand
# i = 250
# shiro.move(
#     shiro.move_xyz([0, 0, 240.29325, 0, 0, 251.17325, 0, 0, 253.20225, 0, 0, 249.33125]))
# # shiro.set_time(2000)
# # shiro.sit()
# shiro.controller()
import numpy as np
one = np.array([2,1,2,1,0])
zero = np.ones(5)
print(one-zero)