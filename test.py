from doggy import Doggy
import pygame
from time import *
shiro = Doggy()
import pygame
shiro.connect()
shiro.pid(1500,800,500)#better
# shiro.pid(300,500,100)#worse
shiro.enable_torque()
shiro.set_time(2000)

# #stand
i = 250
shiro.move(
         shiro.move_xyz([0, 0, 240.29325, 0, 0, 251.17325, 0, 0, 253.20225, 0, 0, 249.33125]))
shiro.set_time(100)
# shiro.sit()
shiro.client_server_controller()
