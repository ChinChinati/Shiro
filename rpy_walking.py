from doggy import Doggy
import threading
import time 
shiro = Doggy()
import numpy as np
from math import pi,sin,cos
from numpy import interp
from pygame_visual import *

STEP_TIME = 250 #ms

offset_x = 0
offset_y = 25
offset_z = 250

r_x1,r_y1,r_z1 = 0,0,0
r_x2,r_y2,r_z2 = 0,0,0
r_x3,r_y3,r_z3 = 0,0,0
r_x4,r_y4,r_z4 = 0,0,0

def traj(t):
    if t > 4:
        t = t%4
    X = 0
    if t <= 3:
        Z = 0
        Y = interp(t,[0,3],[50,-50])
    if t > 3 and t <= 4:
        theta = interp(t,[3,4],[0,pi])
        Y = -50*cos(theta)
        Z = -50*sin(theta)
    return X,Y,Z

def rotation(r,p,y):
    global offset_z
    b = 321.5
    r,p,y = radians(r),radians(p),radians(y)
    # pitch
    p = interp(p,[-pi/2,pi/2],[-350,350])
    r_z1 = -p
    r_z2 = -p
    r_z3 = p
    r_z4 = p

    # roll
    h = offset_z - b*sin(r)/2
    H = offset_z + b*sin(r)/2

    r_x1 = -h*sin(r)
    r_x2 = -H*sin(r)
    r_x3 = -h*sin(r)
    r_x4 = -H*sin(r)

    r_z1 += h*cos(r) - offset_z
    r_z2 += H*cos(r) - offset_z
    r_z3 += h*cos(r) - offset_z
    r_z4 += H*cos(r) - offset_z

    # yaw
    # Use only when X = 0 | Y = 0
    theta = 1.079933399
    R = 341
    X = 341*cos(1.079933399+y) - (321.5/2)
    Y = 341*sin(1.079933399+y) - (601.5/2)
    r_x2,r_y2,r_z2 = X,Y,0
    r_x3,r_y3,r_z3 = -X,-Y,0
    X = 341*cos(1.079933399-y) - (321.5/2)
    Y = 341*sin(1.079933399-y) - (601.5/2)
    r_x1,r_y1,r_z1 = -X,Y,0
    r_x4,r_y4,r_z4 = X,-Y,0
    return r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4





x = 0
y = 0
z = 0

i = 1000
t = 0 
while i:

    x1,y1,z1 = traj(t)
    x2,y2,z2 = traj(t+2)
    x3,y3,z3 = traj(t+1)
    x4,y4,z4 = traj(t+3)

    r_x1,r_y1,r_z1,r_x2,r_y2,r_z2,r_x3,r_y3,r_z3,r_x4,r_y4,r_z4 = rotation(0,0,10)
    # print(rotation(90,0,0))

    prev_angle = shiro.move_xyz([offset_x+r_x1+x1,  offset_y+r_y1+y1,   offset_z+r_z1+z1,
                               offset_x+x2+r_x2,    offset_y+y2+r_y2,   offset_z+z2+r_z2,
                               offset_x+x3+r_x3,   -offset_y+y3+r_y3,   offset_z+z3+r_z3,
                               offset_x+x4+r_x4,   -offset_y+y4+r_y4,   offset_z+z4+r_z4])
    t+=0.2 

    x1,y1,z1 = traj(t)
    x2,y2,z2 = traj(t+2)
    x3,y3,z3 = traj(t+1)
    x4,y4,z4 = traj(t+3)

    pygame_interface(shiro.move_xyz([offset_x+r_x1+x1,  offset_y+r_y1+y1,   offset_z+r_z1+z1,
                               offset_x+x2+r_x2,    offset_y+y2+r_y2,   offset_z+z2+r_z2,
                               offset_x+x3+r_x3,   -offset_y+y3+r_y3,   offset_z+z3+r_z3,
                               offset_x+x4+r_x4,   -offset_y+y4+r_y4,   offset_z+z4+r_z4]), 100, prev_angle)
    i-=1
    