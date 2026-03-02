from machine import Pin, PWM, Timer
from time import sleep

from robug_utils import v3
from robug_constants import constants as c
from robug_robot import robug

r = robug()

for i in range(4):
    for j in range(2):
        r.lLeg[i].joints.set_angle_sid(j, 0)
        sleep(0.25)
        
