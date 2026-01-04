from machine import Pin, PWM
from math import sqrt, asin, acos, degrees, pi
from time import sleep
from robug_utils import v3
from robug_constants import constants as c

class rbik:
    
    # leg frame:
    # pos x -> fwd
    # pos y -> left
    # pos z -> up
    
    def __init__(self):
        self.pos = v3()
    
    def solve(self, pos):

        l1sqr  = c._L_FEMUR_PWR2
        l2sqr  = c._L_TIBIA_PWR2
        l1_x2  = c._L_FEMUR_X2
        l1l2x2 = c._L_FEMUR_X_TIBIA_X2
        l3sqr  = pos.x**2 + pos.z**2
        l3     = sqrt(l3sqr)
        l1l3x2 = l1_x2 *  l3

        fAlpha = asin(pos.x/l3)
        fBeta  = acos(-( l2sqr - l1sqr - l3sqr) / l1l3x2 )
        # tibia
        fGamma = acos(-( l3sqr - l1sqr - l2sqr) / l1l2x2 )
        # femur
        fDelta = pi-fBeta-fAlpha

        # for debug, print joint angels
        # print('femur angle: ', degrees(fDelta))
        # print('tibia angle: ', degrees(fGamma))
        
        # update position variable
        self.pos = pos
        return fDelta, fGamma
