from math import degrees, pi, sin
from time import sleep

from robug_constants import constants as c
from robug_utils import v3
from robug_ik import rbik
from robug_gait import rbgait
from robug_joints import rbjoints

class rbleg:
    
    def __init__(self, iLegID):
        self.ID = iLegID
        self.name = c._GAIT_NAME[iLegID]
        self.gait = rbgait(iLegID)
        self.ik = rbik()
        self.joints = rbjoints(iLegID)
        self.delta = 0.0
        self.gamma = 0.0
        self.deltaTicks = 0
        self.gammaTicks = 0        
        self.foot_pos = v3()
        
    def rad2ticks(self, rad):
        return (rad-c._PI0P5)/c._SERVO_K        
        
    def inc_loop_counter(self):
        self.gait.loop_inc()

    # 27/12/25
    # separation in slove and solve_ik is new
    # test b4 making this default
    
    def solve(self):
        # print(self.ID, ' solve: ', self.foot_pos)
        self.delta, self.gamma = self.ik.solve(self.foot_pos)
        self.deltaTicks = self.rad2ticks(self.delta)
        self.gammaTicks = self.rad2ticks(self.gamma)
        
    def solve_ik(self, pos):
        self.foot_pos.set(pos)
        self.solve()
    
    def calculate_joint_angles(self, bAbs):
        # calculate new z value
        pos, _ = self.gait.calc_substep('z', bAbs)
        # calculate new x value and add to xyz
        pos, _ = self.gait.calc_substep('x', bAbs)
        # add constant offset - don't write back to gait.xyz via pos!
        self.foot_pos.set(pos)
        self.foot_pos.add(c._GAIT_FOOT_OFFSET[self.gait.ID])      
        # solve ik and store leg position
        self.solve()
        
    def set_joints(self):
        self.joints.set_angles(self.deltaTicks, self.gammaTicks)
        
    def set_position(self, pos):
        self.foot_pos.set(pos)
        self.solve()        
        self.set_joints()
        
    def set_delta(self, delta):
        self.delta = delta
        
    def get_delta(self):
        return self.delta        
        
    def set_gamma(self, gamma):
        self.gamma = gamma
        
    def get_gamma(self):
        return self.gamma         
