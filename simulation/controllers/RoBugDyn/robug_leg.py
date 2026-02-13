# Copyright (c) 2026 RobotsForAll
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

from math import degrees, pi, sin
from time import sleep

from robug_constants_dyn import constants as c
from robug_utils import v3
from robug_ik import rbik
from robug_gait import rbgait
from robug_joints_sim import rbjoints

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
    
    def solve(self):
        # print(self.ID, ' solve: ', self.foot_pos)
        self.delta, self.gamma = self.ik.solve(self.foot_pos)
        self.deltaTicks = self.rad2ticks(self.delta)
        self.gammaTicks = self.rad2ticks(self.gamma)
        
    def solve_ik(self, pos):
        self.foot_pos.set(pos)
        self.solve()
        
    def calculate_foot_position(self, bAbs):
        # calculate new z value
        pos, _ = self.gait.calc_substep('z', bAbs)
        # calculate new x value and add to xyz
        pos, _ = self.gait.calc_substep('x', bAbs)
        # add constant offset
        self.foot_pos.set(pos)
        self.foot_pos.add(c._GAIT_FOOT_OFFSET[self.gait.ID])        
    
    def calculate_joint_angles(self, bAbs):
        # calculate new foot position (incl. offset)
        self.calculate_foot_position(bAbs)
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

