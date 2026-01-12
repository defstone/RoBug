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

from math import pi, sin
from robug_utils import v3
from robug_constants import constants as c

############################
## class rbgait
############################
class rbgait:

    def __init__(self, iLegID):

        # gait generator parameters
        self.ID   = iLegID
        self.name = c._GAIT_NAME[iLegID]

        # leg trajectory definition
        self.zmin  = c._GAIT_HEIGHT
        self.zmax  = c._GAIT_HEIGHT + c._GAIT_SWING_AMPL
        self.zapex = c._GAIT_SWING_AMPL
        self.zampl = c._GAIT_SWING_AMPL
        self.xmin  = c._GAIT_HALF_STRIDE * -1
        self.xmax  = c._GAIT_HALF_STRIDE
        self.xreach = c._GAIT_HALF_STRIDE * 2
        self.ifwd   = c._GAIT_SUPPORT_TICKS
        self.irtn   = c._GAIT_SWING_TICKS
        self.xyzImpulse = c._GAIT_PUSH_STRENGTH

        # internal loop counter
        self.i = int(c._GAIT_SUPPORT_TICKS/2)
        self.ioffset = 0
        self.incr  = c._GAIT_LOOP_INC

        # internal state
        self.phase = 0
        self.xyz = v3(0.0, 0.0, 0.0)
        self.bPush = True

        # start / stop / direction
        self.dirX = c._LEG_DIR[iLegID]
        self.dirZ = 1

        # internal book keeping
        self.ifwd_base = c._GAIT_SUPPORT_TICKS
        self.irtn_base = c._GAIT_SWING_TICKS
        self.state = 'move'

        # set up dependent variables
        self.calc_parameters_fullstep()

    def calc_parameters_fullstep_common(self):
        self.ifwd  = self.ifwd_base
        self.irtn  = self.irtn_base
        self.substeps = self.ifwd + self.irtn
        self.ifwd_mid = (self.ifwd/2)
        self.irtn_mid = (self.ifwd + (self.irtn/2))
        self.cycle_mid = self.substeps/2

    def calc_parameters_halfstep_common(self):
        self.ifwd  = self.ifwd_base
        self.irtn  = self.irtn_base/2
        self.substeps = self.ifwd + self.irtn
        self.ifwd_mid = (self.ifwd/2)
        self.irtn_mid = (self.ifwd + (self.irtn/2))
        self.cycle_mid = self.substeps/2        

    def calc_parameters_fullstep(self):
        self.calc_parameters_fullstep_common()
        self.xreach = self.xmax -self.xmin
        self.dxfwd = self.xreach / self.ifwd
        self.dxrtn = self.xreach / self.irtn
        self.az = 0
        self.daz = pi/self.irtn
        self.ioffset = 0

    def calc_parameters_halfstep_end2center(self):
        self.calc_parameters_halfstep_common()
        self.xreach = (self.xmax -self.xmin)/2
        self.dxfwd = (self.xmax -self.xmin) / self.ifwd
        self.dxrtn = self.xreach / self.irtn
        self.az = 0        
        self.daz = pi/self.irtn
        self.ioffset = 0

    def calc_parameters_halfstep_center2start(self):
        self.calc_parameters_halfstep_common()
        self.xreach = (self.xmax -self.xmin)/2
        self.dxfwd = (self.xmax -self.xmin) / self.ifwd
        self.dxrtn = self.xreach / self.irtn
        self.az = 0
        self.daz = pi/self.irtn
        self.ioffset = self.irtn
        
    def set_dxfwd(self, dl, di):
        self.dxfwd = dl / di
        
    def set_dxrtn(self, dl, di):
        self.dxtrn = dl / di
        
    def set_az(self, rad):
        self.az = rad
        
    def set_daz(self, di):
        self.daz = pi/di
        
    def get_xreach(self):
        return self.xreach

    def stepmode(self, strStepType):
        if strStepType == 'full':
            self.calc_parameters_fullstep()
        elif strStepType == 'half_start':
            self.calc_parameters_halfstep_center2start()
        elif strStepType == 'half_stop':
            self.calc_parameters_halfstep_end2center()
        else:
            print('unknown step type')
            exit()

    def calc_substep_X_abs(self):
        # calc x(i) independent from history
        if self.i == 0:
            self.xyz.x = self.xmax * self.dirX
        elif self.i > 0 and self.i < self.ifwd:
            self.xyz.x = (self.xmax - (self.dxfwd * self.i)) * self.dirX
        elif self.i == self.ifwd:
            self.xyz.x = self.xmin * self.dirX
        elif self.i > self.ifwd and self.i < self.substeps:
            self.xyz.x = (self.xmin + ((self.i - self.ifwd) * self.dxrtn)) * self.dirX
        else:
            print('error - unkown gait loop phase in calc_substep_X\n')
            
    def calc_substep_X_rel(self):
        # swing phase of other leg pair is from P0 to P1
        iP0 = self.cycle_mid-self.irtn        
        iP1 = self.cycle_mid
        # calc x(i) by increment / decrement
        if self.i == 0:
            self.xyz.x = self.xmax * self.dirX
        elif self.i > 0 and self.i < self.ifwd:
            # speed up x-movement while swing legs don't have contact
            # -> curve path
            if self.i > iP0 and self.i < iP1: k = c._GAIT_FWD_GAIN[self.ID]
            else: k =1.0
            self.xyz.x = self.xyz.x - (self.dxfwd * k * self.dirX)  
        elif self.i == self.ifwd:
            self.xyz.x = self.xyz.x - (self.dxfwd * self.dirX)
            self.dxrtn = (self.xmax - (self.xyz.x * self.dirX)) / self.irtn
        elif self.i > self.ifwd and self.i < self.substeps:
            self.xyz.x = self.xyz.x + (self.dxrtn * self.dirX)
        else:
            print('error - unkown gait loop phase in calc_substep_X\n')
            
    def calc_substep_X(self,bAbs):
        if bAbs: self.calc_substep_X_abs()
        else: self.calc_substep_X_rel()            
            
    def calc_substep_Z_abs(self):
        # calc az(i) independent from history
        if self.i == 0:
            self.xyz.z = self.zmin
            self.az = 0
        elif self.i > 0 and self.i < self.ifwd:
            self.xyz.z = self.zmin
            self.az = 0            
        elif self.i == self.ifwd:
            self.xyz.z = self.zmin
            self.az = 0            
        elif self.i > self.ifwd and self.i < self.substeps:
            self.az = self.daz * (self.i - self.ioffset - self.ifwd)
            self.xyz.z = self.zmin + (self.zampl * sin(self.az))
        else:
            print('error - unkown gait loop phase in calc_substep_Z\n')
        if self.bPush: self.calc_substep_zpush()            
            
    def calc_substep_Z_rel(self):
        # calc az(i) by increment / decrement        
        if self.i == 0:
            self.xyz.z = self.zmin
            self.az = 0
        elif self.i > 0 and self.i < self.ifwd:
            self.xyz.z = self.zmin
            self.az = 0            
        elif self.i == self.ifwd:
            self.xyz.z = self.zmin
            self.az = 0            
        elif self.i > self.ifwd and self.i < self.substeps:
            self.az = self.az + self.daz
            self.xyz.z = self.zmin + (self.zampl * sin(self.az))
        else:
            print('error - unkown gait loop phase in calc_substep_Z\n')
        if self.bPush: self.calc_substep_zpush()
        
    def calc_substep_Z(self,bAbs):
        if bAbs: self.calc_substep_Z_abs()
        else: self.calc_substep_Z_rel()

    def calc_substep_zpush(self):
        # swing phase of other leg pair is from P0 to P1
        iP0 = self.cycle_mid-self.irtn        
        iP1 = self.cycle_mid
        # normal = no additional push
        iTmpZ = 0
        if self.i > 0 and self.i <= self.ifwd:
            # while swing legs don't support body push harder to carry load   
            if self.i >= iP0 and self.i < iP1:
                iTmpZ = self.xyzImpulse.z
            # keep push after swing legs touch down to mitigate impact  
            elif self.i >= iP1 and self.i < iP1 + c._GAIT_PUSH_OVERLAP:
                iTmpZ = self.xyzImpulse.z/c._GAIT_OVERLAP_PUSH_FACTOR
        elif self.i < 0 or self.i > self.substeps:
            print('error - unkown gait loop phase in calc_substep_push\n')
        self.xyz.z = self.xyz.z + iTmpZ

    def calc_substep(self, strAxis, bAbs):
        if strAxis == 'x':
            self.calc_substep_X(bAbs)
            phase = self.phase
        elif strAxis == 'z':
            self.calc_substep_Z(bAbs)
            phase = self.phase
        return self.xyz, phase

    def loop_inc(self):
        self.i += self.incr
        if self.i < 0: self.i = (self.substeps-1)
        elif self.i >= self.substeps: self.i = 0
        if self.i == 0:
            self.phase = 1
        elif self.i > 0 and self.i < self.ifwd:
            self.phase = 2
        elif self.i == self.ifwd:
            self.phase = 3
        elif self.i > self.ifwd and self.i < self.substeps:
            self.phase = 4
        else:
            print('error - unkown gait loop phase in calc_substep_X\n')
            self.phase = 5

    def is_loop_frame(self, strKey):
        # mode
        if   strKey == 'support_start': a = [1]
        elif strKey == 'support_end':   a = [3]
        elif strKey == 'support_phase': a = [2]
        elif strKey == 'swing_phase':   a = [4]
        elif strKey == 'support_mid': return self.is_support_mid()
        # checking for keyframe
        if int(self.phase) in a: return True
        else: return False

    def is_support_mid(self):
        if self.i == self.ifwd_mid: return True
        else: return False

    def set_direction(self, dir, strAxis):
        if   strAxis == 'x': self.dirX = dir
        elif strAxis == 'z': self.dirZ = dir

    def set_loop_counter(self, i):
        self.i = i
      
    def get_loop_counter(self):
        return self.i

    def get_xyz(self):
        return self.xyz

    def set_xyz(self, xyz):
        self.xyz.set(xyz)
        
    def push_enable(self):
        self.bPush = True
        
    def push_disable(self):

        self.bPush = False
