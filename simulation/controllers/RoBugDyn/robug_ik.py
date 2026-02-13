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

from math import sqrt, asin, acos, degrees, pi
from time import sleep
from robug_utils import v3
from robug_constants_dyn import constants as c

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

