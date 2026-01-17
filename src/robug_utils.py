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

import copy

class v3:

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    def __repr__(self):
        return '[ x: {:.3f}, y: {:.3f}, z: {:.3f} ]'.format(self.x, self.y, self.z)

    def add(self, A):
        self.x += A.x
        self.y += A.y
        self.z += A.z
        return(self)

    def sub(self, A):
        self.x -= A.x
        self.y -= A.y
        self.z -= A.z
        
    def mult(self, k):
        self.x = self.x * k
        self.y = self.y * k
        self.z = self.z * k 

    def set(self, A):
        self.x = A.x
        self.y = A.y
        self.z = A.z
        
    def set_from_list(self, L):
        self.x = L[0]
        self.y = L[1]
        self.z = L[2]       
    
    def to_list(self):
        return [self.x, self.y, self.z]
    
    # micropython says: not deepcopy-able
    # works in cpython -> investigate
#     def copy(self):
#         return copy.deepcopy(self)    
