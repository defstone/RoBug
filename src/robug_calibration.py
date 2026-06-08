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

import json

class rbcal:
    
    def __init__(self): 
        with open('robug_calibration.json', 'rt') as f:
            self.calib_data = json.load(f)
            self.servo_offs  = self.calib_data['robug_calibration_data']['servo_offs']
            self.servo_gain = self.calib_data['robug_calibration_data']['servo_gain']            
        
if __name__ == '__main__':
    bInit = False
    if bInit:
        data = {
            'robug_calibration_data':{
                'servo_offs': [  60,  -10,   20,   -0,  -50,  -30,   30,  -25],
                'servo_gain': [1.07, 1.05, 1.03, 1.05, 1.00, 1.00, 1.05, 1.05]
                }
            } 
        with open('robug_calibration.json', 'wt') as f:
            print(json.dumps(data))
            json.dump(data, f)
            
    cal = rbcal()
    print(cal.servo_offs)
    print(cal.servo_gain)