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

from time import sleep
import math
import json
import asyncio

from robug_constants import constants as c
from robug_robot import robug
from robug_ble import rbble

async def calibration():
    
    while True:  
        code = ble.get_current_code()
        # 0xFF -> idle
        if code != 0xFF:
            if code < 0x80:
                # e.g. code = 41
                # (zero based index)
                # index = 4
                # leg = 2
                # joint of leg = 1 (0 = femur, 1 = tibia)
                # op(eration) = 1 -> inc
                index = int(code / 0x10)
                leg   = int(index / 2)
                joint = index % 2
                op    = code % 0x10
                print('code: ', code, 'index: ', index, 'leg: ', leg, 'joint: ', joint, 'inc/dec: ', op)
                # increment or decrement offset value
                if op == 1:
                    r.lLeg[leg].joints.lServoCal[joint] += 5
                elif op == 0:
                    r.lLeg[leg].joints.lServoCal[joint] -= 5
                # update leg
                r.lLeg[leg].joints.set_angle_sid(joint, 0)
                
            elif code == 0x80:
                lOffs = []
                lGain = []
                for i in range(4):
                    for j in range(2):
                        tmpOffs = r.lLeg[i].joints.lServoCal[j]
                        tmpGain = r.lLeg[i].joints.lServoGain[j]
                        lOffs.append(tmpOffs)
                        lGain.append(tmpGain)
                data = {'robug_calibration_data':{'servo_gain': lGain, 'servo_offs': lOffs, }} 
                with open('robug_calibration.json', 'wt') as f:
                    print(json.dumps(data))
                    json.dump(data, f)
                    
        await asyncio.sleep(0.1)
    
# --------------------------------------------------------
# main
# --------------------------------------------------------                    
if __name__ == "__main__":
    
    async def main():
        
        #start ble task
        task0 = asyncio.create_task(ble.msg_handler())
        task1 = asyncio.create_task(calibration())
        
        await task1
        task0.cancel()

    # ----------------------------
    # initialization
    # ----------------------------
    
    # setup BLE
    ble = rbble()
    ble.set_mode('raw')

    # set up RoBug
    r = robug()
    for i in range(4):
        for j in range(2):
            r.lLeg[i].joints.set_angle_sid(j, 0)
            sleep(0.25) 
    
    # start tasks
    asyncio.run(main())