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
import math
import asyncio
from collections import deque
from machine import Pin, PWM

from robug_utils import v3
from robug_constants import constants as c
from robug_robot import robug
from robug_ctrl import rbctrl
from robug_mocon import rbmocon

# --------------------------------------------------------
# sensors etc.
# --------------------------------------------------------

async def pulse_leds():
    # green led starts at duty=100%
    i = 90
    redPct = 0
    grnPct = 0
    while True:
        r.set_brightness_red(redPct)
        r.set_brightness_grn(grnPct)
        i += 1
        if i == 360: i = 0
        grnPct = 50 + math.sin(math.radians(i)) * 50
        redPct = 50 + math.sin(math.radians(i)+math.pi) * 50        
        await asyncio.sleep_ms(7)
           
# --------------------------------------------------------
# application: init -> walk fwd -> stop
# --------------------------------------------------------
def current_pose():
    lTmp = []
    for i in range(4):
        tmp = v3()
        tmp.set(r.lLeg[i].foot_pos)
        lTmp.append(tmp)
    return lTmp

def calc_overlay_pose(lNeutral, lModified):
    for i in range(4):
        lModified[i].sub(lNeutral[i])
        r.lLeg[i].overlay_pose.set(c._GAIT_FOOT_OFFSET[i])
        r.lLeg[i].overlay_pose.add(lModified[i])
     
def reset_overlay_pose():
    for i in range(4):
        r.lLeg[i].overlay_pose.set(c._GAIT_FOOT_OFFSET[i])
       
async def simplest_test():
    
    await rc.init_pose()
    lPose_Neutral = current_pose()
    
    await asyncio.sleep(1)
    
    await rc.shift_com_fwd()
    lPose_Fwd = current_pose()
    calc_overlay_pose(lPose_Neutral, lPose_Fwd)
    
    await rc.start_to_walk_fwd()
    await asyncio.sleep(5)
    
    await rc.stop_fwd()
    await rc.shift_com_bwd()
    reset_overlay_pose()
    

# --------------------------------------------------------
# main
# --------------------------------------------------------                    
if __name__ == "__main__":
    
    async def main():
        
        # start any tasks that need to run concurrently
        task0 = asyncio.create_task(pulse_leds())
        
        # start the motion controller
        task1 = asyncio.create_task(m.run())
        
        # start the motion controller
        task2 = asyncio.create_task(simplest_test())
        await task2
        task0.cancel()
        task1.cancel()        
        
    # ----------------------------
    # initialization
    # ----------------------------   

    # set up RoBug
    r = robug()
    r.set_loop_counter_resume()
    r.instant_update()
    sleep(1)
    
    # set up motion controller
    MsgQueue  = deque([], 4)
    RplyQueue = deque([], 4)
    rc = rbctrl(MsgQueue, RplyQueue)
    m = rbmocon(r, MsgQueue, RplyQueue)
    
    # start tasks
    asyncio.run(main())
    
    # clean up and shut down
    # r.set_brightness_red(0)
    # r.set_brightness_grn(100)

