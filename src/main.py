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
import random
from collections import deque
from machine import Pin, PWM
from robug_utils import v3
from robug_constants import constants as c
from robug_robot import robug
from robug_mocon import rbmocon

# send commands to motion controller
# wait for 'DONE' message from motion controller
async def wait_for_reply():   
    while True:
        if len(RplyQueue) > 0:
            strRply = RplyQueue.pop()
            print(f'client received reply: {strRply}')
            return strRply
        await asyncio.sleep_ms(100)
        
async def send_cmd(strCMD):
    global MsgQueue
    MsgQueue.append(strCMD)
    print(f'supervisor sending {strCMD}')
    reply = await wait_for_reply()
    if reply == 'DONE': return True
    else: return False

# LED task
# smoothly fade between LEDs
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

# super simple supervisor task
# wait for top touch sensor
# walk for 4s
#repeat
async def walk():
    while True:
        # if r.touch_top():
        if True:            
            # init Robug
            await send_cmd('RESUME_FWD')
            await asyncio.sleep(30)
            await send_cmd('PAUSE')
            break
        await asyncio.sleep_ms(50)
         
async def main():
    # start any sensor related tasks that need to run concurrently
    task1 = asyncio.create_task(pulse_leds())
    # start the motion controller
    task2 = asyncio.create_task(m.run())
    # start supervisor
    task3 = asyncio.create_task(walk())
    await task3
    task1.cancel()
    task2.cancel()

if __name__ == "__main__":
    
    # set up time slice 
    fLoop = c._GAIT_LOOP_TIME
    
    # set up RoBug
    r = robug()
    r.set_loop_counter_resume()
    r.instant_update()
    sleep(1)
    
    # set up motion controller
    MsgQueue  = deque([], 4)
    RplyQueue = deque([], 4)
    m = rbmocon(r, MsgQueue, RplyQueue)
    
    # start tasks
    touch_top = False
    asyncio.run(main())
    
    # clean up and shut down
    r.set_brightness_red(0)
    r.set_brightness_grn(100)
    
    # r.deinit() 

