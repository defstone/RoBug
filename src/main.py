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
from tof_sensor import vl53l0x

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
        
async def loop_timer(loop_ms):
    await asyncio.sleep_ms(loop_ms)

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
 
# simple room explorer
# walks straight until obstacle detected
# then search for exit by turning in an arbitrary direction
# note that the I2C distance sensor driver is synchronous, i.e. blocking

async def get_distance():
    tmp = 0
    for i in range(3):
        tmp += r.get_distance()
        await asyncio.sleep_ms(50)
    tmp = tmp / 3
    print(tmp)
    return(tmp)

async def stop_to_walk_fwd():
    await send_cmd('START_POSE_FWD')            
    await send_cmd('RESUME_FWD')
    
async def walk_fwd_to_lift_legs():
    await send_cmd('STOP_POSE_FWD')            
    await send_cmd('LIFT_LEGS')
    await asyncio.sleep(1)
    await send_cmd('PURR')
    
async def lifted_legs_to_walk_fwd():
    await send_cmd('PUSH_LEGS')
    await stop_to_walk_fwd()            

async def explorer():
    # initialize random number generator
    random.seed()
    
    # set up variables    
    min_dist = 225
    hysteresis = 100
    keep_direction = False
    avoid = False
    picked_up = False
    dist = 1000.0
    
    # init Robug
    await send_cmd('RESUME_FWD')
    await asyncio.sleep(0.25)
    await send_cmd('STOP_POSE_FWD')
    await asyncio.sleep(1)
    
#     for i in range(5):
#         await send_cmd('LOOK_DOWN')
#         await asyncio.sleep(0.1)        
#         
#     await asyncio.sleep(1)
#     
#     for i in range(10):
#         await send_cmd('LOOK_UP')
#         await asyncio.sleep(0.1)
#         
#     await asyncio.sleep(1)   
#         
#     for i in range(5):
#         await send_cmd('LOOK_DOWN')
#         await asyncio.sleep(0.1)
#         
#     await asyncio.sleep(3)          
    
    # let's go
    while True:
        await stop_to_walk_fwd()
        
        # stop when back and belly touched
        while dist > min_dist:
            if not picked_up:
                dist = await get_distance()
            else:
                dist = 42424242
            
            if (not picked_up) and r.touch_top() and r.touch_bot():
                await walk_fwd_to_lift_legs()
                picked_up = True
                
            if picked_up and (not r.touch_top()) and (not r.touch_bot()):
                await lifted_legs_to_walk_fwd()
                picked_up = False
                
            if not picked_up and (not r.touch_top()) and (not r.touch_bot()):
                number = random.randint(0, 500)
                if number == 5:
                    await walk_fwd_to_lift_legs()
                    sleeptime = random.randint(20,180)
                    print(number, sleeptime)
                    await asyncio.sleep(sleeptime)
                    await lifted_legs_to_walk_fwd()
                    
            await asyncio.sleep(0.20)
                
        print(dist)
        avoid = True        
        await send_cmd('STOP_POSE_FWD')
        await asyncio.sleep(0.25)
                
        if keep_direction == False:
            dir = random.randint(0,1)
            
        while avoid:
            
            # turn a little bit
            for i in range(4):
                if dir == 0:
                    await send_cmd('TURN_LFT')
                else:
                    await send_cmd('TURN_RGT')
                    
            # take distance measurement (avg of 3)
            dist = await get_distance()

            # decide 
            if dist >= min_dist + hysteresis:
                if dist >= 380:
                    keep_direction = False
                else:
                    keep_direction = True
                avoid = False
            
            await asyncio.sleep(0.25)            
           
async def main():
    
    # start the motion controller
    task1 = asyncio.create_task(m.run())

    # start any sensor related tasks that need to run concurrently
    task2 = asyncio.create_task(pulse_leds())
    
    # start the supervisor (example above). This could be a task that
    # handles bluetooth remote control, a task to control RoBug depending
    # on sensor readings etc.
    # The supervisor/exlporer task sends commands through the MsgQueue to the
    # RoBug motion control message server (MCMS). The MCMS will reply with
    # 'DONE' when a command completed. It will only look for new commands
    # after completion of the previous command
    task4 = asyncio.create_task(explorer())
    await task4
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
    asyncio.run(main())
    
    # clean up and shut down
    r.set_brightness_red(0)
    r.set_brightness_grn(100)
    
    # r.deinit() 

