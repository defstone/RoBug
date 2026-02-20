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

async def loop_timer(loop_ms):
    await asyncio.sleep_ms(loop_ms)

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
    tmp /= 3
    print(tmp)
    return tmp

async def resume_fwd():
     await send_cmd('RESUME_FWD')
     
async def resume_bwd():
     await send_cmd('RESUME_BWD')     

async def stop_fwd():
     await send_cmd('STOP_POSE_FWD')
     
async def stop_bwd():
     await send_cmd('STOP_POSE_BWD')     

async def start_to_walk_fwd():
    await send_cmd('START_POSE_FWD')            
    await send_cmd('RESUME_FWD')
    
async def start_to_walk_bwd():
    await send_cmd('START_POSE_BWD')            
    await send_cmd('RESUME_BWD')    
    
async def turn(dir, times):
    for i in range(times):
        if dir == -1:
            print('turn left')
            await send_cmd('TURN_LFT')
        else:
            print('turn right')
            await send_cmd('TURN_RGT')
    
async def walk_fwd_to_lift_legs():
    await send_cmd('STOP_POSE_FWD')            
    await send_cmd('LIFT_LEGS')
    await asyncio.sleep(0.5)
    await send_cmd('PURR')
    
async def lifted_legs_to_walk_fwd():
    await send_cmd('PUSH_LEGS')
    await start_to_walk_fwd()
    
async def inspect():
    #print(r.lLeg[0].gait.get_loop_counter(), r.lLeg[0].foot_pos)
    await send_cmd('LOOK_DOWN')
    #print(r.lLeg[0].gait.get_loop_counter(), r.lLeg[0].foot_pos)            
    dist_low = await get_distance()
    await send_cmd('LOOK_UP')
    dist_straight = await get_distance()    
    await send_cmd('LOOK_UP')
    dist_high = await get_distance()
    await send_cmd('LOOK_DOWN')
    # print(r.lLeg[0].gait.get_loop_counter(), r.lLeg[0].foot_pos)
    return dist_low, dist_straight, dist_high

# behavior

async def explorer():
    
    # initialize random number generator
    random.seed()
    # set up variables    
    min_dist = 250.0
    close_combat = 150
    hysteresis = 100.0    
    clear_dist = 300.0
    too_close = 125.0
    panic_level = 0
    frustration = 0
    n = 4

    dirChoice = [-1, 1]
    # await stop_fwd()
    RoBugState = 'init'
    CurrentState = 'init'
    
    while True:
        
        # -------------------------------------------------
        if   RoBugState == 'init':
        # -------------------------------------------------        
        
            await send_cmd('RESUME_FWD')
            await asyncio.sleep(0.25)
            await send_cmd('STOP_POSE_FWD')
            RoBugState = 'waitForActivation'            

        # -------------------------------------------------
        elif RoBugState == 'waitForActivation':
        # -------------------------------------------------
        
            if r.touch_bot():
                await start_to_walk_fwd()
                RoBugState = 'walk'
                
        # -------------------------------------------------
        elif RoBugState == 'walk':
        # -------------------------------------------------
        
            dist = await get_distance()
            if dist < min_dist:
                await stop_fwd()
                await asyncio.sleep(0.25)
                dir = random.choice(dirChoice)                
                if dist <= too_close:
                    RoBugState = 'avoid_phase_3'
                else:
                    dist_scan = []                    
                    RoBugState = 'avoid_phase_1.0'                   
                
        # -------------------------------------------------
        elif RoBugState == 'avoid_phase_1.0':
        # -------------------------------------------------
    
            # will be maintained or overriden in loop
            RoBugState = 'avoid_phase_1.1'
            
            for i in range(n):
                await turn(dir,1)
                dist_tmp = await get_distance()
                if dist_tmp > min_dist + hysteresis:
                    await start_to_walk_fwd()
                    RoBugState = 'walk'
                    break                
                else:
                    dist_scan.append(dist_tmp)
                    

                
        # -------------------------------------------------
        elif RoBugState == 'avoid_phase_1.1':
        # -------------------------------------------------
        
            dir *= -1
            await turn(dir,n)
            RoBugState = 'avoid_phase_1.2'
            
        # -------------------------------------------------
        elif RoBugState == 'avoid_phase_1.2':
        # -------------------------------------------------
        
            RoBugState = 'avoid_phase_1.3'
            
            for i in range(n):
                await turn(dir,1)
                dist_tmp = await get_distance()
                if dist_tmp > min_dist + hysteresis:
                    await start_to_walk_fwd()
                    RoBugState = 'walk'
                    break
                else:
                    dist_scan.append(dist_tmp)

        # -------------------------------------------------
        elif RoBugState == 'avoid_phase_1.3':
        # -------------------------------------------------
        
            dist_scan_0 = dist_scan[:(n-1)]
            dist_scan_1 = dist_scan[n:]
            
            max_value_0 = max(dist_scan_0)
            max_value_1 = max(dist_scan_1)
            max_value = max(max_value_0, max_value_1)
            
            dir *=-1
            if max_value_0 > max_value_1:
                await turn(dir,n)
                await turn(dir, dist_scan.index(max_value_0)+1)
            else:
                await turn(dir, (n-1)-dist_scan_1.index(max_value_1))
                
            if max_value < too_close:
                RoBugState = 'avoid_phase_3'
            elif max_value >= clear_dist:
                await start_to_walk_fwd()
                RoBugState = 'walk'
            else:
                RoBugState = 'avoid_phase_2'        
        
        # -------------------------------------------------
        elif RoBugState == 'avoid_phase_2':
        # -------------------------------------------------
        
            await turn(dir,1)
            dist = await get_distance()
            if dist < too_close:
                RoBugState = 'avoid_phase_3'            
            elif dist < min_dist+hysteresis:
                RoBugState = 'avoid_phase_2'               
            else:
                await start_to_walk_fwd()
                RoBugState = 'walk'
                        
        # -------------------------------------------------                   
        elif RoBugState == 'avoid_phase_3':
        # -------------------------------------------------
         
            await start_to_walk_bwd()
            await asyncio.sleep(1)
            await stop_bwd()
            await asyncio.sleep(0.75)
            dir = random.choice(dirChoice)
            await turn(dir,3)
            await start_to_walk_fwd()
            RoBugState = 'walk'
                
        # -------------------------------------------------
        else:
        # -------------------------------------------------
            
            print('unknown state')
            break
        
        if RoBugState != CurrentState:
            print('new state:', RoBugState)
            CurrentState = RoBugState
        await asyncio.sleep(0.1)
        # -------------------------------------------------

async def main():

    # start any sensor related tasks that need to run concurrently
    task1 = asyncio.create_task(pulse_leds())
    
    # start the motion controller
    task2 = asyncio.create_task(m.run())

    # start the supervisor (example above). This could be a task that
    # handles bluetooth remote control or any task to control RoBug
    task3 = asyncio.create_task(explorer())
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
    asyncio.run(main())
    
    # clean up and shut down
    r.set_brightness_red(0)
    r.set_brightness_grn(100)
    
    # r.deinit() 

