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

async def read_touch_sensors():
    while True:
        print(r.get_touch_top())
        print(r.get_touch_bot())
        await asyncio.sleep(1)

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
    
# example supervisor
# simple linear sequence of commands
async def supervisor():
    
    await send_cmd('RESUME_FWD')       
    await asyncio.sleep(0.25)    
    await send_cmd('STOP_POSE_FWD')    
    await asyncio.sleep(1)
    
    for i in range(7):
        await send_cmd('TURN_LFT')    
    await asyncio.sleep(1)    
    
    await send_cmd('START_POSE_FWD')        
    await send_cmd('RESUME_FWD')    
    await asyncio.sleep(4)    

    await send_cmd('STOP_POSE_FWD')    
    await asyncio.sleep(1)
    
    for i in range(7):    
        await send_cmd('TURN_RGT')
    await asyncio.sleep(1)          
        
    await send_cmd('START_POSE_FWD')
    await send_cmd('RESUME_FWD')
    await asyncio.sleep(4)    

    await send_cmd('STOP_POSE_FWD')       
    await asyncio.sleep(1)    
    print(r.lLeg[0].foot_pos, r.lLeg[1].foot_pos)
 
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

async def explorer():
    # initialize random number generator
    random.seed()
    
    # initialize random number generator    
    min_dist = 225
    hysteresis = 100
    keep_direction = False
    avoid = False
    dist = 1000.0
    
    # init Robug
    await send_cmd('RESUME_FWD')
    await asyncio.sleep(0.25)
    await send_cmd('STOP_POSE_FWD')
    await asyncio.sleep(1)    

    while True:
        
        await send_cmd('START_POSE_FWD')            
        await send_cmd('RESUME_FWD')
        
        while dist > min_dist:
            await asyncio.sleep(0.5)
            dist = await get_distance()
                
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
    task3 = asyncio.create_task(read_touch_sensors())
    
    # start the supervisor (example above). This could be a task that
    # handles bluetooth remote control, a task to control RoBug depending
    # on sensor readings etc.
    # The supervisor/exlporer task send commands through the MsgQueue to the
    # RoBug motion control message Server (MCMS). The MCMS will reply with
    # 'DONE' when a command completed. It will only look for new commands
    # after completion of the previous command
    task4 = asyncio.create_task(explorer())
    
    await task4
    task1.cancel()
    task2.cancel()
    task3.cancel()

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
