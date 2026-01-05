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
# from robug_com import rbcom

def duty(pct):
    return int((pct * pow(2,16))/100)

async def loop_timer(loop_ms):
    await asyncio.sleep_ms(loop_ms)

async def pulse_leds():
    # green led starts at duty=100%
    i = 90
    redPct = 0
    grnPct = 0
    while True:
        redPwm.duty_u16(duty(redPct))
        grnPwm.duty_u16(duty(grnPct))
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
# that's why it's not running in a separate task but is evaluated with in the
# explorer task directly, accepting that I2C I/O blocks the async. task loop
# for a few milliseconds. However, an async ranging function is included in
# this example code for reference. It has the same mentioned limitations
async def explorer():
    
    # initialize random number generator
    random.seed()
    
    # initialize random number generator    
    min_dist = 200
    hysteresis = 50
    
    # init Robug
    await send_cmd('RESUME_FWD')
    await asyncio.sleep(0.25)
    await send_cmd('STOP_POSE_FWD')
    await asyncio.sleep(1)    

    while True:
        
        if vl53.range > min_dist:
            await send_cmd('START_POSE_FWD')            
            await send_cmd('RESUME_FWD')       
            while vl53.range > min_dist:
                await asyncio.sleep(0.25)
                
        await send_cmd('STOP_POSE_FWD')    
        await asyncio.sleep(0.25)
                
        dir = random.randint(0,1)
        while vl53.range < min_dist + hysteresis:
            if dir == 0:
                await send_cmd('TURN_LFT')
            else:
                await send_cmd('TURN_RGT')
            
        await asyncio.sleep(0.25)            
           
async def main():
    
    # start the motion controller
    task1 = asyncio.create_task(m.run())

    # start any sensor related tasks that need to run concurrently
    task2 = asyncio.create_task(pulse_leds())
    
    # start the supervisor (example above). This could be a task that
    # handles bluetooth remote control, a task to control RoBug depending
    # on sensor readings etc.
    # The supervisor send commands through the MsgQueue to the oBug motion
    # control message Server (MCMS). The MCMS will reply with 'DONE' when a
    # command completed. It will only look for new commands after completion
    # of the previous command
    task4 = asyncio.create_task(explorer())
    
    await task4
    task1.cancel()
    task2.cancel()
    task3.cancel()

if __name__ == "__main__":
    
    # led control
    freq = 1000
    cycle = 1/1000
    cycle_us = cycle *1e6
    dc = 50
    green = Pin(4)
    red = Pin(5)
    redPwm = PWM(red)
    grnPwm = PWM(green)
    redPwm.freq(freq)
    grnPwm.freq(freq)
    
    # leds after power-on / restart
    grnPwm.duty_u16(pow(2,16)-1)
    redPwm.duty_u16(0)
    sleep(1)     

    # set up time slice 
    fLoop = c._GAIT_LOOP_TIME
    
    # set up distance sensor
    sda = machine.Pin(20) # SDA pin
    scl = machine.Pin(21) # SCL pin
    i2c = machine.I2C(0, sda=sda, scl=scl, freq=400000)
    vl53 = vl53l0x(i2c)
    vl53.stop_continuous()
    dist = 0.0

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
    grnPwm.duty_u16(pow(2,16)-1)
    redPwm.duty_u16(0)
    
    # r.deinit() 
