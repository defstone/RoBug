from math import degrees, pi, sin
from time import sleep
import math
import asyncio
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
        
async def distance_sensor():
    global dist
    while True:
        dist = vl53.range
        print("Range: {0}mm".format(vl53.range))
        await asyncio.sleep_ms(500)        
        
async def wait_for_reply():   
    while True:
        # print('client waiting', len(RplyQueue))
        if len(RplyQueue) > 0:
            strRply = RplyQueue.pop()
            print(f'client received reply: {strRply}')
            return strRply
        await asyncio.sleep_ms(100)
    
async def msg_handler_client():
    global MsgQueue
    
    MsgQueue.append('RESUME_FWD')
    print('client sending RESUME_FWD')
    reply = await wait_for_reply()
    
    await asyncio.sleep(0.25)
    
    MsgQueue.append('STOP_POSE_FWD')
    print('client sending STOP_FWD')
    reply = await wait_for_reply()
    
    await asyncio.sleep(1)
    
    MsgQueue.append('TURN_LFT')
    print('client sending TURN_LFT')
    reply = await wait_for_reply()
    
    await asyncio.sleep(1)    
    
    MsgQueue.append('START_POSE_FWD')
    print('client sending START_POSE_FWD')
    reply = await wait_for_reply()
        
    MsgQueue.append('RESUME_FWD')
    print('client sending RESUME_FWD')
    reply = await wait_for_reply()
    
    await asyncio.sleep(3)    

    MsgQueue.append('STOP_POSE_FWD')
    print('client sending STOP_FWD')
    reply = await wait_for_reply()
    
    await asyncio.sleep(1)
    
    MsgQueue.append('TURN_LFT')
    print('client sending TURN_LFT')
    reply = await wait_for_reply()    
    
    await asyncio.sleep(1)    
    print(r.lLeg[0].foot_pos, r.lLeg[1].foot_pos)

async def main():
    task1 = asyncio.create_task(m.run())
    task2 = asyncio.create_task(msg_handler_client())
    task3 = asyncio.create_task(pulse_leds())
    # task4 = asyncio.create_task(distance_sensor())    
    await task2
    task1.cancel()
    task3.cancel()
    # task4.cancel()

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
    # vl53 = vl53l0x(i2c)
    # vl53.start_continuous(period_ms = 100)
    dist = 0.0

    # set up RoBug
    r = robug()
    r.set_loop_counter_resume()
    r.instant_update()
    sleep(2)
    
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
