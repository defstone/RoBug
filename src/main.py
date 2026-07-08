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
import json
from machine import Pin, PWM, I2C, ADC

from robug_constants import constants as c
from robug_robot import robug
from robug_ctrl import rbctrl
from robug_mocon import rbmocon
from robug_ble import rbble
from robug_app_calibrator import calibration

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
        redPct = 15 + math.sin(math.radians(i)+math.pi) * 15        
        await asyncio.sleep_ms(7)
        
async def get_distance():
    # median of 6 measurements
    lTmp = []
    for i in range(6):
        lTmp.append(r.get_distance())
        await asyncio.sleep_ms(50)
    lTmp.sort()
    return int((lTmp[2]+lTmp[3])/2)

async def get_soc():
    # store voltage of multiples of 10mV
    voltage = int((adc.read_u16() * cf) / 0.01)
    if voltage < 0: voltage = 0
    await asyncio.sleep_ms(50)
    return voltage

async def serve_sensor_data():
    global distance
    global soc
    print("sensor server up and running")
    while True:
        distance = await get_distance()
        soc = await get_soc()
        ble.dist = distance
        ble.soc = soc
        await asyncio.sleep_ms(250)
             
# --------------------------------------------------------
# application: bluetooth low energy remote control
# --------------------------------------------------------
async def fpv_rc(b):
    global RoBugState, distance
    RoBugState = 'init'
    
    while True:
        
        # get controller button states once(!) per loop
        btn_fwd = b.btn_fwd
        btn_bwd = b.btn_bwd
        btn_lft = b.btn_lft
        btn_rgt = b.btn_rgt
        btn_kck = b.btn_fn0
        btns_idle = not(btn_fwd or btn_bwd or btn_lft or btn_rgt or btn_kck)
        
        # -------------------------------------------------
        if   RoBugState == 'init':
        # -------------------------------------------------                
            await rc.init_pose()
            RoBugState = 'waitForActivation'            

        # -------------------------------------------------
        elif RoBugState == 'waitForActivation':
        # -------------------------------------------------        
            # if r.touch_bot():
            if True:
                RoBugState = 'idle'
                
        # -------------------------------------------------
        elif RoBugState == 'idle':
        # -------------------------------------------------        
            if btns_idle:
                RoBugState = 'idle'
            elif btn_fwd:
                await rc.start_to_walk_fwd()
                RoBugState = 'forward'
            elif btn_bwd:
                await rc.start_to_walk_bwd()
                RoBugState = 'backward'
            elif btn_lft:
                RoBugState = 'turn_left'
            elif btn_rgt:
                RoBugState = 'turn_right'
            elif btn_kck:
                await rc.kick()
            else:
                print('unknown command during idle: ', cmd)
                RoBugState = 'idle'                
                                 
        # -------------------------------------------------                   
        elif RoBugState == 'forward':
        # -------------------------------------------------         
            if btn_fwd:
                RoBugState = 'forward'
                if btn_lft:
                    await rc.walk_lft()
                elif btn_rgt:
                    await rc.walk_rgt()
                else:
                    await rc.walk_strgt()                                
            else:
                await rc.walk_strgt()
                await rc.stop_fwd()
                RoBugState = 'idle'
                
        # -------------------------------------------------                   
        elif RoBugState == 'backward':
        # -------------------------------------------------         
            if btn_bwd:
                RoBugState = 'backward'
                if btn_lft:
                    await rc.walk_lft()
                elif btn_rgt:
                    await rc.walk_rgt()
                else:
                    await rc.walk_strgt()                
            else:
                await rc.walk_strgt()                
                await rc.stop_bwd()
                RoBugState = 'idle'                

        # -------------------------------------------------                   
        elif RoBugState == 'turn_left':
        # -------------------------------------------------         
            if btn_lft:
                await rc.turn(-1)
            else:
                RoBugState = 'idle'
                
        # -------------------------------------------------                   
        elif RoBugState == 'turn_right':
        # -------------------------------------------------         
            if btn_rgt:
                await rc.turn( 1)
            else:
                RoBugState = 'idle'                  

        # -------------------------------------------------                   
        else:
        # -------------------------------------------------                         
            print('unknown state: ', RoBugState)
            RoBugState = 'idle'
            
        await asyncio.sleep(0.05)
        
async def main_cal():
    # start tasks
    task0 = asyncio.create_task(ble.msg_handler())
    task1 = asyncio.create_task(calibration(r, ble))
    # wait for termination        
    await task1
    task0.cancel()    

async def main_rc():
    #start ble task
    task_ble  = asyncio.create_task(ble.msg_handler())
    # start sensor tasks
    task_dist = asyncio.create_task(serve_sensor_data())
    # start any tasks that need to run concurrently
    task_led = asyncio.create_task(pulse_leds())
    # start the motion controller
    task_mc = asyncio.create_task(m.run())
    # start application
    task_rc = asyncio.create_task(fpv_rc(ble))
    # wait for termination
    await task_rc
    task_ble.cancel()    
    task_dist.cancel()
    task_led.cancel()
    task_mc.cancel()          
        
# --------------------------------------------------------
# main
# --------------------------------------------------------                    
if __name__ == "__main__":
    

        
    # ----------------------------
    # initialization
    # ----------------------------   
    
    # declare global variables 
    RoBugState = ''
    distance = 9999
    soc = 9999
    
    # setup BLE
    ble = rbble()
    # generate global robug object
    r = robug()
    sleep(1)
    
    # select application:
    # touch_top = 1 --> calibration
    # touch_top = 0 --> rc
    if r.touch_top():
        ble.set_mode('raw')
        for i in range(4):
            for j in range(2):
                r.lLeg[i].joints.set_angle_sid(j, 0)
                sleep(0.25)
        # start tasks
        asyncio.run(main_cal())        
    else:
        # init RoBug and set start position
        r.reset_loop_counter()
        r.calculate_foot_positions()
        r.solve_ik()
        r.set_joints()
        sleep(1)
        # set up ADC
        adc = ADC(c._PIN_ADC)
        # voltage divider (47k/100k) backwards
        # and scaling to 3.3V
        cf = (3.3 / 65536) * (147/47)
        # set up motion controller
        MsgQueue  = deque([], 4)
        RplyQueue = deque([], 4)
        rc = rbctrl(MsgQueue, RplyQueue)
        m = rbmocon(r, MsgQueue, RplyQueue)
        # start tasks
        asyncio.run(main_rc())
        
    # clean up and shut down
    r.set_brightness_red(50)
    r.set_brightness_grn(50)
    # r.deinit() 
 