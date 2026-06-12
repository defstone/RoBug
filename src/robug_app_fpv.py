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
from machine import Pin, PWM, I2C, ADC

from robug_constants import constants as c
from robug_robot import robug
from robug_ctrl import rbctrl
from robug_mocon import rbmocon
from robug_ble import rbble

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

async def serve_sensor_data():
    global distance
    print("sensor server up and running")
    while True:
        distance = await get_distance()
        ble.value = distance
        # print(distance)
        await asyncio.sleep_ms(250)
        
async def get_battery_voltage():
    while True:
        voltage = adc.read_u16() * cf
        print("Voltage: {:.2f} V".format(voltage))
        await asyncio.sleep_ms(500) 
           
# --------------------------------------------------------
# application: first-person-view remote control
# --------------------------------------------------------
async def fpv_rc(b):
    global RoBugState, distance

    RoBugState = 'init'
    CurrentState = 'init'
    
    while True:
        cmd = b.get_current_cmd()
        
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
            if cmd == 'STOP':
                RoBugState = 'idle'
            elif cmd == 'FWD':
                await rc.start_to_walk_fwd()
                RoBugState = 'forward'
            elif cmd == 'BWD':
                await rc.start_to_walk_bwd()
                RoBugState = 'backward'
            elif cmd == 'LEFT':
                RoBugState = 'turn_left'
            elif cmd == 'RIGHT':
                RoBugState = 'turn_right'
            elif cmd == 'SIT_DOWN':
                await rc.sit_down()
                RoBugState = 'sitting'
            else:
                print('unknown command during idle: ', cmd)
                RoBugState = 'idle'                
                                 
        # -------------------------------------------------                   
        elif RoBugState == 'forward':
        # -------------------------------------------------         
            if cmd == 'STOP':
                await rc.stop_fwd()
                await asyncio.sleep(0.1)
                RoBugState = 'idle'
            else:
                RoBugState = 'forward'
                
        # -------------------------------------------------                   
        elif RoBugState == 'backward':
        # -------------------------------------------------         
            if cmd == 'STOP':
                await rc.stop_bwd()
                await asyncio.sleep(0.1)
                RoBugState = 'idle'
            else:
                RoBugState = 'backward'
                
        # -------------------------------------------------                   
        elif RoBugState == 'turn_left':
        # -------------------------------------------------         
            if cmd != 'STOP':
                await rc.turn(-1, 1)
                await asyncio.sleep(0.05)
            else:
                RoBugState = 'idle'
                
        # -------------------------------------------------                   
        elif RoBugState == 'turn_right':
        # -------------------------------------------------         
            if cmd != 'STOP':
                await rc.turn( 1, 1)
                await asyncio.sleep(0.05)
            else:
                RoBugState = 'idle'                  

        # -------------------------------------------------                   
        elif RoBugState == 'sitting':
        # -------------------------------------------------         
            if cmd == 'STAND_UP':
                await rc.stand_up()
                RoBugState = 'idle'                
            else:
                RoBugState = 'sitting'
                
        # -------------------------------------------------                   
        else:
        # -------------------------------------------------                         
            print('unknown state: ', RoBugState)
            await rc.stop_fwd()
            await asyncio.sleep(0.1)
            RoBugState = 'idle'
            
        await asyncio.sleep_ms(100)

# --------------------------------------------------------
# main
# --------------------------------------------------------                    
if __name__ == "__main__":
    
    async def main():
        
        #start ble task
        task_ble  = asyncio.create_task(ble.msg_handler())
        
        # start sensor tasks
        task_dist = asyncio.create_task(serve_sensor_data())
        task_adc  = asyncio.create_task(get_battery_voltage())
        
        # start any tasks that need to run concurrently
        # no LEDs yet
        task_led = asyncio.create_task(pulse_leds())
        
        # start the motion controller
        task_mc = asyncio.create_task(m.run())

        # start the supervisor (example above). This could be a task that
        # handles bluetooth remote control or any task to control RoBug
        task_rc = asyncio.create_task(fpv_rc(ble))
        
        await task_rc
        task_ble.cancel()    
        task_dist.cancel()
        task_adc.cancel()
        task_led.cancel()
        task_mc.cancel()        
        
    # ----------------------------
    # initialization
    # ----------------------------   
    
    # init distance value
    distance = 9999
    
    # setup BLE
    ble = rbble()

    # init RoBug, set start position
    r = robug()
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
    
    # declare global variables 
    RoBugState = ''
    dist = 0
    
    # start tasks
    asyncio.run(main())
    
    # clean up and shut down
    # r.set_brightness_red(0)
    # r.set_brightness_grn(100)
    
    # r.deinit() 
