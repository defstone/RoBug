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

import math
import asyncio
import json
from time import sleep_ms
from machine import Pin, PWM, I2C

from tof_sensor import vl53l0x
from robug_utils import v3
from robug_constants import constants as c
from robug_leg import rbleg

############################
## class robot
############################

class robug:

    def __init__(self):
        
        self.create_robug()
        self.dirX  = 1
        self.dirZ  = 1
        self.lLegTicks =[[], [], [], []]
        
        # touch sensor setup
        self.touch_top = Pin(c._PIN_TOUCH_TOP, Pin.IN, Pin.PULL_UP)
        self.touch_bot = Pin(c._PIN_TOUCH_BOT, Pin.IN, Pin.PULL_UP)
        
        # distance sensor setup
        self.vl53 = vl53l0x(I2C(c._I2C_BUS, sda=Pin(c._PIN_I2C_SDA), scl=Pin(c._PIN_I2C_SCL), freq=c._I2C_RATE))
        self.vl53.stop_continuous()

        # led setup
        self.freq = c._SERVO_PWM_FREQ
        self.dc = 50
        self.led_grn = Pin(c._PIN_LED_GRN)
        self.led_red = Pin(c._PIN_LED_RED)
        self.pwm_red = PWM(self.led_red)
        self.pwm_grn = PWM(self.led_grn)
        self.pwm_red.freq(self.freq)
        self.pwm_grn.freq(self.freq)
        
        # led init
        self.pwm_grn.duty_u16(pow(2,16)-1)
        self.pwm_red.duty_u16(0)
        Pin(27, Pin.IN)
        Pin(28, Pin.IN)       

    #--------------------------------
    #-- sensor functions ------------
    #--------------------------------
        
    def touch_top(self):
        return self.touch_top.value()
    
    def touch_bot(self):
        return self.touch_bot.value()   
        
    def get_distance(self):
        return self.vl53.range
        
    def duty(self, pct):
        return int((pct * pow(2,16))/100)

    def set_brightness_red(self, pct):
        self.pwm_red.duty_u16(self.duty(pct))
        
    def set_brightness_grn(self, pct):
        self.pwm_grn.duty_u16(self.duty(pct))
        
    #--------------------------------
    #-- robot core functions  -------
    #--------------------------------
        
    def create_robug(self):
        with open('robug_calibration.json', 'rt') as f:
            calib_data = json.load(f)
            lOffs = calib_data['robug_calibration_data']['servo_offs']
            lGain = calib_data['robug_calibration_data']['servo_gain']         
        self.lLeg = [rbleg(0, lOffs, lGain),
                     rbleg(1, lOffs, lGain),
                     rbleg(2, lOffs, lGain),
                     rbleg(3, lOffs, lGain)]
        
    def inc_loop_counters(self):
        for i in range(4):
            self.lLeg[i].inc_loop_counter()
            
    def calculate_foot_positions(self, bAbs=True):
        for i in range(4):
            self.lLeg[i].calculate_foot_position(bAbs)
            
    def solve_ik(self):
        for i in range(4):
            self.lLeg[i].solve()        

    def set_joints(self):
        for i in range(4):        
            self.lLeg[i].set_joints()
            
    async def set_positions_relative(self, lRelPos, iSteps):
        ldX = []
        ldZ = []
        for i in range(4):
            # iSteps = 1 -> immediate change
            # iSteps > 1 -> smooth transition
            ldX.append(lRelPos[i].x/iSteps)
            ldZ.append(lRelPos[i].z/iSteps)
        for i in range(iSteps):
            for i in range(4):            
                self.lLeg[i].foot_pos.x += ldX[i] * c._LEG_DIR[i]
                self.lLeg[i].foot_pos.z += ldZ[i]
                self.solve_ik()
                self.set_joints()
            await asyncio.sleep_ms(c._GAIT_LOOP_TIME)
            
    def push_enable(self):
        for i in range(4):
            self.lLeg[i].gait.push_enable()

    def push_disable(self):
        for i in range(4):
            self.lLeg[i].gait.push_disable()            
            
    def deinit_joints(self):
        for i in range(4):
            # store servo positions before deinit
            self.lLegTicks[i] = self.lLeg[i].joints.lServoPos
            self.lLeg[i].joints.deinit()            
            
    #--------------------------------
    #-- gait loop manipulation ------
    #--------------------------------            

    def set_loop_counter(self, Id, i):
        self.lLeg[Id].gait.set_loop_counter(i)
        
    def reset_loop_counter(self):
        # init diagonal legs with trott phase shift
        for i in range(4):
            self.set_loop_counter(i, c._GAIT_LEG_PHASE_OFFSET[i])
            
    def set_loop_counter_restart(self):
        # calculate default loop states for correct restart with leg0 after stop
        self.set_loop_counter(0, c._GAIT_SUPPORT_TICKS + c._GAIT_SWING_TICKS/2)
        self.set_loop_counter(1, c._GAIT_SUPPORT_TICKS/2)
        self.set_loop_counter(2, c._GAIT_SUPPORT_TICKS/2)
        self.set_loop_counter(3, c._GAIT_SUPPORT_TICKS + c._GAIT_SWING_TICKS/2)
            
    #--------------------------------
    #-- robot status ----------------
    #--------------------------------
            
    def is_stable(self):
        bTmp = True
        for i in range(4):
            if self.lLeg[i].gait.is_loop_frame('swing_phase'):
                bTmp = False
        return bTmp

    def is_support_phase(self, Id):
        return self.lLeg[Id].gait.is_loop_frame('support_phase')
    
    def is_support_phase_any(self):
        Tmp0 = self.lLeg[0].gait.is_loop_frame('support_phase')
        Tmp1 = self.lLeg[1].gait.is_loop_frame('support_phase')
        Tmp2 = self.lLeg[2].gait.is_loop_frame('support_phase')
        Tmp3 = self.lLeg[3].gait.is_loop_frame('support_phase')
        return (Tmp0 or Tmp1 or Tmp2 or Tmp3)    

    def is_swing_phase(self, Id):
        return self.lLeg[Id].gait.is_loop_frame('swing_phase')
        
    def is_swing_phase_any(self):
        Tmp0 = self.lLeg[0].gait.is_loop_frame('swing_phase')
        Tmp1 = self.lLeg[1].gait.is_loop_frame('swing_phase')
        Tmp2 = self.lLeg[2].gait.is_loop_frame('swing_phase')
        Tmp3 = self.lLeg[3].gait.is_loop_frame('swing_phase')
        return (Tmp0 or Tmp1 or Tmp2 or Tmp3)

    def is_support_start(self, Id):
        return self.lLeg[Id].gait.is_loop_frame('support_start')

    def is_support_start_any(self):
        Tmp0 = self.lLeg[0].gait.is_loop_frame('support_start')
        Tmp1 = self.lLeg[1].gait.is_loop_frame('support_start')
        Tmp2 = self.lLeg[2].gait.is_loop_frame('support_start')
        Tmp3 = self.lLeg[3].gait.is_loop_frame('support_start')
        return (Tmp0 or Tmp1 or Tmp2 or Tmp3)

    def is_support_end(self, Id):
        return self.lLeg[Id].gait.is_loop_frame('support_end')

    def is_support_end_any(self):
        Tmp0 = self.lLeg[0].gait.is_loop_frame('support_end')
        Tmp1 = self.lLeg[1].gait.is_loop_frame('support_end')
        Tmp2 = self.lLeg[2].gait.is_loop_frame('support_end')
        Tmp3 = self.lLeg[3].gait.is_loop_frame('support_end')
        return (Tmp0 or Tmp1 or Tmp2 or Tmp3)

    def is_support_mid(self, Id):
        return self.lLeg[Id].gait.is_loop_frame('support_mid')

    def is_support_mid_any(self):
        Tmp0 = self.lLeg[0].gait.is_loop_frame('support_mid')
        Tmp1 = self.lLeg[1].gait.is_loop_frame('support_mid')
        Tmp2 = self.lLeg[2].gait.is_loop_frame('support_mid')
        Tmp3 = self.lLeg[3].gait.is_loop_frame('support_mid')
        return (Tmp0 or Tmp1 or Tmp2 or Tmp3)            

    #--------------------------------
    #-- getters and setters  --------
    #--------------------------------
    
    def get_support_phase_length(self, Id):
        return self.lLeg[Id].gait.ifwd_base

    def get_swing_phase_length(self,Id):
        return self.lLeg[Id].gait.irtn_base    

    def get_loop_length(self):
        return c._GAIT_SUPPORT_TICKS +c._GAIT_SWING_TICKS

    def set_direction(self, dir, strAxis):
        if   strAxis == 'x': self.dirX = dir
        elif strAxis == 'z': self.dirZ = dir
        for i in range(4):
            # dir * c._LEG_DIR[i] accouts for leg mounting orientation
            self.lLeg[i].gait.set_direction(dir * c._LEG_DIR[i], strAxis)
            
    def set_body_lean(self, dir, asym):
        lXoffset = [c._FOOT_XOFFSET - asym * dir,
                    c._FOOT_XOFFSET + asym * dir,
                    c._FOOT_XOFFSET - asym * dir,
                    c._FOOT_XOFFSET + asym * dir]
        lOverlay = [v3(lXoffset[0], 0.0, 0.0),
                    v3(lXoffset[1], 0.0, 0.0),
                    v3(lXoffset[2], 0.0, 0.0),
                    v3(lXoffset[3], 0.0, 0.0)]
        for i in range(4):
            self.lLeg[i].overlay_pose.set(lOverlay[i])
      
    def get_direction(self, strAxis):
        if   strAxis == 'x': return self.dirX
        elif strAxis == 'z': return self.dirZ 

    def get_xyz(self, Id):
        return self.lLeg[Id].gait.get_xyz()

    def set_xyz(self, Id, xyz):
        self.lLeg[Id].gait.set_xyz(xyz)
        
    def set_gait_gains(self, dir):
        if self.dirX == 1:
            if   dir == 'left':     gain = c._GAIT_FWD_GAIN_LFT
            elif dir == 'right':    gain = c._GAIT_FWD_GAIN_RGT
            elif dir == 'straight': gain = c._GAIT_FWD_GAIN
            else: gain = [0, 0, 0, 0]
        elif self.dirX == -1:
            if   dir == 'left':     gain = c._GAIT_BWD_GAIN_LFT
            elif dir == 'right':    gain = c._GAIT_BWD_GAIN_RGT
            elif dir == 'straight': gain = c._GAIT_BWD_GAIN
            else: gain = [0, 0, 0, 0]            
            
        for i in range(4):
            self.lLeg[i].gait.set_gain(gain[i])
            
    def get_gait_gains(self):
        return([self.lLeg[i].gait.get_gain() for i in range(4)])