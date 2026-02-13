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

from machine import Pin, PWM
from time import sleep
from robug_constants_stat import constants as c
        
class rbjoints:
    
    def __init__(self, iLegID):
        self.ID   = iLegID
        self.name = c._GAIT_NAME[iLegID]
        # this remaps the femur joint to ID0 and the tibia joint to ID1
        # ... I guess ...
        ID0 = (2*iLegID)
        ID1 = (2*iLegID)+1
        self.lServoMap  = [c._SERVO_MAP[ID0],  c._SERVO_MAP[ID1]]
        self.lServoSgn  = [c._SERVO_SGN[ID0],  c._SERVO_SGN[ID1]]
        self.lServoCal  = [c._SERVO_CAL[ID0],  c._SERVO_CAL[ID1]]
        self.lServoGain = [c._SERVO_GAIN[ID0], c._SERVO_GAIN[ID1]]        
        self.lServoPos  = [ 0, 0]
        self.iPwmFreq = c._SERVO_PWM_FREQ
        self.iPwmCycle = 1/c._SERVO_PWM_FREQ
        self.iPwmCycle_us = self.iPwmCycle *1e6
        self.fDutyLsb = self.iPwmCycle_us / pow(2,16)
        self.init_joints()
    
    def init_joints(self):
        TmpPin0 = Pin(self.lServoMap[0])
        TmpPin1 = Pin(self.lServoMap[1])            
        TmpPwm0 = PWM(TmpPin0)
        TmpPwm1 = PWM(TmpPin1)            
        TmpPwm0.duty_u16(0)
        TmpPwm1.duty_u16(0)            
        TmpPwm0.freq(self.iPwmFreq)
        TmpPwm1.freq(self.iPwmFreq)            
        self.lServo = [TmpPwm0, TmpPwm1]

    def deinit(self):
        for servo in self.lServo:
            servo.deinit()         
                   
    def safe_limits(self, iTicks):
        if   iTicks > c._SERVO_MAX: return c._SERVO_MAX
        elif iTicks < c._SERVO_MIN: return c._SERVO_MIN
        else: return iTicks
            
    def set_angle_sid(self, sid, iAngleInTicks):        
        iTicks = (c._SERVO_NEUTRAL + self.lServoCal[sid]) + (iAngleInTicks * self.lServoSgn[sid] * self.lServoGain[sid])
        iSafeTicks = self.safe_limits(iTicks)
        iDutyCyle = int(iSafeTicks/self.fDutyLsb)
        self.lServoPos[sid] = iSafeTicks
        self.lServo[sid].duty_u16(iDutyCyle)

    def set_angles(self, iDeltaTicks, iGammaTicks):
        self.set_angle_sid(0,  iDeltaTicks)
        self.set_angle_sid(1,  iGammaTicks)         

