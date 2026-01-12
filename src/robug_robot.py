import math
import asyncio
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
        self.touch_top = Pin(0, Pin.IN)
        self.touch_bot = Pin(1, Pin.IN)
        
        # distance sensor setup
        self.vl53 = vl53l0x(I2C(0, sda=Pin(20), scl=Pin(21), freq=400000))
        self.vl53.stop_continuous()
        
        # led setup
        self.freq = c._LED_PWM_FREQ
        self.dc = 50
        self.led_grn = Pin(4)
        self.led_red = Pin(5)
        self.pwm_red = PWM(self.led_red)
        self.pwm_grn = PWM(self.led_grn)
        self.pwm_red.freq(self.freq)
        self.pwm_grn.freq(self.freq)
        
        # led init
        self.pwm_grn.duty_u16(pow(2,16)-1)
        self.pwm_red.duty_u16(0)
        
    def get_touch_top(self):
        return self.touch_top.value()
    
    def get_touch_bot(self):
        return self.touch_bot.value()     
        
    def get_distance(self):
        return self.vl53.range        
        
    def duty(self, pct):
        return int((pct * pow(2,16))/100)

    def set_brightness_red(self, pct):
        self.pwm_red.duty_u16(self.duty(pct))
        
    def set_brightness_grn(self, pct):
        self.pwm_grn.duty_u16(self.duty(pct))

    def create_robug(self):
        self.lLeg = [rbleg(0), rbleg(1), rbleg(2), rbleg(3)]

    def set_loop_counter(self, Id, i):
        self.lLeg[Id].gait.set_loop_counter(i)
        
    def set_loop_counter_resume(self):
        # init diagonal legs with trott phase shift
        for i in range(4):
            self.set_loop_counter(i, c._GAIT_LEG_PHASE_OFFSET[i])
            
    def set_loop_counter_restart(self):
        # calculate default loop states for correct restart with leg0 after stop
        self.set_loop_counter(0, c._GAIT_SUPPORT_TICKS + c._GAIT_SWING_TICKS/2)
        self.set_loop_counter(1, c._GAIT_SUPPORT_TICKS/2)
        self.set_loop_counter(2, c._GAIT_SUPPORT_TICKS/2)
        self.set_loop_counter(3, c._GAIT_SUPPORT_TICKS + c._GAIT_SWING_TICKS/2)

    def instant_update(self):
        self.calculate_joint_angles()
        self.set_joints()       

    def inc_loop_counters(self):
        for i in range(4):
            self.lLeg[i].inc_loop_counter()

    def calculate_joint_angles(self, bAbs=True):
        for i in range(4):
            self.lLeg[i].calculate_joint_angles(bAbs)

    def set_joints(self):
        for i in range(4):        
            self.lLeg[i].set_joints()
            
    def set_positions(self, lPos):
        # 1.) solve ik for all legs, results stored in leg inst. 
        for i in range(4):
            if isinstance(lPos, list):
                # solve ik with lPos item
                self.lLeg[i].solve_ik(lPos[i])
            else:
                # solve ik with leg.foot_pos
                self.lLeg[i].solve()
        # 2.) set joints all at one to minimize delay btwn legs
        for i in range(4):
            self.lLeg[i].set_joints()
            
    async def set_positions_relative(self, lRelPos, iSteps):
        ldX = []
        ldZ = []
        for i in range(4):
            ldX.append(lRelPos[i].x/iSteps)
            ldZ.append(lRelPos[i].z/iSteps)
        for i in range(iSteps):
            for i in range(4):            
                self.lLeg[i].foot_pos.x += ldX[i] * c._LEG_DIR[i]
                self.lLeg[i].foot_pos.z += ldZ[i]
                self.set_positions(None)
            await asyncio.sleep_ms(c._GAIT_LOOP_TIME)             
            
    def deinit_joints(self):
        for i in range(4):
            # store servo positions before deinit
            self.lLegTicks[i] = self.lLeg[i].joints.lServoPos
            self.lLeg[i].joints.deinit()

    def get_loop_length(self):
        return c._GAIT_SUPPORT_TICKS +c._GAIT_SWING_TICKS

    def get_support_phase_length(self, Id):
        return self.lLeg[Id].gait.ifwd_base

    def get_swing_phase_length(self,Id):
        return self.lLeg[Id].gait.irtn_base

    def set_direction(self, dir, strAxis):
        if   strAxis == 'x': self.dirX = dir
        elif strAxis == 'z': self.dirZ = dir
        # apply direction to legs accouting for leg mounting orientation
        for i in range(4):
            self.lLeg[i].gait.set_direction(dir * c._LEG_DIR[i], strAxis)

    def is_stable(self):
        bTmp = True
        for i in range(4):
            if self.lLeg[i].gait.is_loop_frame('swing_phase'):
                bTmp = False
        return bTmp

    def is_support_phase(self, Id):
        return self.lLeg[Id].gait.is_loop_frame('support_phase')

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

    def get_xyz(self, Id):
        return self.lLeg[Id].gait.get_xyz()

    def set_xyz(self, Id, xyz):
        self.lLeg[Id].gait.set_xyz(xyz)

    def push_enable(self):
        for i in range(4):
            self.lLeg[i].gait.push_enable()

    def push_disable(self):
        for i in range(4):
            self.lLeg[i].gait.push_disable()
            
    def set_push_impulse(self, xyzStrength):
        for i in range(4):
            self.lLeg[Id].gait.xyzImpulse = xyzStrength


