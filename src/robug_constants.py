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

from math import pi
from robug_utils import v3

class constants:
    
    # ------- pre-calculated values -------
    
    _PI0P5 = pi/2
    
    # ------- physical paramters -------
    
    # servo pwm base frequency 
    _SERVO_PWM_FREQ = 250

    _SERVO_NEUTRAL = 1500
    # servo limits
    _SERVO_MAX     = 2600
    _SERVO_MIN     =  400
    
    # servo to gpio mapping
    # [0] front left femur
    # [1] front left tibia
    # ...
    # [7] rear right tibia
    _SERVO_MAP = [ 16,  17,  19,  18,  14,  15,  13,  12]
    
    # direction correction
    _SERVO_SGN = [  1,  -1,  -1,   1,  -1,   1,   1,  -1]
    
    # servo calibration for neutral stance in ticks (90° joint angles)
    _SERVO_CAL = [ 75,   20,  20, -55, -35,   0, -20, -50]
    
    # servo gain correction
    _SERVO_GAIN = [1.00, 1.00, 1.00, 1.03, 1.00, 0.97, 1.00, 0.97]
    
    # by testing: range +/-950 ticks = +/-90° 
    _SERVO_K = pi/1900
    
    # effective femur length axis to axis
    _L_FEMUR =  47.1
    
    # effective tibia length axis to bottom of foot (0 force on suspension)
    _L_TIBIA = 111.6
    
    # distance hip joint from body centre (for body IK)
    _DIST_PIVOT_TO_HIP = v3(  37.0, 0, -5.5)
    
    # pre-calculated valkues for ik solver
    _L_FEMUR_PWR2 = _L_FEMUR**2
    _L_TIBIA_PWR2 = _L_TIBIA**2
    _L_FEMUR_X2 = _L_FEMUR*2
    _L_TIBIA_X2 = _L_TIBIA*2    
    _L_FEMUR_X_TIBIA_X2 = _L_FEMUR * _L_TIBIA *2

    # ------- gait parameters -------
    
    _GAIT_NAME = ['FL', 'RL', 'FR', 'RR']
    
    # loop update rate in ms
    # golden: 11
    # speedy: 8
    _GAIT_LOOP_TIME = 8
    
    # height over ground
    # ref: shoulder joint
    # +z points up, -z points down
    # golden: -110
    _GAIT_HEIGHT = -100
    # low gait - quick
    # _GAIT_HEIGHT = -75
    
    # swing back time in ticks
    # golden: 14    
    # _GAIT_SWING_TICKS = 44
    _GAIT_SWING_TICKS = 14
    # low gait - quick
    # _GAIT_SWING_TICKS = 12    
    
    # linear movement phase in ticks
    # golden: 64    
    # _GAIT_SUPPORT_TICKS = 5*_GAIT_SWING_TICKS
    _GAIT_SUPPORT_TICKS = 54
    # low gait - quick
    # _GAIT_SUPPORT_TICKS = 48    
    
    # how much is loop counter incremented each loop
    _GAIT_LOOP_INC = 1
    
    # highest swing back point rel to _GAIT_HEIGHT
    # golden: -15
    # _GAIT_SWING_AMPL = -25
    _GAIT_SWING_AMPL = 15
    # low gait - quick
    # _GAIT_SWING_AMPL = 12    
    
    # distance from max. x to center of foot trajectory
    # golden: 35
    _GAIT_HALF_STRIDE = 35    
    # low gait - quick    
    # _GAIT_HALF_STRIDE = 25
    
    # amount of additional push of support legs to carry
    # twice the load during swing phase of other 2 legs
    # golden: (0, 0, -5)
    _GAIT_PUSH_STRENGTH = v3(0, 0, -5)
    
    # how long sustain push after touch down of swing legs
    # golden: 0.15
    _GAIT_PUSH_OVERLAP = int(_GAIT_SUPPORT_TICKS * 0.15)
    # low gait - quick    
    # _GAIT_PUSH_OVERLAP = int(_GAIT_SUPPORT_TICKS * 0.1)    
    
    # how much push is sustained as fraction of _GAIT_PUSH_STRENGTH
    # golden: 1
    _GAIT_OVERLAP_PUSH_FACTOR = 1
    
    # gain for direction change
    # test successfull -> right curve
    # _GAIT_FWD_GAIN = [3.0, 1.0, 1.0, 0.4]
    _GAIT_FWD_GAIN = [1.0, 1.0, 1.0, 1.0]
    
    # y-delta for every phase of turning at the spot
    _GAIT_TURN_Y = 9
    # x-delta for every phase of turning at the spot
    _GAIT_TURN_X = 27.5
    
    # offset between diagonal legs for symmetric trott gait
    _GAIT_PHASE_OFFSET = (_GAIT_SUPPORT_TICKS + _GAIT_SWING_TICKS) / 2
    _GAIT_LEG_PHASE_OFFSET = [0, _GAIT_PHASE_OFFSET, _GAIT_PHASE_OFFSET, 0]
   
    # neutral foot position offset
    # golden: + 15 (depends on body height i guess, here it was -100)
    FOOT_OFFSET = _GAIT_HALF_STRIDE + 15
    _GAIT_FOOT_X_OFFSET = [FOOT_OFFSET, FOOT_OFFSET, FOOT_OFFSET, FOOT_OFFSET]
    _GAIT_FOOT_0_OFFSET = v3(_GAIT_FOOT_X_OFFSET[0], 0.0, 0.0)
    _GAIT_FOOT_1_OFFSET = v3(_GAIT_FOOT_X_OFFSET[1], 0.0, 0.0)
    _GAIT_FOOT_2_OFFSET = v3(_GAIT_FOOT_X_OFFSET[2], 0.0, 0.0)
    _GAIT_FOOT_3_OFFSET = v3(_GAIT_FOOT_X_OFFSET[3], 0.0, 0.0)
    _GAIT_FOOT_OFFSET = [_GAIT_FOOT_0_OFFSET, _GAIT_FOOT_1_OFFSET, _GAIT_FOOT_2_OFFSET, _GAIT_FOOT_3_OFFSET]
    
    # leg x direction
    # FL: inherent +x is away from body , so fwd -> no correction req., DIR =  1
    # FR: inherent +x is away from body , so bwd -> correction req.,    DIR = -1
    # FR: inherent +x is away from body , so fwd -> no correction req., DIR =  1
    # RR: inherent +x is away from body , so bwd -> correction req.,    DIR = -1        
    _LEG_DIR = [ 1, -1,  1, -1]
    
    # LED control
    _LED_PWM_FREQ = 1000

    
