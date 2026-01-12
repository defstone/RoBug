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
from robug_utils import v3
from robug_constants import constants as c
from robug_robot import robug
from robug_com import rbcom

############################
## class rbmocon
############################
class rbmocon:

    def __init__(self, robot, MsgQueue, RplyQueue, debug=False):
        self.com = rbcom(MsgQueue, RplyQueue)           
        self.r = robot
        self.fLoopTime  = c._GAIT_LOOP_TIME
        self.bRunLoop = False
        self.bAcceptNewCmd = True
        self.iPhase = 1
        self.State = ''

    def start_step(self):
        r = self.r
        com = self.com
        
        if self.iPhase == 1:
            # re-enable support leg z-push for smooth gait 
            r.push_disable()                    
            self.bRunLoop = True
            r.lLeg[0].gait.set_az(0)
            r.lLeg[3].gait.set_az(0)                    
            self.iPhase = 2
            
        elif self.iPhase == 2:
            if r.is_support_end(1):
                r.lLeg[0].gait.calc_parameters_fullstep()
                r.lLeg[1].gait.calc_parameters_fullstep()
                r.lLeg[2].gait.calc_parameters_fullstep()
                r.lLeg[3].gait.calc_parameters_fullstep()
                self.iPhase = 1
                self.bRunLoop = False
                self.bAcceptNewCmd = True
                com.command_complete()
                
    async def timeSlice(self, fLoopTime):
        await asyncio.sleep_ms(fLoopTime)                
                
    def resume(self, strSubCmd):
        r = self.r
        com = self.com
        
        if   strSubCmd == '_cmd_FWD_':
            r.set_direction( 1, 'x')
        elif strSubCmd == '_cmd_BWD_':
            r.set_direction(-1, 'x')
        else: com.subcommand_unknown()
        
        self.bRunLoop = True
        com.command_complete()
        self.bAcceptNewCmd = True        
                
    def stop_step(self):
        r = self.r
        com = self.com
        
        if self.iPhase == 1:
            if r.is_support_end(0):
                center = c._GAIT_SUPPORT_TICKS/2
                xreach = r.lLeg[0].gait.get_xreach()
                dist  = center - r.lLeg[1].gait.get_loop_counter()
                dxrtn = center/dist
                r.lLeg[0].gait.set_dxrtn(xreach, dist)
                r.lLeg[0].gait.set_daz(dist)                        
                r.lLeg[3].gait.set_dxrtn(xreach, dist)                            
                r.lLeg[3].gait.set_daz(dist)
                # disable support leg z-push for clean stop position 
                r.push_disable()
                self.iPhase = 2
                
        elif self.iPhase == 2:
            if r.is_support_mid(1):
                self.iPhase = 1
                self.bRunLoop = False  
                self.bAcceptNewCmd = True
                com.command_complete()
                                      
    async def attack(self):
        r = self.r
        com = self.com
        lTmpPos = []
            
        lRelPos = [v3(  0,   0,  10), v3(  0,   0,  10), v3(  0,   0,  10), v3(  0,   0,  10)]
        await r.set_positions_relative(lRelPos, 14)             
            
        lRelPos = [v3(-20,   0, -30), v3(-20,   0, -40), v3(-20,   0, -30), v3(-20,   0, -40)]
        await r.set_positions_relative(lRelPos, 3)
        
        await asyncio.sleep_ms(50)
                     
        lRelPos = [v3( 20,   0,  40), v3( 20,   0,  30), v3( 20,   0,  40), v3( 20,   0,  30)]
        await r.set_positions_relative(lRelPos, 4)                         

        self.bAcceptNewCmd = True
        com.command_complete()
        
    async def turn_l(self):
        r = self.r
        com = self.com
        lTmpPos = []
        
        yjump = 8
        xturn = 25
        
        # save foot positions
        for i in range(4):
            lTmpPos.append(r.lLeg[i].foot_pos.to_list())
                            
        # push
        lRelPos = [v3(0, 0,  -1*yjump), v3(     0, 0, -1*yjump), v3(     0, 0, -1*yjump), v3(0, 0,  -1*yjump)]
        await r.set_positions_relative(lRelPos, 5)                    
        # push and turn
        lRelPos = [v3(0, 0,  2*yjump), v3( xturn, 0, -1*yjump), v3(-xturn, 0, -1*yjump), v3(0, 0,  2*yjump)]        
        await r.set_positions_relative(lRelPos, 6) 
        # land
        lRelPos = [v3(0, 0, -1*yjump), v3(     0, 0,  2*yjump), v3(     0, 0,  2*yjump), v3(0, 0, -1*yjump)]
        await r.set_positions_relative(lRelPos, 8) 
        await asyncio.sleep_ms(100)

        # lift tips before moving them on x axis
        lRelPos = [v3(0, 0, 0), v3(     0, 0,  1*yjump), v3(    0, 0,  1*yjump), v3(0, 0, 0)]
        await r.set_positions_relative(lRelPos, 5)                 
        # lift and restore initial x position
        lRelPos = [v3(0, 0, 0), v3(-xturn, 0,  2*yjump), v3(xturn, 0,  2*yjump), v3(0, 0, 0)]        
        await r.set_positions_relative(lRelPos, 6) 
        # make ground contact
        lRelPos = [v3(0, 0, 0), v3(     0, 0, -3*yjump), v3(    0, 0, -3*yjump), v3(0, 0, 0)]        
        await r.set_positions_relative(lRelPos, 8)         
        await asyncio.sleep_ms(100)
        
        #restore foot positions
        for i in range(4):
            r.lLeg[i].foot_pos.set_from_list(lTmpPos[i])
        r.set_positions(None)            

        self.bAcceptNewCmd = True
        com.command_complete()
        
    async def turn_r(self):
        r = self.r
        com = self.com
        lTmpPos = []
        
        yjump = c._GAIT_TURN_Y
        xturn = c._GAIT_TURN_X
        
        # save foot positions
        for i in range(4):
            lTmpPos.append(r.lLeg[i].foot_pos.to_list())
                                
        # push
        lRelPos = [v3(     0, 0, -1*yjump), v3(0, 0, -1*yjump), v3(0, 0, -1*yjump), v3(     0, 0, -1*yjump)]
        await r.set_positions_relative(lRelPos, 5)                    
        # push and turn
        lRelPos = [v3(-xturn, 0, -1*yjump), v3(0, 0,  2*yjump), v3(0, 0,  2*yjump), v3( xturn, 0, -1*yjump)]        
        await r.set_positions_relative(lRelPos, 6) 
        # land
        lRelPos = [v3(     0, 0,  2*yjump), v3(0, 0, -1*yjump), v3(0, 0, -1*yjump), v3(     0, 0,  2*yjump)]
        await r.set_positions_relative(lRelPos, 8) 
        await asyncio.sleep_ms(100)

        # lift tips before moving them on x axis
        lRelPos = [v3(     0, 0,  1*yjump), v3(0, 0, 0), v3(0, 0, 0), v3(     0, 0,  1*yjump)]
        await r.set_positions_relative(lRelPos, 5)                 
        # lift and restore initial x position
        lRelPos = [v3( xturn, 0,  2*yjump), v3(0, 0, 0), v3(0, 0, 0), v3(-xturn, 0,  2*yjump)]        
        await r.set_positions_relative(lRelPos, 6) 
        # make ground contact
        lRelPos = [v3(     0, 0, -3*yjump), v3(0, 0, 0), v3(0, 0, 0), v3(     0, 0, -3*yjump)]        
        await r.set_positions_relative(lRelPos, 8)         
        await asyncio.sleep_ms(100)
        
        #restore foot positions
        for i in range(4):
            r.lLeg[i].foot_pos.set_from_list(lTmpPos[i])
        r.set_positions(None)            

        self.bAcceptNewCmd = True
        com.command_complete()        

    async def run(self):

        # aliases for convenience
        r = self.r
        com = self.com

        while True:
            timer_task = asyncio.create_task(self.timeSlice(self.fLoopTime))

            # execute prepared update from end of last loop
            r.set_joints()

            # if no command is in progress get new command
            if self.bAcceptNewCmd:
                strCmd, strSubCmd = com.get_command()
                self.bAcceptNewCmd = False
                
            # process current cmd/subcmd
            if   strCmd == '_cmd_NOP_':        self.bAcceptNewCmd = True
            
            elif strCmd == '_cmd_START_STEP_': self.start_step()
                  
            elif strCmd == '_cmd_RESUME_':     self.resume(strSubCmd)
            
            elif strCmd == '_cmd_STOP_STEP_':  self.stop_step()
                        
            elif strCmd == '_cmd_TURN_LFT_':   await self.turn_l()
            
            elif strCmd == '_cmd_TURN_RGT_':   await self.turn_r()            

            elif strCmd == '_cmd_EXIT_':
                
                strMotionState  = '_state_EXITING_'
                print('exit, waiting for timing thread to complete')
                await timer_task
                com.command_complete()
                break
            
            else:               
                self.bAcceptNewCmd = True
                com.command_unknown(strCmd)
                
            # calculate new joint angles
            if self.bRunLoop:
                r.inc_loop_counters()
                r.calculate_joint_angles(bAbs=False)
            await timer_task
