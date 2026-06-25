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
import sys
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
        self.pos_ls = []
        
    async def timeSlice(self, fLoopTime):
        await asyncio.sleep_ms(fLoopTime)        
        
    def sign(self, num):
        return -1 if num < 0 else 1        
        
    def rotate_point_center(self, p, theta):
        # Calculate cosine and sine of the angle
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        tmp = v3()
        # Apply rotation matrix
        tmp.x = p.x * cos_theta - p.z * sin_theta
        tmp.z = p.x * sin_theta + p.z * cos_theta
        return tmp

    def rotate_point_point(self, p, o, a):
        # Translate point to origin
        p.sub(o)
        # Rotate using the same formula
        tmp = self.rotate_point_center(p, math.radians(a))
        # Translate back to original position
        return tmp.add(o)
    
    async def rotate_body(self, theta):
        r = self.r
        com = self.com
        sign = self.sign(theta)
        
        # determin vector from pivot to foot position
        lOF = []
        for i in range(4):
            tmp = r.lLeg[i].foot_pos
            tmp.add(c._DIST_PIVOT_TO_HIP)
            lOF.append(tmp)
        
        j = 4
        thetaxj = (theta * j)
        for a in range(1, abs(thetaxj)+1):
            
            # rotate foor position vector relative to body in opposite direction
            r.lLeg[0].foot_pos = self.rotate_point_point(lOF[0], v3(0,0,0),  a * sign/j)
            r.lLeg[1].foot_pos = self.rotate_point_point(lOF[1], v3(0,0,0), -a * sign/j)
            r.lLeg[2].foot_pos = self.rotate_point_point(lOF[2], v3(0,0,0),  a * sign/j)
            r.lLeg[3].foot_pos = self.rotate_point_point(lOF[3], v3(0,0,0), -a * sign/j)
            
            for i in range(4):
                # the ik solver solves reative to hip joint
                # so calc new foot position relative to hip joint
                r.lLeg[i].foot_pos.sub(c._DIST_PIVOT_TO_HIP)
                # solve foot position
                r.lLeg[i].solve()               
            
            for i in range(4):
                # set joint to new joint angles
                r.lLeg[i].set_joints()
            
            # wait until foot position update done
            await asyncio.sleep_ms(10)
            
        self.bAcceptNewCmd = True
        com.command_complete()
        
    def set_direction(self, strSubCmd):
        r = self.r
        if   strSubCmd == '_cmd_FWD_':
            r.set_direction( 1, 'x')
        elif strSubCmd == '_cmd_BWD_':
            r.set_direction(-1, 'x')
        else: com.subcommand_unknown()
        
    def set_body_lean(self, strSubCmd):
        r = self.r
        if   strSubCmd == '_cmd_FWD_':
            r.set_body_lean( 1, c._ASYM_XSHIFT)
        elif strSubCmd == '_cmd_BWD_':
            r.set_body_lean(-1, c._ASYM_XSHIFT)
        else: com.subcommand_unknown()        
        
    async def shift_CoM(self, strSubCmd):
        print('shifting CoM')
        r = self.r
        com = self.com
        if   strSubCmd == '_cmd_FWD_':
            vTmp = v3(-15, 0, 0)
            lRelPos = [vTmp, vTmp, vTmp, vTmp]            
            await r.set_positions_relative(lRelPos, 25)
        elif strSubCmd == '_cmd_BWD_':
            vTmp = v3( 15, 0, 0)
            lRelPos = [vTmp, vTmp, vTmp, vTmp]            
            await r.set_positions_relative(lRelPos, 25)
        self.bAcceptNewCmd = True
        com.command_complete()               
                
    def resume(self, strSubCmd):
        r = self.r
        com = self.com
        self.set_direction(strSubCmd)
        self.bRunLoop = True
        self.bAcceptNewCmd = True
        com.command_complete()
        
    def pause(self):
        r = self.r
        com = self.com
        if r.is_support_start(0):
            self.bRunLoop = False
            self.bAcceptNewCmd = True
            com.command_complete()         
                
    async def start_animation(self, strSubCmd):
        r = self.r
        com = self.com
        self.bRunLoop = False
        self.set_direction(strSubCmd)
        self.set_body_lean(strSubCmd)
        r.push_disable()

        # capture current positions in leg space
        pos_ls_current = [r.lLeg[i].get_foot_pos() for i in range(4)]
        # reset counter to start position 
        r.reset_loop_counter()
        # calculate goal position in leg space
        r.calculate_foot_positions(bAbs=True)
        pos_ls_goal = [r.lLeg[i].get_foot_pos() for i in range(4)]
        # leg0 is reference leg, needs c._GAIT_SWING_TICKS/2
        # from support path midpoint to support path start
        di = c._GAIT_SWING_TICKS/2
        # calculate dx[]
        dx = [(pos_ls_goal[i].x - pos_ls_current[i].x)/di for i in range(4)]
        x = [0, 0, 0, 0]
                
        # angle steps for halfsine trajectory
        da  = c._PI / di
        a   = 0        
        daz = 0
        
        # animation        
        for _ in range(di):
            
            # start timer task
            timer_task = asyncio.create_task(self.timeSlice(self.fLoopTime))
            
            # z contribution of arc (halfsine)
            a += da
            daz = c._GAIT_SWING_AMPL * math.sin(a)
            
            for i in range(4):
                # increment x/z pos. for all legs                
                x[i] += dx[i]
                # foot_pos is the physical pos in leg space
                r.lLeg[i].foot_pos.x = pos_ls_current[i].x + x[i]
                
            # superimpose arc trajectory on leg 0 and 3
            r.lLeg[0].foot_pos.z = pos_ls_current[0].z + daz
            r.lLeg[3].foot_pos.z = pos_ls_current[3].z + daz
            # solve ik for all legs
            r.solve_ik()
            # update joints of all legs                
            r.set_joints()
            # wait for timer completion
            await timer_task
            
        # clean up
        r.push_enable()        
        self.bAcceptNewCmd = True
        com.command_complete()
                
    async def stop_animation(self, strSubCmd):
        r = self.r
        com = self.com
        self.bRunLoop = False 
        
        # loop counter value of support path midpoint
        i_support_mid = r.lLeg[1].gait.get_support_mid()
        
        # all for legs have to reach the midpoint in di steps
        di = i_support_mid - r.lLeg[1].gait.get_loop_counter()        
        
        # get current foot positions in leg space
        pos_ls = [r.lLeg[i].get_foot_pos() for i in range(4)]
        
        # calculate dx per tick, move towards center (x=0 in gait space)
        # target x = c._FOOT_XOFFSET in leg space
        dx = [((pos_ls[i].x - c._FOOT_XOFFSET) / di) * -1 for i in range(4)]
        x = [0, 0, 0, 0]        
        
        # calculate dz per tick so that after di ticks
        # all z-values are c._GAIT_HEIGHT
        dz = [(c._GAIT_HEIGHT-pos_ls[i].z) / di for i in range(4)]
        z = [0, 0, 0, 0]
        
        # angle steps for halfsine trajectory
        da  = c._PI / di
        a   = 0        
        daz = 0

        # animation        
        for _ in range(di):
            
            # start timer task
            timer_task = asyncio.create_task(self.timeSlice(self.fLoopTime))
            
            # z contribution of arc (halfsine)
            a += da
            daz = c._GAIT_SWING_AMPL * math.sin(a)
            
            for i in range(4):
                # increment x/z pos. for all legs                
                x[i] += dx[i]
                z[i] += dz[i]
                # foot_pos is the physical pos in leg space
                r.lLeg[i].foot_pos.x = pos_ls[i].x + x[i]
                r.lLeg[i].foot_pos.z = pos_ls[i].z + z[i]
                
            # superimpose arc trajectory on leg 0 and 3
            r.lLeg[0].foot_pos.z += daz
            r.lLeg[3].foot_pos.z += daz
            # solve ik for all legs
            r.solve_ik()
            # update joints of all legs                
            r.set_joints()
            # wait for timer completion
            await timer_task

        for i in range(4):
            print(r.lLeg[i].foot_pos)
        
        # clean up
        self.bAcceptNewCmd = True
        com.command_complete()
   
    async def sit_down(self):
        r = self.r
        com = self.com
        lRelPos = [v3(  0,   0,  40), v3(  0,   0,  40), v3(  0,   0,  40), v3(  0,   0,  40)]
        await r.set_positions_relative(lRelPos, 35)
        self.bAcceptNewCmd = True
        com.command_complete()
        
    async def stand_up(self):
        r = self.r
        com = self.com        
        lRelPos = [v3(  0,   0, -40), v3(  0,   0, -40), v3(  0,   0, -40), v3(  0,   0, -40)]
        await r.set_positions_relative(lRelPos, 35)
        await asyncio.sleep(0.5)
        self.bAcceptNewCmd = True
        com.command_complete()
        
    async def purr(self):
        r = self.r
        com = self.com
        for i in range(15):
            lRelPos = [v3(  0,   0,  -2), v3(  0,   0,  -2), v3(  0,   0,  -2), v3(  0,   0,  -2)]
            await r.set_positions_relative(lRelPos, 2)            
            await asyncio.sleep_ms(5)
            lRelPos = [v3(  0,   0,   2), v3(  0,   0,   2), v3(  0,   0,   2), v3(  0,   0,   2)]
            await r.set_positions_relative(lRelPos, 2)            
            await asyncio.sleep_ms(5)
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
        
        yjump = c._GAIT_TURN_Y
        xturn = c._GAIT_TURN_X
        
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

        # lift tips before moving them on x axis
        lRelPos = [v3(0, 0, 0), v3(     0, 0,  1*yjump), v3(    0, 0,  1*yjump), v3(0, 0, 0)]
        await r.set_positions_relative(lRelPos, 5)                 
        # lift and restore initial x position
        lRelPos = [v3(0, 0, 0), v3(-xturn, 0,  2*yjump), v3(xturn, 0,  2*yjump), v3(0, 0, 0)]        
        await r.set_positions_relative(lRelPos, 6) 
        # make ground contact
        lRelPos = [v3(0, 0, 0), v3(     0, 0, -3*yjump), v3(    0, 0, -3*yjump), v3(0, 0, 0)]        
        await r.set_positions_relative(lRelPos, 8)         
        await asyncio.sleep_ms(25)
        
        #restore foot positions
        for i in range(4):
            r.lLeg[i].foot_pos.set_from_list(lTmpPos[i])
        r.solve_ik()
        r.set_joints()

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

        # lift tips before moving them on x axis
        lRelPos = [v3(     0, 0,  1*yjump), v3(0, 0, 0), v3(0, 0, 0), v3(     0, 0,  1*yjump)]
        await r.set_positions_relative(lRelPos, 5)                 
        # lift and restore initial x position
        lRelPos = [v3( xturn, 0,  2*yjump), v3(0, 0, 0), v3(0, 0, 0), v3(-xturn, 0,  2*yjump)]        
        await r.set_positions_relative(lRelPos, 6) 
        # make ground contact
        lRelPos = [v3(     0, 0, -3*yjump), v3(0, 0, 0), v3(0, 0, 0), v3(     0, 0, -3*yjump)]        
        await r.set_positions_relative(lRelPos, 8)         
        await asyncio.sleep_ms(25)
        
        #restore foot positions
        for i in range(4):
            r.lLeg[i].foot_pos.set_from_list(lTmpPos[i])
        r.solve_ik()
        r.set_joints()           

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
            
            elif strCmd == '_cmd_START_STEP_': await self.start_animation(strSubCmd)            
                  
            elif strCmd == '_cmd_RESUME_':     self.resume(strSubCmd)
            
            elif strCmd == '_cmd_PAUSE_':      self.pause()            
            
            elif strCmd == '_cmd_STOP_STEP_':
                if r.is_support_end(0):
                    await timer_task
                    await self.stop_animation(strSubCmd)
                    
            elif strCmd == '_cmd_WALK_LFT_':
                if r.is_stable():
                    r.set_gait_gains('left')
                    self.bAcceptNewCmd = True
                    com.command_complete()                    

            elif strCmd == '_cmd_WALK_RGT_':
                if r.is_stable():
                    r.set_gait_gains('right')
                    self.bAcceptNewCmd = True
                    com.command_complete()                    
            
            elif strCmd == '_cmd_WALK_STRGT_':
                if r.is_stable():
                    r.set_gait_gains('straight')
                    self.bAcceptNewCmd = True
                    com.command_complete()                    
            
            elif strCmd == '_cmd_TURN_LFT_':   await self.turn_l()      
            
            elif strCmd == '_cmd_TURN_RGT_':   await self.turn_r()
            
            elif strCmd == '_cmd_SIT_DOWN_':   await self.sit_down()
            
            elif strCmd == '_cmd_STAND_UP_':   await self.stand_up()
            
            elif strCmd == '_cmd_PURR_':       await self.purr()
            
            elif strCmd == '_cmd_ROTATE_DN_':  await self.rotate_body( 15)
            
            elif strCmd == '_cmd_ROTATE_UP_':  await self.rotate_body(-15)
            
            elif strCmd == '_cmd_SHIFT_COM_':  await self.shift_CoM(strSubCmd)            

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
                
                # to include body IK:
                # run gait generator substep to generate foot positions relative to hip for unrotated body
                # calculate foot positions relative to center of rotation
                # rotate body around center by theta = rotate [foot positions relative to center] around center by -theta
                # rotated foot positions are relative to center of rotation
                # calculate new foot positions relative to hip
                # solve ik to generate joint angles
                # update joints
                # bam, that's it
                
                r.inc_loop_counters()
                r.calculate_foot_positions(bAbs=False)
                r.solve_ik()
                r.set_joints()
                
            await timer_task
