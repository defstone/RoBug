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

import asyncio
from collections import deque

class rbctrl:
    
    def __init__(self, MsgQueue, RplyQueue):
        self.txq = MsgQueue
        self.rxq = RplyQueue
        
    async def wait_for_reply(self):   
        while True:
            if len(self.rxq) > 0:
                strRply = self.rxq.pop()
                print(f'client received reply: {strRply}')
                return strRply
            await asyncio.sleep_ms(100)
            
    async def send_cmd(self, strCMD):
        self.txq.append(strCMD)
        print(f'supervisor sending {strCMD}')
        reply = await self.wait_for_reply()
        if reply == 'DONE': return True
        else: return False
        
    async def init_pose(self):
        await self.send_cmd('RESUME_FWD')
        await asyncio.sleep(0.1)
        await self.send_cmd('STOP_POSE_FWD')
        
    async def shift_com_fwd(self):
        await self.send_cmd('SHIFT_COM_FWD')
        
    async def shift_com_bwd(self):
        await self.send_cmd('SHIFT_COM_BWD')            

    async def resume_fwd(self):
         await self.send_cmd('RESUME_FWD')
         
    async def resume_bwd(self):
         await self.send_cmd('RESUME_BWD')     

    async def stop_fwd(self):
         await self.send_cmd('STOP_POSE_FWD')
         
    async def stop_bwd(self):
         await self.send_cmd('STOP_POSE_BWD')     

    async def start_to_walk_fwd(self):
        await self.send_cmd('START_POSE_FWD')            
        await self.send_cmd('RESUME_FWD')
        
    async def start_to_walk_bwd(self):
        await self.send_cmd('START_POSE_BWD')            
        await self.send_cmd('RESUME_BWD')    
        
    async def turn(self, dir, times):
        for i in range(times):
            if dir == -1:
                await self.send_cmd('TURN_LFT')
            else:
                await self.send_cmd('TURN_RGT')
        
    async def sit_down(self):
        await self.send_cmd('SIT_DOWN')
        
    async def stand_up(self):
        await self.send_cmd('STAND_UP')
        