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
import bluetooth
import aioble
import struct

class rbble:
    
    def __init__(self):
        # service and characteristic UUID:
        # https://www.uuidgenerator.net
        self.SERVICE_UUID = bluetooth.UUID('7f257115-9cdd-4e63-81f8-ca2bb1fad841')
        self.CHAR_UUID1   = bluetooth.UUID('ba3ca205-e3fb-4727-ad69-878bb3038b00')
        self.CHAR_UUID2   = bluetooth.UUID('ba3ca205-e3fb-4727-ad69-878bb3038b01')
        self.CHAR_UUID3   = bluetooth.UUID('ba3ca205-e3fb-4727-ad69-878bb3038b02')        

        # service + characteristic
        self.service = aioble.Service(self.SERVICE_UUID)
        # write characterisic -> set 'write_no_response=True' to avoid blocking of R/C android app 
        self.char_cmd = aioble.Characteristic(self.service, self.CHAR_UUID1, write=True, write_no_response=True)
        # read + notify characterisic for distance data
        self.char_notify_dist = aioble.Characteristic(self.service, self.CHAR_UUID2, read=True, notify=True)
        # read + notify characterisic for battery level
        self.char_notify_soc = aioble.Characteristic(self.service, self.CHAR_UUID3, read=True, notify=True)        
        #register service with one characteristic: write
        aioble.register_services(self.service)
        
        self.event = False
        self.btn_fwd = False
        self.btn_bwd = False
        self.btn_lft = False
        self.btn_rgt = False
        
        self.cmd = 'STOP'
        self.dist = 9999
        self.soc  = 9999        
        self.mode = 'rc'
        self.code = 0xFF
        
    def handle_buttons(self,msg):
        if msg == 0x90:
            self.btn_fwd = True
        elif msg == 0x91:
            self.btn_fwd = False
        elif msg == 0x92:
            self.btn_bwd = True
        elif msg == 0x93:
            self.btn_bwd = False
        elif msg == 0xA0:
            self.btn_lft = True
        elif msg == 0xA1:
            self.btn_lft = False
        elif msg == 0xA2:
            self.btn_rgt = True
        elif msg == 0xA3:
            self.btn_rgt = False
        else:
            self.btn_fwd = False
            self.btn_bwd = False
            self.btn_lft = False
            self.btn_rgt = False
        
    # command handling
    def handle_command_rc(self, msg):
        if msg == 0x90:
            self.cmd = 'FWD'
        elif msg == 0x91:
            self.cmd = 'BWD'
        elif msg == 0x92:
            self.cmd = 'STOP_FWD_BWD'            
        elif msg == 0xA0:
            self.cmd = 'LEFT'
        elif msg == 0xA1:
            self.cmd = 'RIGHT'
        elif msg == 0xA2:
            self.cmd = 'STOP_LEFT_RIGHT'
        else:
            print('UNKNOWN:', msg)
            self.cmd = 'STOP'
        print(self.cmd)
        
    def handle_command_raw(self, msg):
        self.code = msg
        # print(self.code)        
            
    def get_current_cmd(self):
        return self.cmd
    
    def get_current_code(self):
        # burn after reading ;-)
        tmp = self.code
        self.code = 0xFF
        return tmp    
    
    async def send_data_dist(self, connection):
        while connection.is_connected():        
            data = struct.pack('<I', self.dist)
            self.char_notify_dist.notify(connection, data)
            await asyncio.sleep_ms(500)
            
    async def send_data_soc(self, connection):
        while connection.is_connected():        
            data = struct.pack('<I', self.soc)
            self.char_notify_soc.notify(connection, data)
            await asyncio.sleep(2)            

    async def receive_cmd(self, connection):
        while connection.is_connected():
            await self.char_cmd.written()
            data = self.char_cmd.read()
            if data:
                cmd = data[0]
                if self.mode == 'rc':
                    # self.handle_command_rc(cmd)
                    self.handle_buttons(cmd)                    
                elif self.mode == 'raw':
                    self.handle_command_raw(cmd)
                               
    async def msg_handler(self):
        while True:
            print('Waiting for connection...')
            connection = await aioble.advertise(50_000, name='RoBug', services=[self.SERVICE_UUID])
            print('Connected!')

            async with connection:
                tx_task0 = asyncio.create_task(self.send_data_dist(connection))
                tx_task1 = asyncio.create_task(self.send_data_soc(connection))                
                rx_task0 = asyncio.create_task(self.receive_cmd(connection))
                await asyncio.gather(tx_task0, tx_task1, rx_task0)
                
    def set_mode(self, mode):
        self.mode = mode
        
    def get_mode():
        return self.mode

if __name__ == '__main__':
    
    async def main(b):
        msg_server = asyncio.create_task(b.msg_handler())
        
        while True:
            print(b.btn_fwd, b.btn_bwd, b.btn_lft, b.btn_rgt)
            await asyncio.sleep(1)
        await msg_server 

    ble = rbble()
    asyncio.run(main(ble))
