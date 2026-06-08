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

        # service + characteristic
        self.service = aioble.Service(self.SERVICE_UUID)
        # write characterisic -> set 'write_no_response=True' to avoid blocking of R/C android app 
        self.char_cmd = aioble.Characteristic(self.service, self.CHAR_UUID1, write=True, write_no_response=True)
        # read + notify characterisic
        self.char_notify = aioble.Characteristic(self.service, self.CHAR_UUID2, read=True, notify=True)        
        #register service with one characteristic: write
        aioble.register_services(self.service)
        
        self.cmd = 'STOP'
        self.value = 9999
        self.mode = 'rc'
        self.code = 0xFF

    # command handling
    def handle_command_rc(self, msg):
        if msg == 0x91:
            self.cmd = 'FWD'
        elif msg == 0x92:
            self.cmd = 'BWD'
        elif msg == 0x93:
            self.cmd = 'LEFT'
        elif msg == 0x94:
            self.cmd = 'RIGHT'
        elif msg == 0x90:
            self.cmd = 'STOP'
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
    
    async def send_data(self, connection):
        while connection.is_connected():        
            data = struct.pack('<I', self.value)
            # NOTIFY AND WRITE ARE NOT FUCKING AWAITABLE
            # ARGHHHH
            self.char_notify.notify(connection, data)
            await asyncio.sleep_ms(500)

    async def receive_cmd(self, connection):
        while connection.is_connected():
            await self.char_cmd.written()
            data = self.char_cmd.read()
            if data:
                cmd = data[0]
                if self.mode == 'rc':
                    self.handle_command_rc(cmd)
                elif self.mode == 'raw':
                    self.handle_command_raw(cmd)
                               
    async def msg_handler(self):
        while True:
            print('Waiting for connection...')
            connection = await aioble.advertise(50_000, name='RoBug', services=[self.SERVICE_UUID])
            print('Connected!')

            async with connection:
                tx_task = asyncio.create_task(self.send_data(connection))
                rx_task = asyncio.create_task(self.receive_cmd(connection))
                await asyncio.gather(tx_task, rx_task)
                
    def set_mode(self, mode):
        self.mode = mode
        
    def get_mode():
        return self.mode

if __name__ == '__main__':
    
    async def main(b):
        msg_server = asyncio.create_task(b.msg_handler())
        await msg_server 

    ble = rbble()
    asyncio.run(main(ble))
