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

class rbrc:
    
    def __init__(self):
        # service and characteristic UUID:
        # https://www.uuidgenerator.net
        self.SERVICE_UUID = bluetooth.UUID('7f257115-9cdd-4e63-81f8-ca2bb1fad841')
        self.CHAR_UUID1   = bluetooth.UUID('ba3ca205-e3fb-4727-ad69-878bb3038b00')
        self.CHAR_UUID2   = bluetooth.UUID('ba3ca205-e3fb-4727-ad69-878bb3038b01')
        self.CHAR_UUID3   = bluetooth.UUID('ba3ca205-e3fb-4727-ad69-878bb3038b02')
        
        self.FWD_P = 0x90
        self.FWD_R = 0X91
        self.BWD_P = 0x92
        self.BWD_R = 0X93
        self.LFT_P = 0xA0
        self.LFT_R = 0XA1
        self.RGT_P = 0xA2
        self.RGT_R = 0XA3        

    async def ble_scan(self):
        async with aioble.scan(duration_ms=5000, interval_us=30000, window_us=30000, active=True) as scanner:
            async for result in scanner:
                if result.name():
                    if 'RoBug' in result.name():
                        print(f"RoBug found: {result.name()} [{result.device.addr_hex()}]")
                        return result.device
                
    async def ble_connection(self):
        while True:
            device = await self.ble_scan()
            
            if not device:
                print('Robug not found, retrying')
                await asyncio.sleep(1)
                continue
                
            try:
                print(f'connectiong to {device.addr_hex()}...')
                connection = await device.connect()
                print('connected')
                
                async with connection:
                    # get service of ble server a.k.a. RoBug
                    service = await connection.service(self.SERVICE_UUID)
                    # get write characteristic of ble server a.k.a. RoBug
                    char_cmd = await service.characteristic(self.CHAR_UUID1)
                    print(service)
                    print(char_cmd)                    
                
                    while True:
                        data = struct.pack('<I', 0x90)
                        await char_cmd.write(data, response=False, timeout_ms=2000)
                        await asyncio.sleep(2)
                        data = struct.pack('<I', 0x91)
                        await char_cmd.write(data, response=False, timeout_ms=2000)
                        await asyncio.sleep(2)                        
                    
            except asyncio.TimeoutError:
                print('connection failed - timeout')
            except aioble.GattError:
                print('GATT error: service or characteristic not found')
            except Exception as e:
                print(f'connection interrupted error {e}')
                
            print(f'connection closed - restarting ble scan')
            await asyncio.sleep(1)                
            
            
rc = rbrc()
asyncio.run(rc.ble_connection())




