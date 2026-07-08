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

from machine import Pin
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
        
        # waveshare pico-lcd-1.3 documentaion
        self.GPIO_FWD = 2
        self.GPIO_BWD = 18
        self.GPIO_LFT = 16
        self.GPIO_RGT = 20
        self.GPIO_KCK = 21
        
        self.switch = [Pin(self.GPIO_FWD, Pin.IN, Pin.PULL_UP),
                       Pin(self.GPIO_BWD, Pin.IN, Pin.PULL_UP),
                       Pin(self.GPIO_LFT, Pin.IN, Pin.PULL_UP),
                       Pin(self.GPIO_RGT, Pin.IN, Pin.PULL_UP),
                       Pin(self.GPIO_KCK, Pin.IN, Pin.PULL_UP)]
        
    def get_controller_state(self):
        tmp = []
        chksum = 0
        for i in range(5):
            pin_state = int(1 - self.switch[i].value())
            tmp.append(pin_state)
            chksum += 2**i * pin_state
        return tmp, chksum
        
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
                    
                    pin_state_new, chksum_new = self.get_controller_state()
                    pin_state_old = pin_state_new
                    chksum_old    = chksum_new
                    
                    while True:
                        print(self.get_controller_state())
                        pin_state_new, chksum_new = self.get_controller_state()
                        if chksum_new != chksum_old:
                            data = struct.pack('<I', chksum_new)                            
                            await char_cmd.write(data, response=False, timeout_ms=2000)
                        pin_state_old = pin_state_new
                        chksum_old    = chksum_new
                        await asyncio.sleep(0.15)                        
                    
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
