from kivy.utils import platform
from kivy.clock import Clock
import asyncio
import json

class AccHelper:
    sensorEnabled: bool = False
    x = 0
    y = 0
    z = 0
    def run(self, acceleration_queue: asyncio.Queue) -> None:
        print(f'in_accelerometer')
        self.acceleration_queue = acceleration_queue
        if platform == 'android' or platform == 'ios':
            from plyer import accelerometer
            self.accelerometer = accelerometer
            try:
                if not self.sensorEnabled:
                    self.accelerometer.enable()
                    print('accelerometer_enable')
                    asyncio.ensure_future(self.get_acceleration(1))
                    self.sensorEnabled = True
                    self.init_velo = 0
                else:
                    self.accelerometer.disable()
                    self.sensorEnabled = False
            except NotImplementedError:
                import traceback
                traceback.print_exc()

    async def get_acceleration(self, dt):
        while self.sensorEnabled:
            value = self.accelerometer.acceleration[:3]
            if not value == (None,None,None):
               self.x = value[0]
               self.y = value[1]
               self.z = value[2]
               print(f'accelerometer_values: {value}')
               try:
                  #self.acceleration_queue.put_nowait(json.dumps({'acceleration_y': self.y}))
                  self.acceleration_queue.put_nowait(self.y)
               except Exception as e:
                  print(f'EXCEPTION ACCELEROMETER :: {e}')
            await asyncio.sleep(dt)