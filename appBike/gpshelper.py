from kivy.utils import platform
from kivymd.uix.dialog import MDDialog
import asyncio
import time

class GpsHelper:
    gps_time: float = 500
    gps_min_distance: float = 3.0
    velocity: int = 0
   
    def run(self, velocity_queue: asyncio.Queue) -> None:
        print('in_gps')
        self.velocity_queue = velocity_queue
    
        # configure GPS
        if platform == 'android' or platform == 'ios':
            from plyer import gps
            gps.configure(on_location=self.on_location, on_status=self.on_auth_status)
            gps.start(minTime=self.gps_time, minDistance=self.gps_min_distance)
            self.start = time.perf_counter()

    def on_location(self, *args, **kwargs):
        self.velocity = (kwargs['speed'] * 3.6)
        self.velocity_queue.put_nowait(self.velocity)
        
    def on_auth_status(self, general_status: str, status_message: str) -> None:
        if general_status == 'provider-enabled':
            pass
        else:
            self.open_gps_access_popup()

    def open_gps_access_popup(self) -> None:
        dialog = MDDialog(title='GPS Error',
                          text="You need to enable GPS access for the app to function properly")
        dialog.size_hint = [0.8, 0.8]
        dialog.pos_hint = {'center_x': 0.5, 'center_y': 0.5}
        dialog.open()
