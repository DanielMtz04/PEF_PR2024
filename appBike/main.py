#Librerias
import os
import asyncio
import json

# Importar librerias para creación de app Kivy e importación de componentes de design kivy
from kivymd.app import MDApp
from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.lang import Builder
from kivy.logger import Logger
from kivy.utils import platform
from kivy.uix.spinner import Spinner, SpinnerOption
from kivy.uix.dropdown import DropDown
from CircularProgressBar import CircularProgressBar
from kivy.factory import Factory

# Importar script BLE, gpshelper y acchelper
from BLE import Connection, communication_manager
from gpshelper import GpsHelper
from AccHelper import AccHelper

# Variables globales
ADDRESS, UUID = None, None
disconnect_flag = {'disconnect': False}


class MainWindow(Screen): pass
class SecondaryWindow(Screen): pass
class WindowManager(ScreenManager): pass
class SpinnerDropdown(DropDown): pass


class App(MDApp):
    screen_flag = True
    dialog = None
    send_q = asyncio.Queue()
    speed_q = asyncio.Queue()
    drop_q = asyncio.Queue()
    battery_q = asyncio.Queue()
    angle_q = asyncio.Queue()
    acc_q = asyncio.Queue()
    man_q = asyncio.Queue()

    def build(self):
        """setting design for application widget development specifications on design.kv"""
        self.theme_cls.theme_style = 'Dark'
        self.theme_cls.primary_palette = 'Gray'
        # Solicitar permisos para GPS
        self.get_permissions()
        return Builder.load_file(filename='design.kv')

    async def launch_app(self):
        """Asyncronous function for kivy app start"""
        await self.async_run(async_lib='asyncio')

    async def start(self):
        """Asyncronous app making sure start is awaited as coroutine"""
        task = asyncio.create_task(self.launch_app())
        (_, pending) = await asyncio.wait({task}, return_when='FIRST_COMPLETED')

    def on_start(self):
        """On start method for building desired variables for later use"""
        Logger.info("Called start")

        # Renames certain components from app that will be used in other functions
        self.button = self.root.get_screen('main_window').ids.ble_button
        self.root.get_screen('main_window').ids.device_dropdown.text = ''
        self.root.get_screen('main_window').ids.device_dropdown.size = (0, 0)
        self.root.get_screen('main_window').ids.device_dropdown.opacity = 0

        self.circle_bar = self.root.get_screen('secondary_window').ids.circle_progress
        self.speedmeter = self.root.get_screen('secondary_window').ids.speed
        self.an_button = self.root.get_screen('secondary_window').ids.angle_button
        self.manip_button = self.root.get_screen('secondary_window').ids.manip_button
        self.speedmeter.font_size_min = self.speedmeter.font_size
        self.sp_button = self.root.get_screen('secondary_window').ids.sp_button
        self.read_slider_text = self.root.get_screen('secondary_window').ids.read_slider_text

        self.circle_bar.text = f'0%'
        self.read_slider_text.text = f'0 %'
        self.manip_button.text = f'M: 0'

        self.per_button_pressed = True
        self.km_button_pressed = False
        self.angle_button_pressed = False

        self.set_angle = 0
        self.slider_label = 'slider'
        self.slider_value = 0
        self.slider_flag = False
        self.test_counter = 0

    def get_permissions(self):
        # Request permissions on Android
        if platform == 'android':
            from android.permissions import Permission, request_permissions
            def callback(permission, results):
                if all([res for res in results]):
                    print('Got all permissions')
                else:
                    print('Did not get all permissions')
            try:
                request_permissions(
                    [Permission.ACCESS_COARSE_LOCATION, Permission.ACCESS_FINE_LOCATION, Permission.BLUETOOTH,
                     Permission.BLUETOOTH_ADMIN, Permission.WAKE_LOCK, Permission.BLUETOOTH_CONNECT,
                     Permission.ACCESS_BACKGROUND_LOCATION,
                     callback])
            except Exception as e:
                print(e)

    def start_BLE(self, touch: bool) -> None:
        """Function handling BLE connection between App and ESP32"""
        if touch:
            self.root.get_screen('main_window').ids.spinner.active = True
            try:
                self.ble_task = asyncio.create_task(run_BLE(self, self.send_q, self.battery_q, self.drop_q, self.angle_q, self.man_q))
                self.gps_task = GpsHelper().run(self.speed_q)
                self.update_battery_task = asyncio.ensure_future(self.update_battery_value())
                self.update_speed_task = asyncio.ensure_future(self.update_speed_value())
                self.update_manipulation_task = asyncio.ensure_future(self.update_manipulation_value())
            except Exception as e:
                print(e)
                

    def device_clicked(self, _, value: str) -> None:
        """Handles events in main window dropdown"""
        if value == "ESP32":
            self.device_clicked_task = asyncio.ensure_future(self.device_event_selected(value))

    async def device_event_selected(self, value: str) -> None:
        await self.drop_q.put(value)
        self.root.get_screen('main_window').ids.spinner.active = True

    def switch_state_motion_detect(self, _, value: bool) -> None:
        """Switch state for adaptive mode switch. When in adaptive mode, the slider is disabled"""
        if value:
            self.root.get_screen('secondary_window').ids.adapt_slider.disabled = False
            self.send_q.put_nowait(json.dumps({'adaptationMode': 1}))
            self.root.get_screen('secondary_window').ids.adapt_slider.value = self.root.get_screen('secondary_window').ids.adapt_slider.min
        else:
            self.root.get_screen('secondary_window').ids.adapt_slider.disabled = False
            self.send_q.put_nowait(json.dumps({'adaptationMode': 0}))


    def slider_unit_km(self, touch: bool) -> None:
        if touch:
            self.km_button_pressed = True
            self.per_button_pressed = False
            self.read_slider_text.text = f'0 km/h'
            self.root.get_screen('secondary_window').ids.adapt_slider.value = 0
            self.root.get_screen('secondary_window').ids.adapt_slider.max = 40
            self.root.get_screen('secondary_window').ids.adapt_slider.step = 1
            self.root.get_screen('secondary_window').ids.adapt_slider.color = "#A7D0D2"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_inactive = "#A7D0D2"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_active = "#A7D0D2"

    def slider_unit_per(self, touch: bool) -> None:
        if touch:
            self.km_button_pressed = False
            self.per_button_pressed = True
            self.read_slider_text.text = f'0 %'
            self.root.get_screen('secondary_window').ids.adapt_slider.value = 0
            self.root.get_screen('secondary_window').ids.adapt_slider.max = 100
            self.root.get_screen('secondary_window').ids.adapt_slider.step = 5
            self.root.get_screen('secondary_window').ids.adapt_slider.color = "#99998F"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_inactive = "#99998F"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_active = "#99998F"


    def slider_on_value(self, _, value: int) -> None:
        """Sends percentage of assistance wanted to ESP32
        In automatic mode: Use BATTERY and ACCELEROMETER values to determine percentage of use"""

        label = 'slider'

        if self.per_button_pressed:
            self.read_slider_text.text = f'{value} %'
            value = int(180 * value / 100)
            label = 'slider_per'

        elif self.km_button_pressed:
            self.read_slider_text.text = f'{value} km/h'
            self.sp_button.text = f'SP: {value}'
            value = value
            label = 'slider_km'

        # Almacenar si la app esta en modo Automatico (km) o Manual(percentage) y el valor de la asistencia requerida 
        self.slider_label = label
        self.slider_value = value
            
        if value == self.root.get_screen('secondary_window').ids.adapt_slider.max:
            self.root.get_screen('secondary_window').ids.adapt_slider.hint_text_color = "red"
            if not self.slider_flag:
                self.send_q.put_nowait(json.dumps({self.slider_label: self.slider_value}))
                self.slider_flag = True
        elif value == self.root.get_screen('secondary_window').ids.adapt_slider.min and not self.slider_flag:
            self.send_q.put_nowait(json.dumps({self.slider_label: self.slider_value}))
            self.slider_flag = True
        else:
            self.root.get_screen('secondary_window').ids.adapt_slider.hint_text_color = "white"
            self.slider_flag = False

    def slider_touch_up(self, *args) -> None:
        try:
            self.send_q.put_nowait(json.dumps({self.slider_label: self.slider_value}))
        except asyncio.QueueFull:
            pass


    async def update_manipulation_value(self) -> None:
        """FOR DEBUGGING PURPOSES"""
        while True:
            print('in_manipulation')
            try:
                manip = await self.man_q.get()
                manip = int(manip)
                self.manip_button.text = f'M: {manip}'  
            except Exception as e:
                print(f'EXCEPTION IN MANIP: {e}')
                await asyncio.sleep(1.0)

    async def update_speed_value(self) -> None:
        """Monitors current speed of bike"""
        speed = 0
        while True:
            print("in_speed")
            try:
                speed = await self.speed_q.get()
                speed = float(speed)
                print(f"speed-> {speed}")
                await self.send_q.put(json.dumps({'speed': speed}))
            except Exception as e:
                print(f'Exception in speed:: {e}')
                await asyncio.sleep(1.0)
            if float(speed) > 2 * self.speedmeter.end_value / 3.6:
                pass
            else:
                self.speedmeter.set_value = speed - 25
                self.speedmeter.text = f'{int(speed)} km/h'

    async def update_battery_value(self) -> None:
        """Monitors Battery life from bike"""
        max_battery_voltage = 23.7  # V //Voltage gotten when fully charged
        min_battery_voltage = 20.0  # V //Lowest voltage before battery starts getting damaged
        battery_life = 0
        while True:
            print("in_battery")
            try:
                current_battery_life = await self.battery_q.get()
                battery_life = float(current_battery_life)
                battery_life = int(battery_life)
                print(f"battery-> {battery_life}")
                if battery_life > 100:
                    battery_life = int(100)
                elif battery_life < 0:
                    battery_life = 0
                else:
                    self.circle_bar.set_value = battery_life
                    self.circle_bar.text = f'{battery_life}%'
            except Exception as e:
                print(f'Exception in battery:: {e}')
                await asyncio.sleep(2.0)

    # Popup Disconnect
    def screen_flag_1(self, touch: bool) -> None:
        self.screen_flag = True
        print(self.screen_flag)

    def screen_flag_2(self, touch: bool) -> None:
        self.screen_flag = False
        print(self.screen_flag)

    def cancel_disconnect(self, touch: bool) -> None:
        if touch:
            if (self.screen_flag == True):
                self.root.get_screen('secondary_window').ids.nav.switch_tab('screen 1')
            elif (self.screen_flag == False):
                self.root.get_screen('secondary_window').ids.nav.switch_tab('screen 2')

    def accept_disconnect(self, touch: bool) -> None:
        if touch:
            self.root.current = 'main_window'
            self.root.get_screen('secondary_window').ids.nav.switch_tab('screen 1')
            self.reset_ui_and_variables()
            disconnect_flag['disconnect'] = True

    def reset_ui_and_variables(self):
        self.button = self.root.get_screen('main_window').ids.ble_button
        self.root.get_screen('main_window').ids.device_dropdown.text = ''
        self.root.get_screen('main_window').ids.device_dropdown.disabled = True
        self.root.get_screen('main_window').ids.device_dropdown.size = (0, 0)
        self.root.get_screen('main_window').ids.device_dropdown.opacity = 0

        self.circle_bar = self.root.get_screen('secondary_window').ids.circle_progress
        self.speedmeter = self.root.get_screen('secondary_window').ids.speed
        self.an_button = self.root.get_screen('secondary_window').ids.angle_button
        self.manip_button = self.root.get_screen('secondary_window').ids.manip_button
        self.speedmeter.font_size_min = self.speedmeter.font_size
        self.sp_button = self.root.get_screen('secondary_window').ids.sp_button
        self.read_slider_text = self.root.get_screen('secondary_window').ids.read_slider_text

        self.circle_bar.text = f'0%'
        self.read_slider_text.text = f'0 %'
        self.manip_button.text = f'M: x'

        self.per_button_pressed = True
        self.km_button_pressed = False
        self.angle_button_pressed = False

        self.set_angle = 0
        self.slider_label = 'slider'
        self.slider_value = 0
        self.slider_flag = False
        self.test_counter = 0

        self.sp_button.text = f'SP: 0'
        self.read_slider_text.text = f'0 km/h'
        self.root.get_screen('main_window').ids.spinner.active = False
        self.root.get_screen('secondary_window').ids.adapt_slider.value = self.root.get_screen(
            'secondary_window').ids.adapt_slider.min
        self.root.get_screen('secondary_window').ids.adapt_switch.active = False

    # Popup Exit
    def cancel_exit(self, touch: bool) -> None:
        if touch:  # Asegúrate de que este método solo se ejecute en respuesta a un evento de toque
            print("Move to screen 1...")
            self.root.current = "main_window"

    def accept_exit(self, touch: bool) -> None:
        if touch:
            print("Exit app...")
            self.stop()
            os._exit(0)


async def run_BLE(app: MDApp, send_q: asyncio.Queue, battery_q: asyncio.Queue, drop_q: asyncio.Queue,
                  angle_q: asyncio.Queue, man_q: asyncio.Queue) -> None:
    """Asyncronous connection protocol for BLE"""
    print('in run_BLE')
    read_char = "00002A3D-0000-1000-8000-00805f9b34fb"
    # write_char = "00002A58-0000-1000-8000-00805f9b34fb" ## DEPRECATED
    flag = asyncio.Event()
    connection = Connection(loop=loop,
                            uuid=UUID,
                            address=ADDRESS,
                            read_char=read_char,
                            write_char=read_char,
                            flag=flag,
                            app=app,
                            drop_q=drop_q)
    disconnect_flag['disconnect'] = False

    try:
        asyncio.ensure_future(connection.manager())
        asyncio.ensure_future(communication_manager(connection=connection,
                                                    write_char=read_char,
                                                    read_char=read_char,
                                                    send_q=send_q,
                                                    battery_q=battery_q, angle_q=angle_q,
                                                    man_q=man_q, disconnect_flag=disconnect_flag))
        print(f"fetching connection")
        await connection.flag.wait()
    finally:
        print(f"flag status confirmed!")
        AccHelper().run(send_q)

    try:
        app.root.current = 'secondary_window'
    except Exception as e:
        print(f'EXCEPTION WHEN CHANGING WINDOW -> {e}')


if __name__ == '__main__':
    async def mainThread():
        """Creating main thread for asynchronous task definition"""
        BikeApp = App()
        task_runApp = asyncio.create_task(BikeApp.start())
        (done, pending) = await asyncio.wait({task_runApp}, return_when='FIRST_COMPLETED')

    loop = asyncio.get_event_loop()
    asyncio.run(mainThread())