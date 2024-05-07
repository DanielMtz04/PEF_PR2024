#Librerias
import os
import asyncio
import json
import math

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
from kivymd.uix.button import MDFillRoundFlatButton
from kivy.properties import ColorProperty, NumericProperty
from kivy.properties import StringProperty, ListProperty


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


class CustomButton(MDFillRoundFlatButton):
    border_color = ColorProperty((0, 0, 0, 1))  
    border_width = NumericProperty(1)  


class App(MDApp):

    screen_flag = True
    deviceSelect_queue = asyncio.Queue()
    dataTx_queue = asyncio.Queue()
    velocity_queue = asyncio.Queue()
    acceleration_queue = asyncio.Queue()
    angle_queue = asyncio.Queue()
    battery_queue = asyncio.Queue()
    manipulation_queue = asyncio.Queue()
    
    def build(self):
        """ Configura diseño inicial de la aplicación, obtiene permisos para gps y ble y carga archivo de diseño kv"""
        self.theme_cls.theme_style = 'Dark'
        self.theme_cls.primary_palette = 'Gray'
        self.get_permissions()
        return Builder.load_file(filename='design.kv')

    async def launch_app(self):
        """Lazamiento de aplicación con el manejo de metodos asincronos"""
        await self.async_run(async_lib='asyncio')

    async def start(self):
        """Inicia la app de forma asincrona esperando que la tarea de lanzamiento finalize"""
        task = asyncio.create_task(self.launch_app())
        (_, pending) = await asyncio.wait({task}, return_when='FIRST_COMPLETED')

    def on_start(self):
        """Metodo que se llama al iniciar la aplicación, configurando los componentes iniciales de la interfz gráfica"""
        Logger.info("Called start")

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
        self.manip_button.text = f'Manipulation: 0'

        self.per_button_pressed = True
        self.km_button_pressed = False
        self.angle_button_pressed = False

        self.set_angle = 0
        self.slider_label = 'slider'
        self.slider_value = 0
        self.slider_flag = False
        self.test_counter = 0

        #self.root.current = 'secondary_window'

    def get_permissions(self):
        """Solicita permisos de acceso a ubicación y bluetooth"""
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
        """Metodo que inicia el proceso de conexión y comunicación BLE y monitoreo de sensores(gps y acelerometro)"""
        if touch:
            self.root.get_screen('main_window').ids.spinner.active = True
            try:
                self.ble_task = asyncio.create_task(run_BLE(self, self.dataTx_queue, self.battery_queue, self.deviceSelect_queue, self.angle_queue, self.manipulation_queue))
                self.gps_task = AccHelper().run(self.acceleration_queue)
                self.gps_task = GpsHelper().run(self.velocity_queue)
                self.update_battery_task = asyncio.ensure_future(self.update_battery_value())
                self.update_acceleration_task = asyncio.ensure_future(self.update_acceleration_value())
                self.update_speed_task = asyncio.ensure_future(self.update_speed_value())
                self.update_manipulation_task = asyncio.ensure_future(self.update_manipulation_value())
            except Exception as e:
                print(e)
                

    def device_clicked(self, _, value: str) -> None:
        """Metodo que inicializa el proceso de selección de dispositvo"""
        if value == "ESP32":
            self.device_clicked_task = asyncio.ensure_future(self.device_event_selected(value))

    async def device_event_selected(self, value: str) -> None:
        """Metodo que maneja el proceso de almacenar el dispostivo seleccionado en una queue y la transición a conexión"""
        await self.deviceSelect_queue.put(value)
        self.root.get_screen('main_window').ids.spinner.active = True

    def switch_state_motion_detect(self, _, value: bool) -> None:
        """Metodo que maneja el cambio de estado del switch de Motion Detect y almacena valor actual en una queue"""
        if value:
            self.root.get_screen('secondary_window').ids.adapt_slider.disabled = False
            self.dataTx_queue.put_nowait(json.dumps({'adaptationMode': 1}))
            self.root.get_screen('secondary_window').ids.adapt_slider.value = self.root.get_screen('secondary_window').ids.adapt_slider.min
        else:
            self.root.get_screen('secondary_window').ids.adapt_slider.disabled = False
            self.dataTx_queue.put_nowait(json.dumps({'adaptationMode': 0}))

    def slider_unit_km(self, touch: bool) -> None:
        """Metodo que configura el slider para controlar asistencia con valores en km/h"""
        if touch:
            self.km_button_pressed = True
            self.per_button_pressed = False
            self.read_slider_text.text = f'0 km/h'
            self.root.get_screen('secondary_window').ids.adapt_slider.value = 0.01
            self.root.get_screen('secondary_window').ids.adapt_slider.max = 40
            self.root.get_screen('secondary_window').ids.adapt_slider.step = 1
            self.root.get_screen('secondary_window').ids.adapt_slider.color = "#0BA7DD"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_inactive = "#0BA7DD"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_active = "#0BA7DD"
            self.root.get_screen('secondary_window').ids.adapt_slider.hint_bg_color= "#0BA7DD"
            # Modo automatic activo
            self.root.get_screen('secondary_window').ids.mode.text = '  Automatic  '
            self.root.get_screen('secondary_window').ids.mode.md_bg_color = (11/255, 167/255, 221/255, 1) 

    def slider_unit_per(self, touch: bool) -> None:
        """Metodo que configura el slider para controlar asistencia con valores en porcentajes"""
        if touch:
            self.km_button_pressed = False
            self.per_button_pressed = True
            self.read_slider_text.text = f'0 %'
            self.sp_button.text = f'Set Point: 0'
            self.manip_button.text = f'Manipulation: 0'
            self.root.get_screen('secondary_window').ids.adapt_slider.value = 0.01
            self.root.get_screen('secondary_window').ids.adapt_slider.max = 100
            self.root.get_screen('secondary_window').ids.adapt_slider.step = 5
            self.root.get_screen('secondary_window').ids.adapt_slider.color = "#99998F"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_inactive = "#99998F"
            self.root.get_screen('secondary_window').ids.adapt_slider.thumb_color_active = "#99998F"
            self.root.get_screen('secondary_window').ids.adapt_slider.hint_bg_color= "#99998F"
            # Modo manual activo
            self.root.get_screen('secondary_window').ids.mode.text = '   Manual   '
            self.root.get_screen('secondary_window').ids.mode.md_bg_color = (153/255, 153/255, 143/255, 1)
            
    def slider_on_value(self, _, value: int) -> None:
        """Metodo que actualiza en la interfaz el valor que el usuario selecciona en el slider de control de asistencia y
           en el modo automatico tambien actualiza el valor de Set Point e igualmente manda el valor de asistencia a la ESP32"""

        label = 'slider'

        if self.per_button_pressed:
            if value == 0.01:
                self.read_slider_text.text = f'{0} %'
                value = 0
            else:
                self.read_slider_text.text = f'{value} %'
            value = int(180 * value / 100)
            label = 'slider_per'            

        elif self.km_button_pressed:
            if value == 0.01:
                self.read_slider_text.text = f'{0} km/h'
                value = 0
            else:
                self.read_slider_text.text = f'{value} km/h'
            self.sp_button.text = f'Set Point: {value}'
            value = value
            label = 'slider_km'

        # Almacenan si la app esta en modo Automatico (km) o Manual(percentage) y el valor de la asistencia requerida 
        self.slider_label = label
        self.slider_value = value
            
        if value == self.root.get_screen('secondary_window').ids.adapt_slider.max:
            self.root.get_screen('secondary_window').ids.adapt_slider.hint_text_color = "red"
            if not self.slider_flag:
                self.dataTx_queue.put_nowait(json.dumps({self.slider_label: self.slider_value}))
                self.slider_flag = True
        elif value == self.root.get_screen('secondary_window').ids.adapt_slider.min and not self.slider_flag:
            self.dataTx_queue.put_nowait(json.dumps({self.slider_label: self.slider_value}))
            self.slider_flag = True
        else:
            self.root.get_screen('secondary_window').ids.adapt_slider.hint_text_color = "white"
            self.slider_flag = False

    def slider_touch_up(self, *args) -> None:
        """Metodo que envia a ESP el valor ultimo del control deslizante de asistencia al momento que el usuario deja de interactuar con el slider"""
        try:
            self.dataTx_queue.put_nowait(json.dumps({self.slider_label: self.slider_value}))
        except asyncio.QueueFull:
            pass

    async def update_manipulation_value(self) -> None:
        """Metodo que actualiza en la interfaz el valor de manipulación al encontrarse el sistema en modo automatico"""
        while True:
            print('in_manipulation')
            try:
                manip = await self.manipulation_queue.get()
                manip = int(manip)
                self.manip_button.text = f'Manipulation: {manip}'  
            except Exception as e:
                print(f'EXCEPTION IN MANIP: {e}')
                await asyncio.sleep(1.0)

    async def update_speed_value(self) -> None:
        """Metodo que actualiza el valor de la velocidad obtenida a traves del GPS del celcular"""
        speed = 0
        while True:
            print("in_speed")
            try:
                speed = await self.velocity_queue.get()
                speed = float(speed)
                print(f"speed-> {speed}")
                await self.dataTx_queue.put(json.dumps({'speed': speed}))
            except Exception as e:
                print(f'Exception in speed:: {e}')
                await asyncio.sleep(1.0)
            if float(speed) > 2 * self.speedmeter.end_value / 3.6:
                pass
            else:
                self.speedmeter.set_value = speed - 25
                self.speedmeter.text = f'{int(speed)} km/h'

    async def update_acceleration_value(self) -> None:
        """Metodo que actualiza el valor de la velocidad obtenida a traves del GPS del celcular"""
        acceleration = 0
        while True:
            print("in_speed")
            try:
                acceleration = await self.acceleration_queue.get()
                acceleration = float(acceleration)
                print(f"acceleration-> {acceleration}")
                await self.dataTx_queue.put(json.dumps({'acceleration_y': acceleration}))
                anguloy = int(math.asin(acceleration / 9.81) * (180.0 / math.pi))
                self.an_button.text = f'Angle : {anguloy}°'
            except Exception as e:
                print(f'Exception in angle:: {e}')
                await asyncio.sleep(1.0)

    async def update_battery_value(self) -> None:
        """Metodo que actualiza la carga restante en la bateria del sistema electronico"""
        max_battery_voltage = 23.7  # V //Voltage gotten when fully charged
        min_battery_voltage = 20.0  # V //Lowest voltage before battery starts getting damaged
        battery_life = 0
        while True:
            print("in_battery")
            try:
                current_battery_life = await self.battery_queue.get()
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

    # Metodos para la desconexión de app del dispositivo ESP y para la acción de cerrar app
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
        self.manip_button.text = f'Manipulation: 0'

        self.per_button_pressed = True
        self.km_button_pressed = False
        self.angle_button_pressed = False

        self.set_angle = 0
        self.slider_label = 'slider'
        self.slider_value = 0
        self.slider_flag = False
        self.test_counter = 0

        self.sp_button.text = f'Set Point: 0'
        self.read_slider_text.text = f'0 km/h'
        self.root.get_screen('main_window').ids.spinner.active = False
        self.root.get_screen('secondary_window').ids.adapt_slider.value = self.root.get_screen('secondary_window').ids.adapt_slider.min
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


async def run_BLE(app: MDApp, dataTx_queue: asyncio.Queue, battery_queue: asyncio.Queue, deviceSelect_queue: asyncio.Queue,
                  angle_queue: asyncio.Queue, manipulation_queue: asyncio.Queue) -> None:
    """Método que inicia la conexión por el protocolo de BLE, asi como la comunicación entre servidor y cliente y el manejo de queues
       para el envio y repcion de datos"""
    
    print('in run_BLE')
    read_char = "00002A3D-0000-1000-8000-00805f9b34fb"
    flag = asyncio.Event()
    connection = Connection(loop=loop,
                            uuid=UUID,
                            address=ADDRESS,
                            read_char=read_char,
                            write_char=read_char,
                            flag=flag,
                            app=app,
                            deviceSelect_queue=deviceSelect_queue)
    disconnect_flag['disconnect'] = False

    try:
        asyncio.ensure_future(connection.manager())
        asyncio.ensure_future(communication_manager(connection=connection,
                                                    write_char=read_char,
                                                    read_char=read_char,
                                                    dataTx_queue=dataTx_queue,
                                                    battery_queue=battery_queue, angle_queue=angle_queue,
                                                    manipulation_queue=manipulation_queue, disconnect_flag=disconnect_flag))
        print(f"fetching connection")
        await connection.flag.wait()
    finally:
        print(f"flag status confirmed!")
        #AccHelper().run(dataTx_queue)

    try:
        app.root.current = 'secondary_window'
        app.root.get_screen('secondary_window').ids.manual_button.border_color = (0.4, 0.898, 0.223, 1) 
        app.root.get_screen('secondary_window').ids.manual_button.border_width = 6 
    except Exception as e:
        print(f'EXCEPTION WHEN CHANGING WINDOW -> {e}')


if __name__ == '__main__':
    async def mainThread():
        """Hilo principal para el lanzamiento de la aplicación"""
        BikeApp = App()
        task_runApp = asyncio.create_task(BikeApp.start())
        (done, pending) = await asyncio.wait({task_runApp}, return_when='FIRST_COMPLETED')

    loop = asyncio.get_event_loop()
    asyncio.run(mainThread())