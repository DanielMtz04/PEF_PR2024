from bleak.backends.p4android.client import BleakClientP4Android
from bleak.backends.p4android.scanner import BleakScannerP4Android
from bleak import BleakScanner, BleakClient
from kivymd.app import MDApp
from datetime import datetime
from typing import Any, Union
import uuid as ui
import json
import asyncio

class Connection:
    
    client: BleakClientP4Android = None

    def __init__(self,
                 loop: asyncio.AbstractEventLoop,
                 app: MDApp,
                 uuid: Union[str, None], address: Union[str, None],
                 read_char: str, write_char: str,
                 deviceSelect_queue: asyncio.Queue,
                 dump_size: int = 256,
                 flag: asyncio.Event = False):
        self.loop = loop
        self.uuid = uuid
        self.address = address
        self.read_char = read_char
        self.p_read_char = ui.UUID(self.read_char)
        self.write_char = write_char
        self.dump_size = dump_size
        self.flag = flag
        self.app = app
        self.deviceSelect_queue = deviceSelect_queue

        self.connected = False
        self.connected_device = None

        self.rx_data = []
        self.rx_timestamps = []
        self.rx_delays = []

    async def on_disconnect(self) -> None:
        """Metodo asincrono que se ejecuta al desconectarde de dispostivo BLE"""
        self.connected = False
        print(f"Disconnected from {self.connected_device.name}!")

    async def manager(self) -> None:
        """Metodo que lleva a cabo a la conexión BLE, comienza la conexión una vez que el cliente exista, o busca primero 
           el dispostivo si no existe en un principio"""
        while True:
            if self.client:
                await self.connect_client()
                await asyncio.sleep(1.0)
            else:
                try:
                    await self.search_device()
                except Exception as e:
                    print(e)
                await asyncio.sleep(3.0)

    async def connect_client(self) -> None:
        """Metodo principal de conexión con dispositivo BLE y configura la conexión para recibir datos de una caracteristica especifica 
           a la cual se suscribe"""
        
        print('in connection client')

        if self.connected:
            return
        
        while True:
            try:
                print('Trying to connect to device...')
                await self.client.connect()
                self.connected = self.client.is_connected
                print(f"Connection status: {self.connected}")

                if self.connected:
                    try:
                        print(self.client.services.characteristics)
                        self.set_connect_flag()  
                        await self.client.start_notify(self.read_char, self.notification_handler)
                    except Exception as e:
                        print(f'EXCEPTION TRYING TO CONNECT {e}')
                    while True:
                        if not self.connected:
                            self.app.root.current = 'main_window'
                            break
                        await asyncio.sleep(10.0)
                else:
                    print(f"Failed to connect to {self.connected_device.name}")

            except Exception as e:
                print(f"IN EXCEPTION CLIENT {e}")
                self.connected = False
                self.app.root.get_screen('main_window').ids.device_dropdown.text = '...'
                self.app.root.get_screen('main_window').ids.spinner.active = False
                self.app.root.get_screen('main_window').ids.device_dropdown.disabled = False
            finally:
                break

    async def search_device(self, uuid: str = None, address: str = None) -> None:
        """Metodo que busca un dispositvo BLE por su UUID o dirección para crear un cliente, si no encuentra inicia el escaneo
           para encontrar dispositivos disponibles"""
            
        print(f"bluetooth LE hardware warming up...{datetime.now()}")
        await asyncio.sleep(3.0)  # Wait for BLE to initialize.
        print(datetime.now())

        if self.uuid and self.address:
            self.connected_device = [uuid, address]
            print('In self.client')
            while True:
                self.client, _ = BleakClientP4Android(address, loop=self.loop)
                print(f"in connection protocol")

                if self.client:
                    print("Breaking away")
                    break
                else:
                    print('Device not found')
                    await asyncio.sleep(1.0)
        else:
            await self.scann_devicesBLE()

    async def scann_devicesBLE(self) -> None:
        """Metodo principal para escanear dispostivos BLE y actualizar el menu desplegable e igualmente espera 
           a que el dispositivo sea seleccionado"""
        
        dropdown_devices = list()
        dropdown_dict = dict()
        device_found = False
        response = -1
        while not device_found:
            devices = await asyncio.create_task(BleakScanner.discover())
            
            for i, device in enumerate(devices):
                if device.name != None:
                    print(f"{i}: {device.name}, {device.address}")
                    self.app.root.get_screen('main_window').ids.spinner.active = False
                    self.app.root.get_screen('main_window').ids.device_dropdown.disabled = False
                    self.app.root.get_screen('main_window').ids.device_dropdown.text = '...'
                    self.app.root.get_screen('main_window').ids.device_dropdown.opacity = 1
                    dropdown_devices.append(str(device.name))
                    dropdown_dict.update({device.name: i})
            self.app.root.get_screen('main_window').ids.device_dropdown.pos_hint = {'center_x': 0.5, 'center_y': 0.6}
            self.app.root.get_screen('main_window').ids.device_dropdown.size = (50, 100)
            self.app.root.get_screen('main_window').ids.device_dropdown.width = self.app.root.width - 200
            self.app.root.get_screen('main_window').ids.device_dropdown.values = dropdown_devices

            device = await self.deviceSelect_queue.get()
            print(f'Device select: {device}')
            response = dropdown_dict[device]

            if devices:
                device_found = True
                self.app.root.get_screen('main_window').ids.device_dropdown.disabled = False

        print(f"Connecting to {devices[response].name}")
        self.connected_device = devices[response]
        try:
            self.client = BleakClient(address_or_ble_device=devices[response].address, loop=self.loop)
        except Exception as e:
            print(f"There was a problem connecting to device... {e}")

    def notification_handler(self, sender: str, data: Any):
        self.rx_data.append(int.from_bytes(data, byteorder="big"))
        self.record_time_info()
        if len(self.rx_data) >= self.dump_size:
            self.clear_lists()
    
    def record_time_info(self) -> None:
        present_time = datetime.now()
        self.rx_timestamps.append(present_time)
        self.last_packet_time = present_time
        self.rx_delays.append((present_time - self.last_packet_time).microseconds)

    def clear_lists(self):
        self.rx_data.clear()
        self.rx_delays.clear()
        self.rx_timestamps.clear()
    
    def set_connect_flag(self):
        self.flag.set()

    def restore(self):
        self.app.root.get_screen('main_window').ids.spinner.active = False
        self.on_disconnect()
    
    #def angle_refresh(self, angle_str) -> None:
        #self.app.root.get_screen('secondary_window').ids.angle_button.text = f'Angle : {angle_str}°'


async def communication_manager(connection: Connection,
                                write_char: str, read_char: str,
                                dataTx_queue: asyncio.Queue, battery_queue: asyncio.Queue, angle_queue: asyncio.Queue,
                                manipulation_queue: asyncio.Queue, disconnect_flag: dict):
    """Metodo que se encarga de manegar la comunicación bidireccional con dispostivo BLE"""
  
    buffer = list()
    while True:
        if disconnect_flag.get('disconnect', True):
            connection.restore()
            await connection.client.disconnect()
            return
        if connection.client and connection.connected:
            try:
                print(f'q_size -> {dataTx_queue.qsize()}')
                if dataTx_queue.qsize() > 1:
                    for i in range(dataTx_queue.qsize()):
                        input_str = await dataTx_queue.get()
                        buffer.append(str(input_str))
                else:
                    buffer.append(str(dataTx_queue.get_nowait()))
                if len(buffer) > 0:
                    input_str = f"{buffer[0]} \n"
                    for i in buffer:
                        bytes_to_send = bytearray(map(ord, str(i)))
                        await connection.client.write_gatt_char(write_char, bytes_to_send, response=True)
                        await asyncio.sleep(0.1)
                    print(f'send_str: {str(buffer)}')
                    buffer.clear()
            except asyncio.QueueEmpty:
                print('EXCEPTION_BLE')
            await asyncio.sleep(0.1)

            # Read meesage sent from ESP
            msg_read = await connection.client.read_gatt_char(read_char)
            print(f"message received -> {msg_read.decode()}")
            msg_json = json.loads(msg_read.decode())  

            # Read battery
            try:
                bat_str = msg_json['battery']
            except Exception as e:
                print(f'EXCEPTION JSON BATTERY: {e}')
                bat_str = None
            if bat_str is not None:
                await battery_queue.put(bat_str)
            # Read angle    
            try:
                angle_str = msg_json['angle']
                print(f'angle_str : {angle_str}')
                #connection.angle_refresh(angle_str)
            except Exception as e:
                print(f'EXCEPTION JSON ANGLE: {e}')
                angle_str = None
            # Read manipulation
            try:
                man_str = msg_json['manipulation']
            except Exception as e:
                print(f'EXCEPTION JSON MAN: {e}')
                man_str = None
            if man_str is not None:
                await manipulation_queue.put(man_str)
                print(f'man_str: {man_str}')
            

        else:
            await asyncio.sleep(2.0)
