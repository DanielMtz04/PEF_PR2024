# PEF-OT23
Repositorio para códigos e información del PEF 
# Android App
## Front-End
Se utiliza Kivy para el diseño del UI. Cuenta con las siguientes features:
+ Botón de conexión a BLE
+ Switch con modo adaptivo para bicicleta
+ Slider para input de usuario en modo manual
+ Velocímetro
+ Display de batería

### Pantalla Principal
<img width="600" alt="MainWindow" src="https://github.com/jesustrevino/PEF-OT23/assets/74314228/e833f3e4-02aa-4e5d-a7f9-c67a65bf803d">

### Pantalla Secundaria
<img width="600" alt="SecondaryWindow" src="https://github.com/jesustrevino/PEF-OT23/assets/74314228/0b49fbc7-a39d-4213-9aeb-9258b02e2234">


## Back-End
Entre los factores más importantes, se tiene el uso de las siguientes features:
+ Bleak - para establecimiento del protocolo BLE entre ESP32 y Android App. [Pueden verse las funciones programadas en BLE.py]
+ Asyncio - se une al uso de kivy para crear corutinas, teniendo así la aplicación y en el background la conexión BLE
