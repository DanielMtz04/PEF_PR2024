#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <PIDController.h>

#include <EEPROM.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

Servo myservo; 
#define EEPROM_SIZE 512

#define WINDOW_SIZE 5
#define WINDOW_SIZE_DC 3

PIDController pid;

String slider_per="";
String slider_km="";
String angulo="";
String velo="";
float accel=0;
int adapt=0;
int eepromaddress=0;
int velo_km=-1;
int READINGS[WINDOW_SIZE];
int READINGS_DC[WINDOW_SIZE_DC];
int INDEX=0;
int VALUE=0;
int SUM=0;
int AVERAGED=0;
int INDEX_DC=0;
int VALUE_DC=0;
int SUM_DC=0;
int AVERAGED_DC=0;
int mov_flag = 0;
int slider_value = 0;
int slider_km_flag = 0;
int j = 0; //variable para el envio de generador DC
const int move_thresh = 1; //threshold para saber si la bicicleta está en movimiento
int pid_flag = 0;

//A partir de aquí se definiran las variables del PID 
double slider_ref=0;
float Kc=5; //---- Constante proporcional del PID
float Taui=0.4; //----- Constante integral del PID
float Taud=0.5; //----- Constante derivativa del PID
float T=1;  //------ Periodo de muestreo de la variable de proceso
float Mk=0; //----Manipulación del PID actual
float Mk1=0; //----- Manipulación del PID del tiempo de muestreo anterior 
float E=0; //----- Error actual
float E1=0;  //----- Error anterior
float E2=0; //------- Eerror 2 veces atrás del tiempo de muestreo actual
float Ref_vel_km=5; //-------- Referencia de velocidad (km/hr)
float Vel_km=0; //----- Variable de proceso (velocidad) 

//---------------- Constantes del PID digital ---------------
float BC1=0;
float BC2=0;
float BC3=0;
//----------------

//Leemos el valor en nuestra EEPROM 
  int angulooffset=EEPROM.read(eepromaddress);

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
std::string receivedValue = ""; // Declarar receivedValue aquí
bool dataSent = false; // Bandera para controlar si los datos se enviaron

//Se crea la variable para generar el documento JSON, se realizara dinámico para que se vaya actualizando 
DynamicJsonDocument jsonDoc(512);
//Se crea la variable JSON para recibir el valor desde la app 
DynamicJsonDocument jsonreceived(256);

// UUIDs para el servicio y la característica BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "00002A3D-0000-1000-8000-00805f9b34fb"

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void set_pwm() {
  Serial.print("Entraste a la función PWM: ");
  Serial.println(slider_value*mov_flag);
  if (mov_flag==1 || adapt==0){
  myservo.write(slider_value);
  delay(500);}
  else {
    myservo.write(0);
    delay(100);
  }
  }

void set_angle(int address, int angle) {
  Serial.print("Entraste a la función SET_angle: ");
  Serial.println(angle);
  EEPROM.write(address,angle);
  EEPROM.commit();
  angulooffset = angle;
}

// Función para leer datos de la EEPROM
byte readEEPROM(int address) {
  return EEPROM.read(address);}
  
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      receivedValue = pCharacteristic->getValue();
      Serial.print("Valor BLE recibido: ");
      Serial.println(receivedValue.c_str());
      deserializeJson(jsonreceived,receivedValue.c_str());
      String angulo=jsonreceived["set_angle"];
      String slider_per=jsonreceived["slider_per"];
      String slider_km=jsonreceived["slider_km"];
      String velo=jsonreceived["speed"];
      String apt=jsonreceived["adapt"];
      String acc=jsonreceived["acc_y"];
      Serial.println(acc.c_str());
      Serial.println(slider_per.c_str());
      Serial.println(angulo.c_str());
      Serial.println(slider_km.c_str());
      Serial.println(velo.c_str());
      accel=acc.toFloat();
      if(apt != "null"){
        adapt=apt.toInt();
      }
      if(angulo != "null"){
        set_angle(eepromaddress, angulo.toInt());
        }
      if(slider_per != "null") {
        Serial.println("in pwm");
        slider_km_flag = 0;
        Serial.println(slider_per);
        slider_value = slider_per.toInt();
        Serial.println(slider_per.toInt());
        set_pwm();
        }
       if(slider_km != "null") {
        Serial.println("in pid");
        slider_km_flag = 1;
        slider_ref=slider_km.toDouble(); 
       }
       if(velo != "null" && slider_km_flag == 1) {
        velo_km=velo.toInt();
        // set_pwm();
        pid_flag = 1;
       }
       else if (velo == "null") {pid_flag = 0;}
    }
};



//Se crean las variables para identificar los puertos de entrada  
const int puertobattery = 35;
const int puerto_dc = 36;

void setup() {
  pinMode(puertobattery,INPUT);
  adcAttachPin(puertobattery);
  EEPROM.begin(EEPROM_SIZE);
  myservo.attach(16);
  Serial.begin(115200);

  //Calculo de parametros PID
  BC1=Kc*(1+(T/Taui)+(Taud/T));
  BC2=Kc*(-1-(2*Taud/T));
  BC3=Kc*Taud/T;

  //Leemos el valor en nuestra EEPROM 
  int angulooffset=EEPROM.read(eepromaddress);

  // Se crea el dispositivo BLE
  BLEDevice::init("ESP32");

  // Se inicializa y crea el servidor BLE
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Se crea el servicio BLE
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Se crea la característica BLE
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  
  // Creación de un Descriptor BLE
  pCharacteristic->addDescriptor(new BLE2902());

  // Asignamos la función callback para escritura
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Se inicia el servicio
  pService->start();

  // Se inicia el proceso de publicidad 
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Esperando una conexión con cliente para notificar...");
}

void loop() {
  //PID DIGITAL
    if(velo_km >= 0 && slider_km_flag==1 && pid_flag == 1)
    {
      Serial.println("en digital");
      //CHANGED LINES 234<>243
      if (velo_km >= slider_ref){
        Mk = 0;
        E = 0;
      }
      else {
        E = slider_ref-velo_km;
        Serial.print("E: ");
        Serial.println(E);
        Mk=Mk1+BC1*E+BC2*E1+BC3*E2;
        }
      Serial.print("MK: ");
      Serial.println(Mk);
      if(Mk>180){
        Mk=180;
        }
      else if (Mk < 0){
        Mk=0;
      }
      
      slider_value = Mk;
      set_pwm();
      Serial.print("PID Manipulation:: ");
      Serial.println(Mk);
      Mk1=Mk;
      E2=E1;
      E1=E;
      }
    
    SUM=SUM-READINGS[INDEX];
    SUM_DC=SUM_DC-READINGS_DC[INDEX_DC];
     
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("inicia publicidad");
        oldDeviceConnected = deviceConnected;
    }

    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }

    // Lectura de generador dc para saber si la bicicleta se está moviendo
    int mov = analogRead(puerto_dc);
    Serial.print("mov: "); 
    Serial.println(mov);
    delay(50);

// Lectura de la batería 

  float battery=analogRead(puertobattery); //lectura analógica del puerto 25 para guardarlo en una variable 
  delay(50);
  float batteryvolts=(battery/4095.0)*3.3;
  int lecture = ((batteryvolts-2.45)/(3.15-2.45)) *100;
  if (lecture < 0) {lecture = 0;} else if (lecture > 100) {lecture = 100;}

  Serial.print("Valor Batería: ");
  Serial.println(lecture);
  

// Se convierten los valores de voltaje en ángulos (°) usando la función atan
  //float angulox=atan2(x_g,1.0)* (180.0 / PI);
  float anguloy=asin(accel/9.81)* (180.0 / PI);
  

  //MAF
  READINGS[INDEX]=anguloy;
  SUM=SUM+anguloy;
  INDEX=(INDEX+1)%WINDOW_SIZE;
  AVERAGED = SUM/WINDOW_SIZE;
  AVERAGED = AVERAGED-angulooffset;
  Serial.print("Ángulo leído en Y: ");
  Serial.println(AVERAGED);
  

  // Lectura de generador dc para saber si la bicicleta se está moviendo
  // mov = (mov*3.3)/4096; //Vdc 10-bits umbral 10-
  //Cuando se active por primera vez, poder mandar el pwm correcto
  READINGS_DC[INDEX_DC]=mov;
  SUM_DC=SUM_DC+mov;
  INDEX_DC=(INDEX_DC+1)%WINDOW_SIZE_DC;
  AVERAGED_DC=SUM_DC/WINDOW_SIZE_DC;
  Serial.print("AVERAGED DC: ");
  Serial.println(AVERAGED_DC);
  Serial.print("Valor generador dc: ");
  Serial.println(mov);
  Serial.print("j = ");
  Serial.println(j);
  if ((AVERAGED_DC > move_thresh) && (j <= 0)) {
    Serial.println("moving");
    mov_flag = 1;
    set_pwm();
    j = 1;
  } else if (AVERAGED_DC < move_thresh){
    mov_flag = 0;
    // slider_value = 0;
    set_pwm();
    j = 0;
  }
 
  
// Imprime los valores de aceleración en el puerto serial
  Serial.print(" grados\t Ángulo en el eje Y: ");
  Serial.print(anguloy);
  Serial.println(" grados");

    // Enviar los valores por BLE
  if (deviceConnected) {
    // Convierte la batería a una cadena
    String batteryStr=String(lecture);
    //Convertir el ángulo y a cadena 
    float anyStr=float(AVERAGED);
    int outputStr=int(Mk);

    //Se crea el objeto en JSON para enviarlo 
    jsonDoc["battery"]=batteryStr;
    jsonDoc["angle"]=anyStr;
    jsonDoc["manipulation"]=outputStr;
    //Serial.println("String: ");
    //Serial.println(outputStr);

    // Convertir el objeto JSON en una cadena
     String jsonString;
     serializeJson(jsonDoc, jsonString);
     
    
    // Enviar la cadena por BLE usando la característica
    pCharacteristic->setValue(jsonString.c_str());
    delay(100);
  }
}
