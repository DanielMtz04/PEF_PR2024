// Librerías
#include <ESP32Servo.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <analogWrite.h>

// Constantes y definiciones
#define CW  -1       // Representa la rotación en sentido horario
#define CCW  1       // Representa la rotación en sentido antihorario

// UUIDs para el servicio y la característica BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "00002A3D-0000-1000-8000-00805f9b34fb"

#define PWM_PIN 16
#define BATTERY_PIN 33
#define BATTERY_READ_FLAG 32
#define HALL_SENSOR_U_PIN 25
#define HALL_SENSOR_V_PIN 26
#define HALL_SENSOR_W_PIN 27

//---------------------- Variables para el envio y recepción de datos(ESP(Servidor)-APP(Cliente)) -----------------
DynamicJsonDocument jsonDoc(512);
DynamicJsonDocument jsonreceived(256);

Servo myservo;

// variables para almacenar datos del documento json
String velocity = "0";
float acceleration = 0;

//PWM
int adaptationMode = 0;
int movementFlag = 0;
int sliderValue = 0;

// variables para motion detect
const int rpmThreshold = 200;

//variables modo automatico
int pidFlag = 0;
int automaticFlag = 0;
int velocityPerKm = -1;

//---------------------- Variables del PID -----------------
double sliderReferenceKm = 0;
float Kc = 5;                  //---- Constante proporcional del PID
float Taui = 0.4;              //---- Constante integral del PID
float Taud = 0.5;              //---- Constante derivativa del PID
float T = 1;                   //---- Periodo de muestreo de la variable de proceso
float Mk = 0;                  //---- Manipulación del PID actual
float Mk1 = 0;                 //---- Manipulación del PID del tiempo de muestreo anterior
float E = 0;                   //---- Error actual
float E1 = 0;                  //---- Error anterior
float E2 = 0;                  //---- Error 2 veces atrás del tiempo de muestreo actual

//---------------- Constantes del PID digital ---------------
float BC1 = 0;
float BC2 = 0;
float BC3 = 0;

//---------------------- Variables para el calculo de las revoluciones y uso de sensores HALL -----------------
int directionFlagMotor = 1;
int directionMotor = 1;         // Integer variable to store BLDC rotation direction
int pulseCount;                 // Integer variable to store the pulse count
int rpm;                        // Float variable to store calculated revolutions per minute
float ppm;                      // Float variable to store calculated pulses per minute
float startTime;                // Float variable to store the start time of the current interrupt
float prevTime;                 // Float variable to store the start time of the previous interrupt
float pulseTimeW;               // Float variable to store the elapsed time between interrupts for hall sensor W
float pulseTimeU;               // Float variable to store the elapsed time between interrupts for hall sensor U
float pulseTimeV;               // Float variable to store the elapsed time between interrupts for hall sensor V
float AvPulseTime;              // Float variable to store the average elapsed time between all interrupts
bool HSU_Val = digitalRead(HALL_SENSOR_U_PIN);   // Set the U sensor value as boolean and read initial state
bool HSV_Val = digitalRead(HALL_SENSOR_V_PIN);   // Set the V sensor value as boolean and read initial state
bool HSW_Val = digitalRead(HALL_SENSOR_W_PIN);   // Set the W sensor value as boolean and read initial state
bool HSU_Val_old;
bool HSV_Val_old;
bool HSW_Val_old;

//---------------------- Variables para la conexión Bluetooht-----------------
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
std::string receivedValue = "";

//---------------------- Variables para la lectura de la carga de la bateria-----------------
unsigned long previousMillis = 0;        
const long intervalRead = 300000;  
int readBattery = 0;  
float battery = 0;
float batteryVolts = 0;
float lecture = 0;    
int battery_flag = 1;



//----------------------Clase para el Control de eventos de conexión y desconexión BLE-----------------
class MyServerCallbacks: public BLEServerCallbacks {
 
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      myservo.write(0);
      ESP.restart();
    }
   
};

//----------------------Función para el envio de señal PWM-----------------
void set_pwm() {
 
  if (movementFlag == 1 || adaptationMode == 0){
   myservo.write(sliderValue);
  }
  else {
   myservo.write(0);
  }
 
}

//----------------------Clase para el Control de recepción de datos----------------
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
     
      receivedValue = pCharacteristic->getValue();
      deserializeJson(jsonreceived,receivedValue.c_str());
     
      String sliderPercentageString = jsonreceived["slider_per"];
      String sliderKmString = jsonreceived["slider_km"];
      String velocityValid = jsonreceived["speed"];//APPPPPP
      String adaptationModeString = jsonreceived["adaptationMode"];
      String accelerationString = jsonreceived["acceleration_y"];
 
      Serial.print("velocity: ");
      Serial.println(velocityValid);
 
      Serial.print("acceleration: ");
      Serial.println(accelerationString);

      if (sliderPercentageString != "null") {
        automaticFlag = 0;
        sliderValue = sliderPercentageString.toInt();
        set_pwm();
      }

      if (sliderKmString != "null") {
        sliderReferenceKm = sliderKmString.toDouble();
        automaticFlag = 1;
       }
       

      if (velocityValid != "null") {
          velocity = velocityValid;
        }

      if (velocity != "null" && automaticFlag == 1) {
        velocityPerKm = velocity.toInt();
        pidFlag = 1;
      }

      if (adaptationModeString != "null") {
        adaptationMode=adaptationModeString.toInt();
      }
     
      acceleration = accelerationString.toFloat();
       
       
    }
};





void setup() {


  //----------------------Configuración inicial-----------------
  pinMode(BATTERY_READ_FLAG ,OUTPUT);
  digitalWrite(BATTERY_READ_FLAG, LOW);
  myservo.attach(PWM_PIN);
  Serial.begin(115200);

  
  
 
  //----------------------Configuración de los Pines de los sensores Hall-----------------
  pinMode(HALL_SENSOR_U_PIN, INPUT);
  pinMode(HALL_SENSOR_V_PIN, INPUT);
  pinMode(HALL_SENSOR_W_PIN, INPUT);

  //----------------------Calculo de parametros PID-----------------
  BC1 = Kc * (1 + (T / Taui) + (Taud / T));
  BC2 = Kc * (- 1 - (2 * Taud / T));
  BC3 = Kc * Taud / T;

  //----------------------Proceso de inicialización BLE----------------------
 
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
  pAdvertising->setMinPreferred(0x0);  
  BLEDevice::startAdvertising();

  //----------------------Creación de la tarea para el modo Motion Detect----------------------
  xTaskCreatePinnedToCore(
    TaskToMotionDetect,   // Función de la tarea
    "TaskToMotionDetect", // Nombre de la tarea
    10000,          // Tamaño del stack de la tarea
    NULL,          // Parámetro de la tarea
    1,             // Prioridad de la tarea
    NULL,          // Handle de la tarea
    1              // Núcleo donde se ejecuta la tarea
  );
 
}


//--------------------------------Inicio del main loop---------------------------
void loop() {
 
  //----------------------PID Digital-----------------
  if (velocityPerKm >= 0 && automaticFlag==1 && pidFlag == 1) {
   
    if (velocityPerKm >= sliderReferenceKm) {
      Mk = 0;
      E = 0;
    }
    else {
      E = sliderReferenceKm-velocityPerKm;
      Mk = Mk1 + BC1 * E + BC2 * E1 + BC3 * E2;
    }
   
    if (Mk > 180) {
      Mk = 180;
    }
    else if (Mk < 0) {
      Mk = 0;
    }
   
    sliderValue = Mk;
    set_pwm();
    Mk1 = Mk;
    E2 = E1;
    E1 = E;
  }
 
  //----------------------Proceso de desconexión-----------------
  if (!deviceConnected && oldDeviceConnected) {
      delay(500);
      pServer->startAdvertising(); // restart advertising
      oldDeviceConnected = deviceConnected;
  }

  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }

  //----------------------Lectura de la bateria-----------------
  digitalWrite(BATTERY_READ_FLAG, LOW);
  unsigned long currentMillis = millis();
 
  if (currentMillis - previousMillis >= intervalRead) {
   
    previousMillis = currentMillis;      
    digitalWrite(BATTERY_READ_FLAG, HIGH);
   
    delay(100);
   
    battery = analogRead(BATTERY_PIN);
    batteryVolts = ((battery / 4095.0) * 3.3);
    lecture = ((batteryVolts - 2.45) / (3.3 - 2.45)) * 100;
   
    if (lecture < 0) {
      lecture = 0;
    }
    else if (lecture > 100) {
      lecture = 100;
    }
   
    Serial.print("baterry: ");
    Serial.println(batteryVolts);

    Serial.print("lecture: ");
    Serial.println(lecture);
             
  }

 
  //----------------------Calculo del Angulo-----------------
  int anguloy = asin(acceleration / 9.81) * (180.0 / PI);
  if (anguloy == 2147483647){
    anguloy = 90;
  }
  else{
    int anguloy = asin(acceleration / 9.81) * (180.0 / PI);
  }
 
  //----------------------Envio de datos por BLE-----------------
  if (deviceConnected) {
    if(battery_flag == 1){
          digitalWrite(BATTERY_READ_FLAG, HIGH);
          delay(100);
          battery = analogRead(BATTERY_PIN);
          batteryVolts = ((battery / 4095.0) * 3.3);
          lecture = ((batteryVolts - 2.45) / (3.3 - 2.45)) * 100;
          digitalWrite(BATTERY_READ_FLAG, LOW);
          battery_flag = 2;
     }
    // Convierte el valor de la bateria  a un String
    String batteryStr = String(lecture);
    // Convierte el valor de la manipulación a entero
    int outputStr=int(Mk);

    //Se crea el documento JSON para enviar los datos
    
    jsonDoc["battery"]=batteryStr;
    jsonDoc["angle"]=anguloy;
    jsonDoc["manipulation"]=outputStr;
   
    // Convertir el documento JSON en una cadena de caracteres
    String jsonString;
    serializeJson(jsonDoc, jsonString);
     
    // Envio de la cadena por BLE usando la característica
    pCharacteristic->setValue(jsonString.c_str());
    delay(100);
  }
 
}
//--------------------------------Fin del main loop---------------------------

//----------------------------- Funciones para el calculo de rpm a partir de los sensores HALL -----------------------------------
void HallSensorW()
{
  startTime = micros();                                        // Set startTime to current microcontroller elapsed time value
  HSW_Val = digitalRead(HALL_SENSOR_W_PIN);                                   // Read the current W hall sensor value
  HSW_Val = !HSW_Val;
  HSV_Val = digitalRead(HALL_SENSOR_V_PIN);                                   // Read the current V (or U) hall sensor value
  directionMotor = (HSW_Val == HSV_Val) ? CW : CCW;            // Determine rotation direction (ternary if statement)
  pulseTimeW = startTime - prevTime;                           // Calculate the current time between pulses
  AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);    // Calculate the average time time between pulses
  ppm = (1000000 / AvPulseTime) * 60;                          // Calculate the pulses per min (1000 micros in 1 second)
  rpm = ppm / 90;                                              // Calculate revs per minute based on 90 pulses per rev
  prevTime = startTime;                                        // Remember the start time for the next interrupt
}

void HallSensorV()
{
  startTime = micros();
  HSV_Val = digitalRead(HALL_SENSOR_V_PIN);
  HSU_Val = digitalRead(HALL_SENSOR_U_PIN);          
  directionMotor = (HSV_Val == HSU_Val) ? CW : CCW;
  pulseTimeV = startTime - prevTime;        
  AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);  
  ppm = (1000000 / AvPulseTime) * 60;        
  rpm = ppm / 90;
  prevTime = startTime;
}

void HallSensorU()
{
  startTime = micros();
  HSU_Val = digitalRead(HALL_SENSOR_U_PIN);
  HSW_Val = digitalRead(HALL_SENSOR_W_PIN);          
  HSW_Val = !HSW_Val;  
  directionMotor = (HSU_Val == HSW_Val) ? CW : CCW;
  pulseTimeU = startTime - prevTime;        
  AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV)/3);  
  ppm = (1000000 / AvPulseTime) * 60;        
  rpm = ppm / 90;
  prevTime = startTime;
}


//----------------------Tarea para la implementación del Motion Detect-----------------
void TaskToMotionDetect(void *pvParameters) {
  (void) pvParameters;
 
  while (1) {
    if ((micros() - prevTime) > 100000) rpm = 0;

    HSU_Val_old = HSU_Val;
    HSV_Val_old = HSV_Val;
    HSW_Val_old = HSW_Val;
    HSU_Val = digitalRead(HALL_SENSOR_U_PIN);  
    HSV_Val = digitalRead(HALL_SENSOR_V_PIN);    
    HSW_Val = digitalRead(HALL_SENSOR_W_PIN);  
    HSW_Val = !HSW_Val;
 
    if (HSW_Val_old != HSW_Val) {
        HallSensorW();
    }  
    else if (HSV_Val_old != HSV_Val) {
        HallSensorV();
    }
    else if (HSU_Val_old != HSU_Val) {
        HallSensorU();
    }
   
    if (rpm > rpmThreshold && directionMotor == 1 && directionFlagMotor==1) {
       movementFlag = 1;
       directionFlagMotor = 0;
       set_pwm();
      }
      else if (rpm > rpmThreshold &&  directionFlagMotor==0) {
       movementFlag = 1;
       set_pwm();
      }
      else{
        movementFlag = 0;
        directionFlagMotor = 1;
        set_pwm();
    }
   
    //Serial.println(rpm);
    //Serial.println(directionMotor);
    //Serial.print(HSU_Val); Serial.print(HSV_Val); Serial.println(HSW_Val);
    //Serial.println(ppm);
  }
}
