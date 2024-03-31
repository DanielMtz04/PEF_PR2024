#include <ESP32Servo.h>
#include <analogWrite.h>
#include <ESP32Tone.h>
#include <ESP32PWM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>

// Constantes y definiciones
#define CW  -1       // Representa la rotación en sentido horario
#define CCW  1       // Representa la rotación en sentido antihorario

// UUIDs para el servicio y la característica BLE
#define SERVICE_UUID         "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID  "00002A3D-0000-1000-8000-00805f9b34fb"

#define SERVO_PIN 16
#define BATTERY_PIN 35
#define HALL_SENSOR_U_PIN 25
#define HALL_SENSOR_V_PIN 26
#define HALL_SENSOR_W_PIN 27

//---------------------- Variables para el envio y recepción de datos(ESP-APP) -----------------
DynamicJsonDocument jsonToSend(512);
DynamicJsonDocument jsonReceived(256);

Servo myservo; 
float acceleration = 0;
int adaptationMode = 0;
int velocityKmPerHour = -1;
double sliderReferenceSpeed = 0;
int sliderKmFlag = 0;
int movementFlag = 0;
int RPM_thresh = 70; 
int slider_value = 0;
int pidFlag = 0;

bool deviceConnected = false;
bool oldDeviceConnected = false;
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
std::string receivedValue = ""; 

//---------------------- Variables del PID -----------------
float Kc = 5;          //---- Constante proporcional del PID
float Taui = 0.4;      //---- Constante integral del PID
float Taud = 0.5;      //---- Constante derivativa del PID
float T = 1;           //---- Periodo de muestreo de la variable de proceso
float Mk = 0;          //---- Manipulación del PID actual
float Mk1 = 0;         //---- Manipulación del PID del tiempo de muestreo anterior 
float E = 0;           //---- Error actual
float E1 = 0;          //---- Error anterior
float E2 = 0;          //---- Eerror 2 veces atrás del tiempo de muestreo actual
float BC1 = 0;
float BC2 = 0;
float BC3 = 0;

//---------------------- Variables para el calculo de las revoluciones y uso de sensores HALL -----------------
int flag_dir = 1;
int direct = 1;         // Integer variable to store BLDC rotation direction
int pulseCount;         // Integer variable to store the pulse count
bool HSU_Val_old;
bool HSV_Val_old;
bool HSW_Val_old;
int RPM;                // Float variable to store calculated revolutions per minute
float startTime;        // Float variable to store the start time of the current interrupt 
float prevTime;         // Float variable to store the start time of the previous interrupt 
float pulseTimeW;       // Float variable to store the elapsed time between interrupts for hall sensor W 
float pulseTimeU;       // Float variable to store the elapsed time between interrupts for hall sensor U 
float pulseTimeV;       // Float variable to store the elapsed time between interrupts for hall sensor V 
float AvPulseTime;      // Float variable to store the average elapsed time between all interrupts 
float PPM;              // Float variable to store calculated pulses per minute

void HallSensorW()
{
  startTime = millis();                                        // Set startTime to current microcontroller elapsed time value
  HSW_Val = digitalRead(HALL_SENSOR_W_PIN);                    // Read the current W hall sensor value
  HSW_Val = !HSW_Val; 
  HSV_Val = digitalRead(HALL_SENSOR_V_PIN);                    // Read the current V (or U) hall sensor value 
  direct = (HSW_Val == HSV_Val) ? CW : CCW;                    // Determine rotation direction (ternary if statement)
  pulseCount = pulseCount + (1 * direct);                      // Add 1 to the pulse count
  pulseTimeW = startTime - prevTime;                           // Calculate the current time between pulses
  AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV) / 3);    // Calculate the average time time between pulses
  PPM = (1000 / AvPulseTime) * 60;                             // Calculate the pulses per min (1000 millis in 1 second)
  RPM = PPM / 90;                                              // Calculate revs per minute based on 90 pulses per rev
  prevTime = startTime;                                        // Remember the start time for the next interrupt
}

void HallSensorV()
{
  startTime = millis();
  HSV_Val = digitalRead(HALL_SENSOR_V_PIN);
  HSU_Val = digitalRead(HALL_SENSOR_U_PIN);          
  direct = (HSV_Val == HSU_Val) ? CW : CCW;
  pulseCount = pulseCount + (1 * direct);
  pulseTimeV = startTime - prevTime;        
  AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV) / 3);   
  PPM = (1000 / AvPulseTime) * 60;          
  RPM = PPM / 90;
  prevTime = startTime;
}

void HallSensorU()
{
  startTime = millis();
  HSU_Val = digitalRead(HALL_SENSOR_U_PIN);
  HSW_Val = digitalRead(HALL_SENSOR_W_PIN);           
  HSW_Val = !HSW_Val;   
  direct = (HSU_Val == HSW_Val) ? CW : CCW;
  pulseCount = pulseCount + (1 * direct);
  pulseTimeU = startTime - prevTime;        
  AvPulseTime = ((pulseTimeW + pulseTimeU + pulseTimeV) / 3);   
  PPM = (1000 / AvPulseTime) * 60;          
  RPM = PPM / 90;
  prevTime = startTime;
}

// Funcion para enviar señal PWM al driver
void set_pwm() {
  if (movementFlag == 1 || adaptationMode == 1){
    myservo.write(slider_value);
  }
  else {
    myservo.write(0);
  }
  }

// Funciones para activar o desactivar la bandera de la conexión de dispositvo
class MyServerCallbacks: public BLEServerCallbacks {
  
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      myservo.write(0);
    }
    
};

// Funcion para la recepción y el almacenamiento de datos por BLE
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      
      receivedValue = pCharacteristic->getValue();
      deserializeJson(jsonReceived,receivedValue.c_str());
      
      String modeAdapt=jsonReceived["adapt"];
      String sliderPercentage=jsonReceived["sliderPercentage"];
      String sliderKm=jsonReceived["slider_km"];
      String velocity=jsonReceived["speed"];
      String acc=jsonReceived["acc_y"];
      
      acceleration=acc.toFloat();
      
      if (modeAdapt != "null") {
        adaptationMode=modeAdapt.toInt();
      }

      if (sliderPercentage != "null") {
        sliderKmFlag = 0;
        slider_value = sliderPercentage.toInt();
        set_pwm();  
      }

      if (sliderKm != "null") {
        sliderKmFlag = 1;
        sliderReferenceSpeed=sliderKm.toDouble(); 
      }
      
      if(velocity != "null" && sliderKmFlag == 1) {
        velocityKmPerHour=velocity.toInt();
        // set_pwm();
        pidFlag = 1;
      }
      else if (velocity == "null") {
        pidFlag = 0;
      }
      
    }
};



void setup() {

  // Configuración inicial
  pinMode(LED_PIN, OUTPUT);
  pinMode(BATTERY_PIN, INPUT);
  adcAttachPin(BATTERY_PIN);
  myservo.attach(SEVO_PIN);
  Serial.begin(115200);
  
  // Configuración de los sensores Hall
  pinMode(HALL_SENSOR_U_PIN, INPUT);
  pinMode(HALL_SENSOR_V_PIN, INPUT);
  pinMode(HALL_SENSOR_W_PIN, INPUT);

  //Calculo de parametros PID
  BC1 = Kc*(1+(T/Taui)+(Taud/T));
  BC2 = Kc*(-1-(2*Taud/T));
  BC3 = Kc*Taud/T;

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
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  // Configuracion Tarea en FreeRTOS
  xTaskCreatePinnedToCore(
    TaskToMotionDetect,   // Función de la tarea
    "TaskToMotionDetect", // Nombre de la tarea
    10000,                // Tamaño del stack de la tarea
    NULL,          
    1,                    // Prioridad de la tarea
    NULL,          
    1                     // Núcleo donde se ejecuta la tarea
  );
  
}// FIN Void set up

void loop() {
  
  // Implementación de PID Digital
  if(velocityKmPerHour >= 0 && sliderKmFlag==1 && pidFlag == 1){
    
    if (velocityKmPerHour >= sliderReferenceSpeed) {
      Mk = 0;
      E = 0;
    }
    else {
      E = sliderReferenceSpeed - velocityKmPerHour;
      Mk = Mk1 + BC1 * E + BC2 * E1 + BC3 * E2;
    }
    
    if (Mk > 180) {
      Mk = 180;
      }
    else if (Mk < 0){
      Mk = 0;
    }
    
    slider_value = Mk;
    set_pwm();
    Mk1 = Mk;
    E2 = E1;
    E1 = E;
    
  }
     
  // Restart adversiting when device is not connected
  if (!deviceConnected && oldDeviceConnected) {
      delay(500);                            
      pServer->startAdvertising();            
      oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }

  // Lectura de la batería 
  float battery = analogRead(BATTERY_PIN); 
  float batteryVolts = (battery / 4095.0) * 3.3;
  int lecture = ((batteryVolts - 2.45) / (3.15 - 2.45)) * 100;
  
  if (lecture < 0) {
    lecture = 0;
  } 
  else if (lecture > 100) {
    lecture = 100;
  }

  // Calculo del angulo a partir de la aceleración
  int angulo = asin(acceleration / 9.81) * (180.0 / PI);
  if (angulo == 2147483647){
    angulo = 90;
  }
  else{
    int angulo = asin(acceleration / 9.81) * (180.0 / PI);
  }

  // Envio de datos por BLE
  if (deviceConnected) {

    String batteryStr=String(lecture);
    int outputStr=int(Mk);

    //Se crea el objeto en JSON para enviarlo 
    jsonToSend["battery"]=batteryStr;
    jsonToSend["angle"]=angulo;
    jsonToSend["manipulation"]=outputStr;
  
    String jsonString;
    serializeJson(jsonToSend, jsonString);
    
    // Enviar la cadena por BLE usando la característica
    pCharacteristic->setValue(jsonString.c_str());
    delay(100);
  }

  
} // FIN Void loop


// Función para la tarea de Motion Detect
void TaskToMotionDetect(void *pvParameters) {
  (void) pvParameters;
  while (1) {
    
    if ((millis() - prevTime) > 100) RPM = 0; 

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
  
    if (RPM > RPM_thresh && direct == 1 && flag_dir == 1) {
      movementFlag = 1;
      flag_dir = 0;
      set_pwm();
    } 
    
    else if (RPM > RPM_thresh && flag_dir==0) {
      movementFlag = 1;
      set_pwm();
    } 
    else {
      movementFlag = 0;
      flag_dir = 1;
      set_pwm();  
    }
  
    //Serial.println(RPM); 
    Serial.println(direct);
    //Serial.print(HSU_Val); Serial.print(HSV_Val); Serial.println(HSW_Val);
    //Serial.println(PPM);
  }
  
}// FIN Task
