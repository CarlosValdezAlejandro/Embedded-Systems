#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

//**************************************
//*********** DEFINICIONES *************
//**************************************
 
#define SENSIBILIDAD 3800      //Sensibilidad de la vibración debería ser mayor a 200

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 5       // GIO05 conectado al pin INT del MPU6050
#define M_PI 3.14  //159265358979323846
// GPIO23 conectado a switch Normalmente Cerrado y a tierra.
// Al presionar el boton, se desconecta el gio23 de tierra
// y activa el modo calibración, para setear todos los valores del 
// acelerometro giroscopo a cero.
// Para que suceda se debe presionar el boton de reset e inmediatamente
// despues el boton de "Calibracion" por 5 segundos y esperar 5s a 10s a que calibre.
#define PIN_CALIBRACION 23    


//**************************************
//*********** MQTT CONFIG **************
//**************************************
const char *mqtt_server = "ioticos.org";
const int   mqtt_port = 1883;
const char *mqtt_user = "";  //Aquí va el usuario mqtt del Broker ioticos
const char *mqtt_pass = "";	//Aquí va la contraseña mqtt del Broker ioticos
const char *root_topic_subscribe = "/input";  //el topico + /input
const char *root_topic_publish = "/output";
const char *root_topic_publish_gps = "/output/gps";
const char *root_topic_publish_giroscopio = "/output/giroscopio";
const char *root_topic_publish_vibracion = "/output/vibracion";


//**************************************
//*********** WIFICONFIG ***************
//**************************************
const char* ssid = "";
const char* password =  "";


//**************************************
//*********** GLOBALES   ***************
//**************************************
WiFiClient espClient;
PubSubClient client(espClient);
MPU6050 mpu;

// Tareas
TaskHandle_t Task1, Task2;


// Buffer para JSON
char buffer[720];
char buffer_Hz[880];
char buffer_angulo[370];


//  --------------   Variables propias para el funcionamiento del MPU6050  ----------------------//
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer original 64

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}

bool vibra, en_10s;

unsigned long previousMillis = 0;
const long interval = 1000;

unsigned int cont;
static volatile unsigned int cont_copia;

// ++++ Frecuencia ++++ //
int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
int16_t AcX_anterior, AcY_anterior, AcZ_anterior;
//  ----------------------------------------------------------------------------------------------//

//************************
//** F U N C I O N E S ***
//************************
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void setup_wifi();

void json_to_mqtt_3();  //nuevo JSON, nuevo requerimiento
void json_to_mqtt_3_Hz();
void json_to_mqtt_3_angulos();

void envio10s();        //envío de JSON cada 10s
void envio10ms();       //envío JSON cada 10ms
void envioConFrecuencia(); //envío de JSON con frecuencia

void obtenerAngulos();  //funcion para obtener ángulos 
void iniciarAcelerometro();

// ++++ Frecuencia ++++ //
void obtenerRAW();
void valoresRAW_gyro();
void vibracion();
void contador();

//************************
//**   Nuevas Tareas    **
//************************

void loop1(void *parameter){
  for(;;){
    vibracion();
    if(vibra==true){
    en_10s=false;
    contador();
    obtenerAngulos();
    }
    en_10s=true;
  }
  //vTaskDelay(10);
}

void loop2(void *parameter){
  for(;;){
      TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
      TIMERG0.wdt_feed=1;
      TIMERG0.wdt_wprotect=0;
      if(vibra==true && cont!=0) {
      envio10ms();
  }
    }
    //vTaskDelay(10);  
}
  
//++++  SETUP   ++++

void setup() {
  TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
  TIMERG0.wdt_feed=1;
  TIMERG0.wdt_wprotect=0;

  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  //client.setCallback(callback);

  pinMode(PIN_CALIBRACION, INPUT_PULLUP);
  iniciarAcelerometro();

  en_10s=true;
  vibra=false;

  delay(3000);
  // ------ TAREA 1 ------- //
  xTaskCreatePinnedToCore(
    loop1,
    "Task_1",
    3000,
    NULL,
    2,
    &Task1,
    0
  );
  // ------ TAREA 2 ------- //
  xTaskCreatePinnedToCore(
    loop2,
    "Task_2",
    3000,
    NULL,
    1,
    &Task2,
    0
  );
}

void loop() {
    
  if (!client.connected()) {
		reconnect();
	}

  if (client.connected()){
  if(en_10s==true) envio10s();
  }

  client.loop();
  vTaskDelay(10);
}





//*****************************
//***    CONEXION WIFI      ***
//*****************************
void setup_wifi(){
	delay(10);
	// Nos conectamos a nuestra red Wifi
	Serial.println();
	Serial.print("Conectando a ssid: ");
	Serial.println(ssid);

	WiFi.begin(ssid, password);

	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
	}

	Serial.println("");
	Serial.println("Conectado a red WiFi!");
	Serial.println("Dirección IP: ");
	Serial.println(WiFi.localIP());
}



//*****************************
//***    CONEXION MQTT      ***
//*****************************

void reconnect() {

	while (!client.connected()) {
		Serial.print("Intentando conexión Mqtt...");
		// Creamos un cliente ID
		String clientId = "IOTICOS_H_W_";
		clientId += String(random(0xffff), HEX);
		// Intentamos conectar
		if (client.connect(clientId.c_str(),mqtt_user,mqtt_pass)) {
			Serial.println("Conectado!");
			// Nos suscribimos
			if(client.subscribe(root_topic_subscribe)){
        Serial.println("Suscripcion ok");
      }else{
        Serial.println("fallo Suscripciión");
      }
		} else {
			Serial.print("falló :( con error -> ");
			Serial.print(client.state());
			Serial.println(" Intentamos de nuevo en 5 segundos");
			delay(5000);
		}
	}
}


//*****************************
//***       CALLBACK        ***
//*****************************

void callback(char* topic, byte* payload, unsigned int length){
	String incoming = "";
	Serial.print("Mensaje recibido desde -> ");
	Serial.print(topic);
	Serial.println("");
	for (int i = 0; i < length; i++) {
		incoming += (char)payload[i];
	}
	incoming.trim();
	Serial.println("Mensaje -> " + incoming);

}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//              Funciones para obtener ángulo y vibración               //
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

void iniciarAcelerometro(){

  Wire.begin();
  Wire.setClock(400000);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    
    if(digitalRead(PIN_CALIBRACION)==1){
      Serial.println("Ejecutando calibración, aguarde...");
      delay(2000);

    // Funciones generadoras de OFFSET
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);

    // Funciones de OFFSET
    //mpu.setXGyroOffset(-400);
    //mpu.setYGyroOffset(69);
    //mpu.setZGyroOffset(-225);
    //mpu.setXAccelOffset(-1598);
    //mpu.setYAccelOffset(-627);
    //mpu.setZAccelOffset(1404);
    
    Serial.println();
    mpu.PrintActiveOffsets();
    }
    
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    //mpu.setMotionDetectionDuration(1000);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}


void obtenerAngulos(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gy, fifoBuffer);

#endif

  }
}


void envio10s(){
    delay(9900);
    AcZ_anterior=0;
    delay(100);
    json_to_mqtt_3_angulos();
}

void envio10ms(){

  for(int i = 0; i < 99; i++){
    json_to_mqtt_3();
    //mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    delay(10);
  }
  envioConFrecuencia();
}

void envioConFrecuencia(){

     json_to_mqtt_3_Hz();
}

void obtenerRAW(){
   mpu.setDLPFMode(0);
   //Leer los valores del Acelerometro de la IMU
   Wire.beginTransmission(0x68);
   Wire.write(0x3B);
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,6,true);
   AcX=Wire.read()<<8|Wire.read();
   AcY=Wire.read()<<8|Wire.read();
   AcZ=Wire.read()<<8|Wire.read();
   Wire.endTransmission(true);
   valoresRAW_gyro();
}
void valoresRAW_gyro(){
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,6,true);   //A partir del 0x43, se piden 6 registros
  GyX=Wire.read()<<8|Wire.read(); //Cada valor ocupa 2 registros
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
  Wire.endTransmission(true);
}

void vibracion(){

   obtenerRAW();
  if(AcX==-1) return;
  if( abs(AcZ - AcZ_anterior) > SENSIBILIDAD || abs(AcX - AcX_anterior) > SENSIBILIDAD || abs(AcY -AcY_anterior) > SENSIBILIDAD ){

    AcX_anterior=AcX;
    AcY_anterior=AcY;
    AcZ_anterior=AcZ;
    
    vibra=true;
  } 
  else{
    AcX_anterior=AcX;
    AcY_anterior=AcY;
    AcZ_anterior=AcZ;
    vibra=false;
  }
}

void contador(){
    int i = 0;
    cont=0;
    while(i == 0){
      vibracion();

        if(millis() - previousMillis >= interval){
          i=1;
          previousMillis = millis();
          cont_copia=cont;
          vibra=false;
          }

        else if(vibra==true){
          cont++;        
  }
  }
}

//+++++++++++++++++++++++++++++++++++++++++++++++++//
//  ++++++++++++   OBJETOS JSON   +++++++++++++++++//
//+++++++++++++++++++++++++++++++++++++++++++++++++//

// ++++ JSON enviado cada 10s ++++ //
void json_to_mqtt_3_angulos(){
const size_t capacity = 370;
DynamicJsonDocument doc(capacity);

JsonArray sensors = doc.createNestedArray("sensors");

JsonObject sensors_0 = sensors.createNestedObject();
sensors_0["name"] = "Vibración";

JsonArray sensors_0_values = sensors_0.createNestedArray("values");

JsonObject sensors_0_values_0 = sensors_0_values.createNestedObject();
sensors_0_values_0["type"] = "X";
sensors_0_values_0["value"] = (ypr[1] * 180 / M_PI);
sensors_0_values_0["unit"] = "r/s";

JsonObject sensors_0_values_1 = sensors_0_values.createNestedObject();
sensors_0_values_1["type"] = "Y";
sensors_0_values_1["value"] = (ypr[2] * 180 / M_PI);
sensors_0_values_1["unit"] = "r/s";

JsonObject sensors_0_values_2 = sensors_0_values.createNestedObject();
sensors_0_values_2["type"] = "Z";
sensors_0_values_2["value"] = (ypr[0] * 180 / M_PI);
sensors_0_values_2["unit"] = "r/s";

size_t n = serializeJson(doc, buffer_angulo);
client.publish(root_topic_publish, buffer_angulo, n);
//serializeJson(doc, Serial);
//Serial.println("\n");
}

// ++++ JSON sin frecuencia cada 10ms ++++ //
void json_to_mqtt_3(){

const size_t capacity = 720;
DynamicJsonDocument doc(capacity);

JsonArray sensors = doc.createNestedArray("sensors");

JsonObject sensors_0 = sensors.createNestedObject();
sensors_0["name"] = "Vibracion";

JsonArray sensors_0_values = sensors_0.createNestedArray("values");

JsonObject sensors_0_values_0 = sensors_0_values.createNestedObject();
sensors_0_values_0["type"] = "X";
sensors_0_values_0["value"] = (ypr[1] * 180 / M_PI);
sensors_0_values_0["unit"] = "r/s";

JsonObject sensors_0_values_1 = sensors_0_values.createNestedObject();
sensors_0_values_1["type"] = "Y";
sensors_0_values_1["value"] = (ypr[2] * 180 / M_PI);
sensors_0_values_1["unit"] = "r/s";

JsonObject sensors_0_values_2 = sensors_0_values.createNestedObject();
sensors_0_values_2["type"] = "Z";
sensors_0_values_2["value"] = (ypr[0] * 180 / M_PI);
sensors_0_values_2["unit"] = "r/s";

JsonObject sensors_1 = sensors.createNestedObject();
sensors_1["name"] = "giroscopio";

JsonArray sensors_1_values = sensors_1.createNestedArray("values");

JsonObject sensors_1_values_0 = sensors_1_values.createNestedObject();
sensors_1_values_0["type"] = "X";
sensors_1_values_0["value"] = GyX;
sensors_1_values_0["unit"] = "°/s";

JsonObject sensors_1_values_1 = sensors_1_values.createNestedObject();
sensors_1_values_1["type"] = "Y";
sensors_1_values_1["value"] = GyY;
sensors_1_values_1["unit"] = "°/s";

JsonObject sensors_1_values_2 = sensors_1_values.createNestedObject();
sensors_1_values_2["type"] = "Z";
sensors_1_values_2["value"] = GyZ;
sensors_1_values_2["unit"] = "°/s";

size_t n = serializeJson(doc, buffer);
client.publish(root_topic_publish, buffer, n);
//serializeJson(doc, Serial);
//Serial.println("\n");
}

// ++++ JSON con frecuencia ++++ //
void json_to_mqtt_3_Hz(){

const size_t capacity = 880;
DynamicJsonDocument doc(capacity);

JsonArray sensors = doc.createNestedArray("sensors");

JsonObject sensors_0 = sensors.createNestedObject();
sensors_0["name"] = "Vibracion";

JsonArray sensors_0_values = sensors_0.createNestedArray("values");

JsonObject sensors_0_values_0 = sensors_0_values.createNestedObject();
sensors_0_values_0["type"] = "X";
sensors_0_values_0["value"] = (ypr[1] * 180 / M_PI);
sensors_0_values_0["unit"] = "r/s";

JsonObject sensors_0_values_1 = sensors_0_values.createNestedObject();
sensors_0_values_1["type"] = "Y";
sensors_0_values_1["value"] = (ypr[2] * 180 / M_PI);
sensors_0_values_1["unit"] = "r/s";

JsonObject sensors_0_values_2 = sensors_0_values.createNestedObject();
sensors_0_values_2["type"] = "Z";
sensors_0_values_2["value"] = (ypr[0] * 180 / M_PI);
sensors_0_values_2["unit"] = "r/s";

JsonObject sensors_1 = sensors.createNestedObject();
sensors_1["name"] = "giroscopio";

JsonArray sensors_1_values = sensors_1.createNestedArray("values");

JsonObject sensors_1_values_0 = sensors_1_values.createNestedObject();
sensors_1_values_0["type"] = "X";
sensors_1_values_0["value"] = GyX;
sensors_1_values_0["unit"] = "°/s";

JsonObject sensors_1_values_1 = sensors_1_values.createNestedObject();
sensors_1_values_1["type"] = "Y";
sensors_1_values_1["value"] = GyY;
sensors_1_values_1["unit"] = "°/s";

JsonObject sensors_1_values_2 = sensors_1_values.createNestedObject();
sensors_1_values_2["type"] = "Z";
sensors_1_values_2["value"] = GyZ;
sensors_1_values_2["unit"] = "°/s";

JsonObject sensors_2 = sensors.createNestedObject();
sensors_2["name"] = "Frecuencia";

JsonArray sensors_2_values = sensors_2.createNestedArray("values");

JsonObject sensors_2_values_0 = sensors_2_values.createNestedObject();
sensors_2_values_0["type"] = "Frecuencia";
sensors_2_values_0["value"] = cont_copia;
sensors_2_values_0["unit"] = "Hz";

//serializeJson(doc, Serial);
//Serial.println("\n");
size_t n = serializeJson(doc, buffer_Hz);
client.publish(root_topic_publish, buffer_Hz, n);
}
