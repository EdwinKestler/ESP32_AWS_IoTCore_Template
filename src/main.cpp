#include <Arduino.h>
#include "settings.h"             //Variables for connection such as MQTTSERVER TOPICS nad so on look in /lib dir

#include "SPIFFS.h"               // SPIFFS.h is the ESP32 native filesistem library witch manipulates files uploaded to the ESP32
#include <WiFiClientSecure.h>
#include "time.h"                 //time.h is the ESP32 native time library which does graceful NTP server synchronization

#include <PubSubClient.h>         //Pub/Sub library for MQTT communication
#define MQTT_MAX_PACKET_SIZE 512  //This tells the library what the maximun lenght of th massage can be. modify if bigger massage. can mesurate it here : https://arduinojson.org/v6/assistant/#

#include <ArduinoJson.h>          //Libarry for JSON data contrcution
//============================================================================================================================= SSL variables to store your certificates for after been read from the filesystem

String Read_rootca;
String Read_cert;
String Read_privatekey;

//============================================================================================================================= Program Variables.
#define BUFFER_LEN  256 //leght of buffer to store the msg befor data transmition in SendDatatoAWS()
long lastMsg = 0;       //variable to store the millis() when last time was of tanstion SendDatatoAWS()
char msg[BUFFER_LEN];   //buffer to store the data to be sent to AWS SendDatatoAWS()
byte mac[6];            //variable to stre mac values
char mac_Id[18];        //buffer to store mac value
int count = 1;          //variable to store how many data massages have been sent

uint32_t chipId = 0;    //variable donde se alamacena el identificador de cliente para el servicio de MQTT (OJO Este debe ser un identificador unico para cada dispositivo fisico sino se reiniciara el servidor MQTT)
//=============================================================================================================================Datetime variables.
time_t now;             //variable that stores NTP time in UNIX format
//=============================================================================================================================Sensor variables
float h ;               // Reading Temperature form DHT sensor
float t ;               // Reading Humidity form DHT sensor
//============================================================================================================================= StateMachine variables
static enum fsm_state {STATE_IDLE = 0, STATE_CHECK_SENSOR, STATE_TRANSMITIR_DATOS} fsm_state = STATE_IDLE;  
//============================================================================================================================= Communication Clients

WiFiClientSecure espClient;         //Cliente de WiFi
PubSubClient client(espClient);     //Cliente MQTT

//=============================================================================================================================> Setup Wifi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print(F("Connecting to "));
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(F(""));
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
}
//=============================================================================================================================> callback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] "); // imprimir el Topico de donde provino el menaje. 
  String payload_buff;
  for (int i = 0; i < length; i++) {
    payload_buff= payload_buff+ String((char)payload[i]);
  }
  Serial.println(payload_buff); //imprimir el mensaje recibido
  
  if (strcmp (rebootTopic, topic) == 0) {                                                                 //verificar si el topico conicide con el Topico rebootTopic[] definido en el archivo settings.h local
    Serial.println(F("Rebooting in 1 seconds..."));                                                                    //imprimir mensaje de Aviso sobre reinicio remoto de unidad.
    delay(1000);
    ESP.restart();                                                                                          //Emitir comando de reinicio para ESP32
  }

}
//=============================================================================================================================>Get Chip ID
void ESP32ID(){
  for(int i=0; i<17; i=i+8) {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }
  
  Serial.printf("ESP32 Chip model = %s Rev %d\n", ESP.getSdkVersion(), ESP.getChipRevision());
  Serial.printf("This chip has %d cores\n", ESP.getFlashChipSpeed());
  Serial.print("Chip ID: "); Serial.println(chipId);
  
  delay(3000);
}
//============================================================================================================================= Json de Configuracion 
void configuration_json (){
  // Create a random client ID
  char UniqueCID [60];
  String CID (clientId + String(chipId)); //Random number for client so to not collide wihth another node.
  Serial.println(CID); 
  CID.toCharArray(UniqueCID, 60);
  //Stablish Json Configutation file variables.
  #define JSON_MSG_BUFFER_LEN  256
  char json_msg_buffer [JSON_MSG_BUFFER_LEN];
  Serial.println(F("Sending Configuration Json to responseTopic"));
  char buf[16];
  sprintf(buf, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  Serial.println(String(buf));
  StaticJsonDocument<256> json_config_doc;
  // create an object
 
  json_config_doc["AliveUpdate:hrs"]   = isalivemsg_interval/3600000;
  json_config_doc["MqttServer"]        = mqtt_server;
  json_config_doc["MqttPort"]          = mqtt_port;
  json_config_doc["gloveCode"]         = chipId ;
  json_config_doc["IP"]                = String (buf);
  json_config_doc["ClientID"]          = String(UniqueCID);

  String JSONBuffer;
  serializeJson(json_config_doc, JSONBuffer);
  snprintf(json_msg_buffer, JSON_MSG_BUFFER_LEN,"%s", JSONBuffer.c_str());
  Serial.println(F("publishing device responseTopic:"));
  Serial.println(json_msg_buffer);
  while (!client.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (client.connect(UniqueCID)) {

      Serial.println(F("connected"));
      Serial.println(F("sending Configuration JSON"));

      if(client.publish(updateTopic, json_msg_buffer)){
        Serial.println(F("Configuration JSON Publish ok"));
      }else{
        Serial.println(F("device Publish failed:"));
      }

    } else {
      Serial.print(F("failed, rc="));
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//=============================================================================================================================> Reconnect
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    char UniqueCID [30];
    String CID (clientId + String(chipId)); //Random number for client so to not collide wihth another node. 
    CID.toCharArray(UniqueCID, 30);
    // Attempt to connect
    if (client.connect(UniqueCID)){
      Serial.println(F("connected"));
      Serial.println(F("sending reconnection status"));
      // Once connected, publish an announcement...
      client.publish(manageTopic, "{\"satatus\":\"reconnect\"}");
      // ... and resubscribe
      Serial.println(F("subscribing to topics"));
      if(client.subscribe(configTopic)){ // Subscribe to config topic, returns true or false
        Serial.print(F("Sucribed to, "));
        Serial.println(configTopic);
      }else{
        Serial.print(F("couldnt subscribe to: "));
        Serial.println(configTopic);
      }
      if(client.subscribe(rebootTopic)){ // Subscribe to Reboot topic, returns true or false
        Serial.println(rebootTopic);
      }else{
        Serial.print(F("couldnt subscribe to: "));
        Serial.println(rebootTopic);
      }
      if(client.subscribe(manageTopic)){ // Subscribe to manage topic, returns true or false
        Serial.print(F("Sucribed to, "));
        Serial.println(manageTopic);
      }else{
        Serial.print(F("couldnt subscribe to: "));
        Serial.println(manageTopic);
      }
      } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//============================================================================================================================= Print Local time with offsets
void printLocalTime()
{
  time_t now = time(nullptr);
  struct tm * timeinfo;
  //time (&now);
  timeinfo = localtime(&now);
  printf ("Current local time and date: %s\n", asctime(timeinfo));
  if(timeinfo->tm_year == 70){     // == number of years since 1900, 70 means no time obtained from NTP server
    Serial.println(F("Restart.."));
    delay(6000);
    ESP.restart();
  }      
}
//*************************************************************************************************************************-SETUP-**********************************************************
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
   //-------------------------------------------------------------------testing the setting
  //iniciamos desplegando informacion sobre el chip y la version de firmware. 
  Serial.println(F("")); 
  Serial.println(F("Inicializacion de programa de ESP32;"));
  Serial.println(F("Parametros de ambiente de funcionamiento:"));
  Serial.print(F("            CHIPID: "));
  ESP32ID();
  Serial.print(F("            HARDWARE: "));
  Serial.println(HardwareVersion);
  Serial.print(F("            FIRMWARE: "));
  Serial.println(FirmwareVersion);
  Serial.print(F("            Servidor de NTP: "));
  Serial.println(ntpServer);
  Serial.print(F("            Servidor de MQTT: "));
  Serial.println(mqtt_server);
  Serial.print(F("            Client ID: "));
  Serial.println(chipId); 
  delay(500);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(2, OUTPUT);
  setup_wifi();
  delay(1000);
  //=============================================================
  if (!SPIFFS.begin(true)) {
    Serial.println(F("An Error has occurred while mounting SPIFFS"));
    return;
  }
  //=======================================
  //Root CA File Reading.
  File file2 = SPIFFS.open("/AmazonRootCA1.pem", "r");
  if (!file2) {
    Serial.println(F("Failed to open file for reading"));
    return;
  }
  Serial.println(F("Root CA File Content:"));
  while (file2.available()) {
    Read_rootca = file2.readString();
    //Serial.println(Read_rootca);
  }
  //=============================================
  // Cert file reading
  File file4 = SPIFFS.open("/cda94561be-certificate.pem.crt", "r");
  if (!file4) {
    Serial.println(F("Failed to open file for reading"));
    return;
  }
  Serial.println(F("Cert File Content:"));
  while (file4.available()) {
    Read_cert = file4.readString();
    //Serial.println(Read_cert);
  }
  //=================================================
  //Privatekey file reading
  File file6 = SPIFFS.open("/cda94561be-private.pem.key", "r");
  if (!file6) {
    Serial.println(F("Failed to open file for reading"));
    return;
  }
  Serial.println(F("privateKey File Content:"));
  while (file6.available()) {
    Read_privatekey = file6.readString();
    //Serial.println(Read_privatekey);
  }
  //=====================================================

  char* pRead_rootca;
  pRead_rootca = (char *)malloc(sizeof(char) * (Read_rootca.length() + 1));
  strcpy(pRead_rootca, Read_rootca.c_str());

  char* pRead_cert;
  pRead_cert = (char *)malloc(sizeof(char) * (Read_cert.length() + 1));
  strcpy(pRead_cert, Read_cert.c_str());

  char* pRead_privatekey;
  pRead_privatekey = (char *)malloc(sizeof(char) * (Read_privatekey.length() + 1));
  strcpy(pRead_privatekey, Read_privatekey.c_str());

  espClient.setCACert(pRead_rootca);
  espClient.setCertificate(pRead_cert);
  espClient.setPrivateKey(pRead_privatekey);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  //====================================================================================================================
  WiFi.macAddress(mac);
  Serial.println(F("Read MAC address"));
  snprintf(mac_Id, sizeof(mac_Id), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(mac_Id);
  //===================================================================================================================== init and get the time
  Serial.println(F("Read MAC address"));
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //ignore the following precompiler error...for some reaseon i couldnt debug, but hey, it works as it should!. 
  setenv("TZ", "CST+6",1); // You must include '0' after first designator e.g. GMT0GMT-1, ',1' is true or ON
  printLocalTime();
  Serial.println(F("Terminando el Setup Enviando el Json de configuracion"));
  configuration_json();
}
//============================================================================================================================= Check The current time is SYNC
void CheckNTPTime (){
  //===============================================
  now = time(nullptr);
  struct tm * timeinfo;
  // time (&now); // not necessary
  timeinfo = localtime(&now);
  // below several approaches to time are displayed to help you get insight
  // spaces in this code are for ease of maintaining alignment in the monitor output
  printf(                 "1 - Current local time : %s\n", asctime(timeinfo));
  Serial.println((String) "6 - time(&now)         : "+time(&now)); //The Unix epoch (or Unix time or POSIX time or Unix timestamp) 
  Serial.println(F("*************************************************"));
  //===============================================
}
//============================================================================================================================= Check Sensor
void CheckSensor(){
  //suppose this are readnigs from your sensor.
  h = 96.02;      // Reading Temperature form DHT sensor
  t = 26.05;      // Reading Humidity form DHT sensor
  if (isnan(h) || isnan(t))
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }  
  // display TEMP
  Serial.print(h);
  Serial.print(F("Temperature: "));
  Serial.println(F("C"));
  // display humidity
  Serial.print(h);
  Serial.print(F("%"));
  Serial.print(F(" "));
  Serial.println(F("Rel H"));
  
}
//============================================================================================================================= Send Datato AWS
void SendDatatoAWS(String macIdStr, String Temprature, String Humidity){
  long sendnow = millis();
  if (sendnow - lastMsg > 5000) {
    CheckNTPTime();
    lastMsg = sendnow;
    //=============================================================================================
    StaticJsonDocument<256> sensor_data;
    sensor_data["mac_Id"]     =   macIdStr.c_str();
    sensor_data["temprature"] =   Temprature.c_str();
    sensor_data["humidity"]   =   Humidity.c_str();
    sensor_data["timestamp"]  =   time(&now);

    String JSONmessageBuffer;
    serializeJson(sensor_data, JSONmessageBuffer);

    snprintf(msg, BUFFER_LEN,"%s", JSONmessageBuffer.c_str());

    Serial.print("Publish message: ");
    Serial.println(msg);

    if(client.publish(eventTopic, msg)){
      Serial.println(F("device Publish ok"));
    }else {
      Serial.println(F("device Publish failed:"));
    }
    count = count + 1;
    Serial.print("messages published:");
    Serial.println(count);
    //================================================================================================
  }
}
//============================================================================================================================= Flash Onboard LED of ESP32
void flashLEDS(){
  digitalWrite(2, HIGH);             // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(2, LOW);              // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}

//*************************************************************************************************************************-LOOP-**********************************************************
void loop() {
  
  switch(fsm_state){
    case STATE_IDLE:
      if(millis() - last_Case_Status_Millis > intervalo_Case_Status_Millis){
        last_Case_Status_Millis = millis();
        Serial.print(F("fsm_state: "));
        Serial.println(fsm_state);
        Serial.println(F(""));
        fsm_state = STATE_CHECK_SENSOR;
      }
      
      if (!client.connected()){
        reconnect();
      }
      
      client.loop();
      delay(10);
    break;

    case STATE_CHECK_SENSOR:
      Serial.println(F("Listo para recolectar los datos del sensor"));
      CheckSensor();
      fsm_state = STATE_TRANSMITIR_DATOS;
    break;

    case STATE_TRANSMITIR_DATOS:
      Serial.println(F("Listo para transmitir la data recolectada"));
      SendDatatoAWS(mac_Id,String(t),String(h));
      delay(10);
      flashLEDS();
      fsm_state = STATE_IDLE;
    break;
    
    default:
      fsm_state = STATE_IDLE;
    break;
  }
}
