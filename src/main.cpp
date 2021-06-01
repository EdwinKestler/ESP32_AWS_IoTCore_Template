#include <Arduino.h>
#include "settings.h" //Variables for connection such as MQTTSERVER TOPICS nad so on look in /lib dir

#include "SPIFFS.h" // SPIFFS.h is the ESP32 native filesistem library witch manipulates files uploaded to the ESP32
#include <WiFiClientSecure.h>
#include "time.h" //time.h is the ESP32 native time library which does graceful NTP server synchronization

#include <PubSubClient.h>
#include <ArduinoJson.h>                      

String Read_rootca;
String Read_cert;
String Read_privatekey;

//=============================================================================================================================
#define BUFFER_LEN  256
long lastMsg = 0;
char msg[BUFFER_LEN];
int value = 0;
byte mac[6];
char mac_Id[18];
int count = 1;
uint32_t chipId = 0;                                                                                      //variable donde se alamacena el identificador de cliente para el servicio de MQTT (OJO Este debe ser un identificador unico para cada dispositivo fisico sino se reiniciara el servidor MQTT)

//=============================================================================================================================

WiFiClientSecure espClient;
PubSubClient client(espClient);

//=============================================================================================================================> Setup Wifi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros()); //initializes the pseudo-random number generator, causing it to start at an arbitrary point in its random sequence.

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
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

//=============================================================================================================================> Reconnect
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String cId = clientId;
    cId += String(random(0xffff), HEX); //Random number for client so to not collide wihth another node. 
    // Attempt to connect
    if (client.connect(cId.c_str())) {
      Serial.println("connected");
      Serial.println("sending reconnection status");
      // Once connected, publish an announcement...
      client.publish(manageTopic, "{\"satatus\":\"reconnect\"}");
      // ... and resubscribe
      Serial.println("subscribing to topics");
      client.subscribe(configTopic);
      client.subscribe(rebootTopic); // Subscribe to channel.
      client.subscribe(manageTopic); // Subscribe to channel.     
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
//============================================================================================================================= Print Local time with offsets
void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}
//============================================================================================================================= Json de Configuracion 
void configuration_json (){
  Serial.println("Sending Configuration Json to responseTopic");
  char buf[16];
  sprintf(buf, "IP:%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
  Serial.println(String(buf));

  const int capacity = JSON_OBJECT_SIZE(40);
  StaticJsonDocument<capacity> edin_json_config_doc;
  // create an object
  JsonObject object = edin_json_config_doc.to<JsonObject>();

  object["AliveUpdate:hrs"]   = isalivemsg_interval/3600000;
  object["MqttServer"]        = mqtt_server;
  object["MqttPort"]          = mqtt_port;
  object["gloveCode"]         = chipId ;
  object["IP"]                = String (buf);
  object["ClientID"]          = String(clientId);
    
  String output;
  size_t n = serializeJson(object, output);                                                                  //SAve CPU cycles by calculatinf the size.
  Serial.println(F("publishing device responseTopic:"));
  //Serial.println(output);
  
  if (!client.connected()) {
    Serial.println("Configjson: Client not connected reconnecting");
    reconnect();
  }
  if (client.connected()) {
    client.publish(responseTopic, (char*) output.c_str(), n);
    client.print(output);
    Serial.println(F("device Publish ok"));
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
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  //=======================================
  //Root CA File Reading.
  File file2 = SPIFFS.open("/AmazonRootCA1.pem", "r");
  if (!file2) {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("Root CA File Content:");
  while (file2.available()) {
    Read_rootca = file2.readString();
    Serial.println(Read_rootca);
  }
  //=============================================
  // Cert file reading
  File file4 = SPIFFS.open("/cda94561be-certificate.pem.crt", "r");
  if (!file4) {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("Cert File Content:");
  while (file4.available()) {
    Read_cert = file4.readString();
    Serial.println(Read_cert);
  }
  //=================================================
  //Privatekey file reading
  File file6 = SPIFFS.open("/cda94561be-private.pem.key", "r");
  if (!file6) {
    Serial.println("Failed to open file for reading");
    return;
  }
  Serial.println("privateKey File Content:");
  while (file6.available()) {
    Read_privatekey = file6.readString();
    Serial.println(Read_privatekey);
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
  snprintf(mac_Id, sizeof(mac_Id), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(mac_Id);
  //===================================================================================================================== init and get the time

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  configuration_json(); 
}

//*************************************************************************************************************************-LOOP-**********************************************************
void loop() {
  float h = 96.02;      // Reading Temperature form DHT sensor
  float t = 26.05;      // Reading Humidity form DHT sensor

  if (isnan(h) || isnan(t))
  {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  //===============================================

  // display TEMP
  Serial.print(h);
  Serial.print("Temperature: ");
  Serial.println("C");

  // display humidity
  Serial.print(h);
  Serial.print("%");
  Serial.print(" ");
  Serial.println("Rel H");


  //===============================================

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 5000) {
    lastMsg = now;
    //=============================================================================================
    String macIdStr = mac_Id;
    String Temprature = String(t);
    String Humidity = String(h);
    snprintf (msg, BUFFER_LEN, "{\"mac_Id\" : \"%s\", \"Temprature\" : %s, \"Humidity\" : \"%s\"}", macIdStr.c_str(), Temprature.c_str(), Humidity.c_str());
    Serial.print("Publish message: ");
    Serial.print(count);
    Serial.println(msg);
    client.publish(eventTopic, msg);
    count = count + 1;
    //================================================================================================
  }

  digitalWrite(2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(2, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}