#include <Arduino.h>

//=============================================================================================================================CLIENT Settings
#define ORG "flatbox"
#define DEVICE_TYPE "ESP32"
#define DEVICE_ID "AWS-Serverless"
char clientId[] = "d:" ORG ":" DEVICE_TYPE ":" DEVICE_ID;                                   //Variable de Identificacion de Cliente para servicio de MQTT

//=============================================================================================================================WIFI SETTINGS

const char* ssid = "POSMOVIL1"; //Provide your SSID
const char* password = "TEC_pos2016";          // Provide Password

//=============================================================================================================================AWS MQTT IoT CORE Settings

const char* mqtt_server = "a1gyzb539iq1lf-ats.iot.us-west-2.amazonaws.com";                 // Relace with your MQTT END point
const int mqtt_port = 8883;

//=============================================================================================================================Definicion de Topicos segun canal del mensaje

const char responseTopic[]    = "iot/evnt/reponse/json";                                    //Topico al cual se publican las respuestas 
const char eventTopic[]       = "iot/evnt/data";                                            //Topico al cual se publica la identidad
const char configTopic[]      = "iot/device/config";                                        //Topico al cual se publican los mensajes de estado
const char updateTopic[]      = "iot/device/update";                                        //Topico del cual se reciben las directrices de configuracion
const char rebootTopic[]      = "iot/mgmt/initiate/device/reboot";                          //Topico del cual se reciben los mensajes de re-inicio
const char manageTopic[]      = "iot/mgmt/initiate/device/manage";                          //cambia de estado pregunta a estado respuesta

//=============================================================================================================================Definicion de versionamiento

String FirmwareVersion= "V1.00";                                                            //Variable de version de Firmware
String HardwareVersion= "V1.00";         

//=============================================================================================================================Definicion de NTP
const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

//============================================================================================================================= Defeinicion de intervalos de tiempo paracontadores
unsigned long isalivemsg_interval           = 3600000;                                      //Cada cuanto debe enviar un mensaje de que esta vivo
