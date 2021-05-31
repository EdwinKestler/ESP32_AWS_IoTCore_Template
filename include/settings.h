#include <Arduino.h>
//=============================================================================================================================WIFI SETTINGS

const char* ssid = "POSMOVIL1"; //Provide your SSID
const char* password = "TEC_pos2016";          // Provide Password

//=============================================================================================================================AWS MQTT IoT CORE Settings

const char* mqtt_server = "a1gyzb539iq1lf-ats.iot.us-west-2.amazonaws.com"; // Relace with your MQTT END point
const int mqtt_port = 8883;

//=============================================================================================================================Definicion de Topicos segun canal del mensaje

const char operationTopic[]   = "iot/edin/evnt/reponse/json";                                     //Topico al cual se publican las respuestas 
const char responseTopic[]    = "iot/edin/evnt/testing_tinyPico";                                 //Topico al cual se publica la identidad
const char configTopic[]      = "iot/edin/mgmt/manage";                                           //Topico al cual se publican los mensajes de estado
const char stillaliveTopic[]  = "iot/edin/device/update";                                         //Topico del cual se reciben las directrices de configuracion
const char rebootTopic[]      = "iot/edin/mgmt/initiate/device/reboot";                           //Topico del cual se reciben los mensajes de re-inicio
const char cambioTopic[]      = "iot/edin/evnt/initiate/device/cambio";                           //cambia de estado pregunta a estado respuesta
const char pongTopic[]        = "pong";

//=============================================================================================================================Definicion de versionamiento

String FirmwareVersion= "V1.00";                                                                  //Variable de version de Firmware
String HardwareVersion= "V1.00";         