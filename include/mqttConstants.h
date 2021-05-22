#include "header.h"

//Ethernet Variable for connecting
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 117);
IPAddress myDns(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT server IP and topic subscription
const char* server = "192.168.1.65";
const char* topic_sensor_data = "HYD-1/sensor_data/";
const char* topic_probe_data = "HYD-1/probe_data/";
const char* topic_commands = "HYD-1/commands/"; //to be updated with other commands with sub topics
const char* topic_init_pumps = "HYD-1/commands/init_pumps/";
const char* connection = "HYD-1/Connection/";