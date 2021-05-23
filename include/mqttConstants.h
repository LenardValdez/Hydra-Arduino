#include "header.h"

//Ethernet Variable for connecting
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 117);
IPAddress myDns(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT server IP and topic subscription
const char* server = "192.168.1.65";
const char* topic_sensor_data = "hyd-1/sensor_data";
const char* topic_probe_data = "hyd-1/probe_data";
const char* command_new_crop = "hyd-1/commands/new_crop";
const char* change_value_ph = "hyd-1/commands/ph_reading";
const char* change_value_ec = "hyd-1/commands/ec_reading";
const char* change_value_air_hum = "hyd-1/commands/air_humidity";
const char* change_value_air_temp = "hyd-1/commands/air_temperature";
const char* topic_init_pumps = "hyd-1/commands/init_pumps";
const char* harvest_command = "hyd-1/commands/init_pumps/harvest";
const char* connection = "hyd-1/connection/";