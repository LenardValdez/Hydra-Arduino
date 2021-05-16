//custom Libraries
#include "GravityTDS.h"       // Custom TDS library modified for Non Uno boards
#include "DFRobot_PH.h"       // Custom PH level Library modified for Non Uno boards
#include "DFRobot_EC.h"       // Custom EC level library modified for Non Uno boards

//Arduino Libraries
#include <Adafruit_SI1145.h>    //Original UV IR and Visible light Sensor Library
#include <DHT.h>                //Humidity and air temp sensor library
#include <DallasTemperature.h>  //Temprature probe sensor library
#include <Wire.h>
#include <OneWire.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <millisDelay.h>