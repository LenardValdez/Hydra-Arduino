#ifndef __HEADER_H__
#define __HEADER_H__

//custom Libraries
#include "DFRobot_PH.h"       // Custom PH level Library modified for Non Uno boards
#include "DFRobot_EC.h"       // Custom EC level library modified for Non Uno boards

//Arduino Libraries
#include <Adafruit_SI1145.h>    //UV IR and Visible light Sensor Library
#include <DHT.h>                //Humidity and air temp sensor library
#include <DallasTemperature.h>  //Temprature probe sensor library
#include <Wire.h>
#include <OneWire.h>
#include <ArduinoJson.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <millisDelay.h>
#include <SSLClient.h>
#endif // __HEADER_H__