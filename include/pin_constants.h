//Pins of Analog sensors
#define PH_PIN A1           //PH Level Sensor
#define EC_PIN A2           //EC Level Sensor
#define TdsSensorPin A3     //TDS sensor
#define DHTTYPE DHT21       // AM2301 

//Pins of Digital Sensors
const int Contact_less_sensor = 3;  // Contact less liquid sensor
#define DHT21PIN 4                  // Humidity and Air temp Sensor
#define TEMP_SENSOR_PIN 5           // Temperature probe Sensor
const int Float_Switch_Low = 6;     // Lower float siwtch on reservior
const int Float_Switch_High = 7;    // Higher float siwtch on reservior

//Relay PINS and Assignment
// const int RELAY_PIN1 = 22;      //peristaltic pump 5 (right most)
// const int RELAY_PIN2 = 24;      //peristaltic pump 4
// const int RELAY_PIN3 = 26;      //peristaltic pump 3
// const int RELAY_PIN4 = 28;      //peristaltic pump 2
// const int RELAY_PIN5 = 30;      //peristaltic pump 1 (left most)
const int RELAY_PIN6 = 32;      //Fan
const int RELAY_PIN7 = 34;      //Grow light
const int RELAY_PIN8 = 36;      //Water pump