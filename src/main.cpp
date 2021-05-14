//Arduino framework
#include <Arduino.h>

//custom Libraries
#include "GravityTDS.h"       // Custom TDS library modified for Non Uno boards
#include "DFRobot_PH.h"       // Custom PH level Library modified for Non Uno boards
#include "DFRobot_EC.h"       // Custom EC level library modified for Non Uno boards
#include <Ethernet.h>           //modified Ethernet client for larger bufers
#include "certificates.h"     //TLS cert of root CA information

//Arduino Libraries
#include <Adafruit_SI1145.h>    //Original UV IR and Visible light Sensor Library
#include <DHT.h>                //Humidity and air temp sensor library
#include <DallasTemperature.h>  //Temprature probe sensor library
#include <Wire.h>
#include <OneWire.h>
#include <SPI.h>
#include <SSLClient.h>          //TLS library for Arduino DUE
#include <PubSubClient.h>       //MQTT library for arduino

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
const int RELAY_PIN1 = 22;      //peristaltic pump 5 (right most)
const int RELAY_PIN2 = 24;      //peristaltic pump 4
const int RELAY_PIN3 = 26;      //peristaltic pump 3
const int RELAY_PIN4 = 28;      //peristaltic pump 2
const int RELAY_PIN5 = 30;      //peristaltic pump 1 (left most)
const int RELAY_PIN6 = 32;      //Fan
const int RELAY_PIN7 = 34;      //Grow light
const int RELAY_PIN8 = 36;      //Water pump

//MQTT vars and certs LOL A LONG LONG LONG STRING
const char my_cert[] = "-----BEGIN CERTIFICATE-----\nMIIDVjCCAj4CFFEYQdopwp5pc6S2mo48taHJ5TMeMA0GCSqGSIb3DQEBCwUAMIGA\nMQswCQYDVQQGEwJQSDEPMA0GA1UECAwGTWFuaWxhMQ8wDQYDVQQHDAZNYWthdGkx\nCzAJBgNVBAoMAkNBMQswCQYDVQQLDAJDYTEQMA4GA1UEAwwHSHlkcmFDQTEjMCEG\nCSqGSIb3DQEJARYUbGVuYXJkLmV4ZUBnbWFpbC5jb20wHhcNMjEwNTE0MDUzNTI4\nWhcNMjIwNTA5MDUzNTI4WjBOMQ0wCwYDVQQDDARIWUQxMQswCQYDVQQGEwJQSDEM\nMAoGA1UECAwDUFFFMQswCQYDVQQHDAJCRjEVMBMGA1UECgwMSFlEUkEtQ2xpZW50\nMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEA8G1ZuhISfp1XfJfUqFXi\ngEH9+UURT2y9FH6+1KGLDG1OkmpSf0Ny6I8XOWJEBQfZGUWhZWEIAv50Hd5kwSfq\nAfKI1kRTthu5x3J3sTJ+XK9muu8kNqYNcpCnEyFTaFlwNoDZGCtzV1O4YEQd3Gkb\nz+zoO/LXdTXL2vRNOU30Z2D5dCrt4aG/UZNAIIh3asVDrMgvwtxQN2RO09Qid+j9\nYHBPKOggJwTUG/hjf2HZ/jzQ+uMFXILL84mUqYc6M8u5Wc83zVf4jLfTXMiVunms\nyASpHB+SQ3eY6a8gLrtvsM+bvauwowTcyVG9na626gVLsdO1FTik6wVFRBUd0OBu\npQIDAQABMA0GCSqGSIb3DQEBCwUAA4IBAQA7wNCAxqS8ri7J/K/y2I8cS4SO5j6/\nvDR7J9RpgefoazdUkGY3i0W4lFfP/xlCrrBIjC2LYsZt/8jORx73Ju4SE1eIjY6F\nC4yffKBckNXGPzBieyAhUKbbo47yoyUj2PUtTobVbjLdQYOVuPgYxzTAthaZvePv\nhy5vngt2weLDDo6fdp1bTOwsGVGCpAVBzVYjeKljk7yRKbICwY8eCfOntT7ygxUP\n+0eeQbZMmw+ICcsgCbMzKIXlrvtDeQ/GjBOH1gDm0X/Q/EtdGXmC1cOLT0Xvaec/\nsyVKKHoEwaP0Jo8Fupmu7waGlBycIKcsFlmrWFV+BBWN2B6OuKHr66TI\n-----END CERTIFICATE-----\n";
const char my_key[] = "-----BEGIN RSA PRIVATE KEY-----\nMIIEowIBAAKCAQEA8G1ZuhISfp1XfJfUqFXigEH9+UURT2y9FH6+1KGLDG1OkmpS\nf0Ny6I8XOWJEBQfZGUWhZWEIAv50Hd5kwSfqAfKI1kRTthu5x3J3sTJ+XK9muu8k\nNqYNcpCnEyFTaFlwNoDZGCtzV1O4YEQd3Gkbz+zoO/LXdTXL2vRNOU30Z2D5dCrt\n4aG/UZNAIIh3asVDrMgvwtxQN2RO09Qid+j9YHBPKOggJwTUG/hjf2HZ/jzQ+uMF\nXILL84mUqYc6M8u5Wc83zVf4jLfTXMiVunmsyASpHB+SQ3eY6a8gLrtvsM+bvauw\nowTcyVG9na626gVLsdO1FTik6wVFRBUd0OBupQIDAQABAoIBAHgShI8y2120gq7M\nvP3c9qOLUaWd4slByyRjMDbM/r6UEHt68fxw2QBgBf84njPM+ZVveu1AilnEboVE\nPhlhL7MNR12K/iuiqSN8fV5Xk1gCmVGegP1x7KFcsLQTHq6sjCgMLV+uw4nHW/tn\nyWn8bFFpRYo6+3OkDE0rWS3XSsodYlx4pFthSVoXmjxvpHjFfmqwY/MYkQdKJjsz\ne1zj4eWW02LM0IZWfEEJm8V3IcQyxtWBc/VSWT0DSWgIXxZ4kZ/ZsK/WqvqwQ8Ce\nWNsYs0FSP3Duk4kdkCxyPgllBiUIzCl50b53CT/5qZIdEoPdnA+vSFY/Lr+CBNHn\nKWcF+YECgYEA/UDEIFcsAeNHqjKu7NKbqAZahhLaV5c9s91GqJBcyJlmJBUvHQMk\ntW91jBFItXzDneM0gnVhl613Ow+VQndbgwAIx2VHiEUFxOddzdY2gWj7ZR2OXdmU\nhA37dVwMiHC1+sCJxiOFpXIqMW5AhHawcSRI0aNHtGD3bXKT28/Z8TUCgYEA8wj4\nMwyNEuTo4/jxE8QOtUmsgxGR5xs8yoH3cDvuxHbA418vA4WvIvK79sVBNe/87CIN\nBAfqUtxt7/PxIT8TrrCpmsWVfiUZTDn88FVho7a4Sm4eSVy4+cek4XehwERpBiqf\ndVQkyRBkt0CqBvSK9sNUKygsABVJpOqMusyEJbECgYAGPKgX2827pitHp8PjFTS8\nKJvmHWS/L3xXh6WL1TJTxmNblCH65u7qgPo2Ht7ZiV8P7l6Gr/ldUOpUzAdzAZmB\nWmjc8EOYuYpw1PncbeAdw8YdMWMmMIuVzPpaY4/zdEMnD8LU3F8YOf2MBhkUqg+i\nWbBpaUoEyTaOvXmiujW12QKBgCQQV6xTl3ePSlPR9Awn0rrqFNRaPhlaPFockRgx\nfIvgx1y2Gibepo98D5Jd0QBs/U4Y+292sTnchd/OhiCV2JjalEBoQ3e1j5x1Gmib\ncuK1UYGR96KRRM2j536hnvoF7MPCfWkOtGq1qQxcZx6jGR/m+k1xJ55XilcTixCJ\nGq7xAoGBAN8PYDRKwLo+A3Ygn6ojE6YIDMuOWjZAJJDlXKAwOaaK5KayKx5rE9t2\n3thJqC9cHoFrggJEBpGWtXhW2t40n0ortIDHqHBFAbFketuoiuV/DASkqe2/KfzB\nJ6K5DHK7OlVF1fx177o7kJGA59RTn8ZcIigWwAGpETOdxUqcyp2b\n-----END RSA PRIVATE KEY-----\n";
SSLClientParameters mTLS = SSLClientParameters::fromPEM(my_cert, sizeof my_cert, my_key, sizeof my_key);

//Sensor Initialization
DFRobot_PH ph;
DFRobot_EC ec;
GravityTDS gravityTds;
Adafruit_SI1145 uv = Adafruit_SI1145();
DHT dht(DHT21PIN, DHTTYPE);
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature probeSensor(&oneWire);

//global variables
float phValue;
float phVoltage;
float ecValue;
float ecVoltage;
float temperature;
float tdsValue = 0;
int liqsenseor = 0;

//Ethernet vars and MQTTS
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
byte ip[] = { 192, 168, 1, 177 };
const char* mqttServer = "mqtts.HYDRA-Server.com"; // Broker address

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

//initialize ethernet and mqtt
EthernetClient ethClient;
SSLClient ethClientSSL(ethClient, TAs, (size_t)TAs_NUM, A5);
PubSubClient client(mqttServer, 8883, callback, ethClientSSL);

//connect to MQTT Broker/server
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic","hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Ethernet.begin(mac, ip);
  Serial.begin(115200);

  pinMode(Float_Switch_Low, INPUT_PULLUP);
  pinMode(Float_Switch_High, INPUT_PULLUP);
  pinMode(Contact_less_sensor,INPUT);
  pinMode(RELAY_PIN7, OUTPUT);
  pinMode(RELAY_PIN6, OUTPUT);
  pinMode(RELAY_PIN1, OUTPUT);

  //TDS setup default params
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(3.5);  //reference voltage on ADC, 3.5V on Arduino DUE
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC

  ph.begin();           //PH sensor
  ec.begin();           //EC sensor
  dht.begin();          //hum&temp sensor
  probeSensor.begin();  //probe sensor
  gravityTds.begin();   //TDS sensor

  //Check UV/IR/Vis light sensor readiness for I2C
  if (! uv.begin(&Wire1)) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
}

bool contactless_liquid_level(){
  liqsenseor=digitalRead(Contact_less_sensor);
    if (liqsenseor==HIGH) { 
      return true;
//      Serial.println("Water level is detected");
    } 
    else { 
      return false;
//      Serial.println("No water"); 
    }
}

String reservior_level(){
  
  int floatLow = !digitalRead(Float_Switch_Low);
  int floatHigh = !digitalRead(Float_Switch_High);
  
  if(floatHigh == 1 && floatLow == 0){
    return ("100%");
//    Serial.println(floatHigh);
//    Serial.println(floatLow);
//    Serial.println("100%");
    }
  else if(floatHigh == 0 && floatLow == 0){
    return ("50%");
//    Serial.println(floatHigh);
//    Serial.println(floatLow);
//    Serial.println("50%");
    }
  else if(floatHigh == 0 && floatLow == 1){
    return ("0%");
//    Serial.println(floatHigh);
//    Serial.println(floatLow);
//    Serial.println("0%");
    }
  else {
    return ("Float sensors might be stuck");
   }
}

int visible_light() {
  return (uv.readVisible());
}

int infrared_light(){
  return (uv.readIR());
}

float UV_light() {
  float UVindex = uv.readUV();
  UVindex /= 100.0;
  return (UVindex);
}

float readProbeTemperature(){
  probeSensor.requestTemperatures();
  float tempC = probeSensor.getTempCByIndex(0);
  return (tempC);
}

float PH_reading() {
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        // temperature = readProbeTemperature();      // read your temperature sensor to execute temperature compensation
        phVoltage = analogRead(PH_PIN)/1024.0*3500;  // read the voltage
        phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
        return (phValue);
//        Serial.print("temperature:");
//        Serial.print(temperature,1);
//        Serial.print("^C  pH:");
//        Serial.println(phValue,2);
    }   
}

float EC_reading(){
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      ecVoltage = analogRead(EC_PIN)/1024.0*3500;   // read the voltage
      // temperature = readProbeTemperature();         // read your temperature sensor to execute temperature compensation
      ecValue =  ec.readEC(ecVoltage,temperature);  // convert voltage to EC with temperature compensation
      return (ecValue);
//      Serial.print("^C  EC:");
//      Serial.print(ecValue,4);
//      Serial.println("ms/cm");
    }
}

float TDS_reading() {
  // temperature = readProbeTemperature();         // read your temperature sensor to execute temperature compensation
  gravityTds.setTemperature(temperature);       // set the temperature and execute temperature compensation
  gravityTds.update();                          //sample and calculate 
  tdsValue = gravityTds.getTdsValue();          // then get the value
  return (tdsValue);
//  Serial.print(tdsValue,4);
//  Serial.println("ppm");
}

void light_check(int ir_value){
  if(ir_value < 300){
    digitalWrite(RELAY_PIN7, LOW);
  } else {
    digitalWrite(RELAY_PIN7, HIGH);
  }
  delay(1000);
}

void run_fan(float hum_value) {
  if(hum_value > 70){
    digitalWrite(RELAY_PIN6, LOW);
  } else {
    digitalWrite(RELAY_PIN6, HIGH);
  }
}

float Air_humidity() {
  float hum = dht.readHumidity();
  if (isnan(hum)) 
  {
    Serial.println("Failed to read from DHT");
  }
  return (hum);
}

float Air_temperature(){
  float temp = dht.readTemperature();
  if (isnan(temp)) 
  {
    Serial.println("Failed to read from DHT");
  }
  return (temp);
}

void loop() {
  light_check(infrared_light());
  run_fan(Air_humidity());
  temperature = readProbeTemperature();
  Serial.print("\n============================================\n");
  Serial.print("Humidity: "); Serial.print(Air_humidity()); Serial.print("%\n");
  Serial.print("Air Temperature: "); Serial.print(Air_temperature()); Serial.print("*C\n");
  Serial.print("Contactless: "); Serial.print(contactless_liquid_level());
  Serial.print("\nTDS: "); Serial.print(TDS_reading()); Serial.print("ppm\n");
  Serial.print("EC level: "); Serial.print(EC_reading(),2); Serial.print("mS/cm\n");
  Serial.print("PH level: ");Serial.print(PH_reading(),2);
  Serial.print("\nWater temperatre: "); Serial.print(temperature); Serial.print("*C\n");
  Serial.print("UV level: "); Serial.print(UV_light());
  Serial.print("\nInfrared level: ") ;Serial.print(infrared_light());
  Serial.print("\nVisible light(lux): "); Serial.print(visible_light());
  Serial.print("\nReservior level: "); Serial.print(reservior_level());
  delay(5000);
}