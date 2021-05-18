//Arduino framework
#include <Arduino.h>          // Arduino framework for CPP
// #include "header.h"           //Important headers for dependencies
#include "pin_constants.h"    //header for pin assignment constants
#include "mqttConstants.h"    //for mqtt constants

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

//pump interval
millisDelay waterPumpDelay;

//mqtt publish interval
millisDelay mqttDelay;

millisDelay perPump;

millisDelay primeDelay1, primeDelay2, primeDelay3, primeDelay4, primeDelay5;


bool contactless_liquid_level(){
  if ((digitalRead(Contact_less_sensor)) == HIGH) { 
    return true;
  } 
  else { 
    return false;
  }
}

String reservoir_level(){
  
  if((!digitalRead(Float_Switch_High)) == 1 && (!digitalRead(Float_Switch_Low)) == 0){
    return ("100%");
    }
  else if((!digitalRead(Float_Switch_High)) == 0 && (!digitalRead(Float_Switch_Low)) == 0){
    return ("50%");
    }
  else if((!digitalRead(Float_Switch_High)) == 0 && (!digitalRead(Float_Switch_Low)) == 1){
    return ("0%");
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
  phVoltage = analogRead(PH_PIN)/1024.0*3500;  // read the voltage
  phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
  return (phValue);
}

float EC_reading(){
  ecVoltage = analogRead(EC_PIN)/1024.0*3500;   // read the voltage
  ecValue =  ec.readEC(ecVoltage,temperature);  // convert voltage to EC with temperature compensation
  return (ecValue);
}

float TDS_reading() {
  gravityTds.setTemperature(temperature);       // set the temperature and execute temperature compensation
  gravityTds.update();                          //sample and calculate 
  tdsValue = gravityTds.getTdsValue();          // then get the value
  return (tdsValue);
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

void primeTubes() {
  digitalWrite(RELAY_PIN1, LOW);
  digitalWrite(RELAY_PIN2, LOW);
  digitalWrite(RELAY_PIN3, LOW);
  digitalWrite(RELAY_PIN4, LOW);
  digitalWrite(RELAY_PIN5, LOW);
}

void primeTubes(int pump_number){
  switch (pump_number)
  {
  case 1:
    digitalWrite(RELAY_PIN1, HIGH);
    break;
  case 2:
    digitalWrite(RELAY_PIN2, HIGH);
    break;
  case 3:
    digitalWrite(RELAY_PIN3, HIGH);
    break;
  case 4:
    digitalWrite(RELAY_PIN4, HIGH);
    break;
  case 5:
    digitalWrite(RELAY_PIN5, HIGH);
    break;
  default:
    break;
  }
}

void waterPumpActuate(){
  if(digitalRead(RELAY_PIN8) == HIGH){
    digitalWrite(RELAY_PIN8, LOW);
  }
  else if (digitalRead(RELAY_PIN8) == LOW){
    digitalWrite(RELAY_PIN8, HIGH);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println("");
  if(!strcmp(topic, topic_commands)){
    primeDelay1.start(40000/7);
    primeDelay2.start(5625);
    primeDelay3.start(7307.6923);
    primeDelay4.start(5555.5555);
    primeDelay5.start(5735.2941);
    primeTubes();
    Serial.print("test");
  }
}

// Ethernet and MQTT related objects
EthernetClient ethClient;
PubSubClient mqttClient(server, 1883, callback, ethClient);

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("HYD-1")) {
      Serial.println("connected");
      // ...resubscribe
      // mqttClient.subscribe(topic_sensor_data);
      mqttClient.subscribe(topic_commands);
      // mqttClient.subscribe(connection);
      // Once connected, publish an announcement...
      mqttClient.publish(connection,"connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

//sensor data json
DynamicJsonDocument sensor_data(1024);

void setup() {
  Serial.begin(115200);

  waterPumpDelay.start(30000);  // pump 30 sec interval
  mqttDelay.start(5000);        // mqtt 5 sec send delay
  perPump.start(4923);

  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  mqttClient.setServer(server, 1883);

  pinMode(Float_Switch_Low, INPUT_PULLUP);
  pinMode(Float_Switch_High, INPUT_PULLUP);
  pinMode(Contact_less_sensor,INPUT);
  pinMode(RELAY_PIN8, OUTPUT);
  digitalWrite(RELAY_PIN8, HIGH);
  pinMode(RELAY_PIN7, OUTPUT);
  digitalWrite(RELAY_PIN7, HIGH);
  pinMode(RELAY_PIN6, OUTPUT);
  digitalWrite(RELAY_PIN6, HIGH);
  pinMode(RELAY_PIN5, OUTPUT);
  digitalWrite(RELAY_PIN5, HIGH);
  pinMode(RELAY_PIN4, OUTPUT);
  digitalWrite(RELAY_PIN4, HIGH);
  pinMode(RELAY_PIN3, OUTPUT);
  digitalWrite(RELAY_PIN3, HIGH);
  pinMode(RELAY_PIN2, OUTPUT);
  digitalWrite(RELAY_PIN2, HIGH);
  pinMode(RELAY_PIN1, OUTPUT);
  digitalWrite(RELAY_PIN1, HIGH);

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

void loop() {
  light_check(infrared_light());
  run_fan(Air_humidity());

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  char buffer[256];
  if(mqttDelay.justFinished()){
    sensor_data["Air_Humidity"] = Air_humidity();
    sensor_data["Air Temperature"] = Air_temperature();
    sensor_data["water_temp"] = readProbeTemperature();
    sensor_data["contactless_liquid_level"] = contactless_liquid_level();
    sensor_data["TDS_reading"] = TDS_reading();;
    sensor_data["EC_reading"] = EC_reading();
    sensor_data["PH_reading"] = PH_reading();
    sensor_data["UV_light"] = UV_light();
    sensor_data["infrared_light"] = infrared_light();
    sensor_data["visible_light"] = visible_light();
    sensor_data["reservoir_level"] = reservoir_level();

    // serializeJsonPretty(sensor_data, Serial);
    serializeJson(sensor_data, buffer);
    mqttClient.publish(topic_sensor_data, buffer);
    mqttDelay.restart();
  }

  // if(waterPumpDelay.justFinished()){
  //   waterPumpActuate();
  // }

  //Ppump turn off
  if(primeDelay1.justFinished()){
    primeTubes(1);
  }
  if (primeDelay2.justFinished()){
    primeTubes(2);
  }
  if (primeDelay3.justFinished()){
    primeTubes(3);
  }
  if (primeDelay4.justFinished()){
    primeTubes(4);
  }
  if (primeDelay5.justFinished()){
    primeTubes(5);
  }
}