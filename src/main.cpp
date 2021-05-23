//Arduino framework
#include <Arduino.h>          // Arduino framework for CPP
// #include "header.h"           //Important headers for dependencies
#include "pin_constants.h"    //header for pin assignment constants
#include "mqttConstants.h"    //for mqtt constants
#include "constants.h"

//Sensor Initialization
DFRobot_PH ph;
DFRobot_EC ec;
// GravityTDS gravityTds;
Adafruit_SI1145 uv = Adafruit_SI1145();
DHT dht(DHT21PIN, DHTTYPE);
OneWire oneWire(TEMP_SENSOR_PIN);
DallasTemperature probeSensor(&oneWire);

//pump interval
millisDelay waterPumpDelay;

//mqtt publish interval
millisDelay mqttDelay;

// peristaltic pumps delay inits
millisDelay phUPDelay, phDOWNDelay, nutrientCDelay, nutrientBDelay, nutrientADelay;

millisDelay phRoutineDelay, ecRoutineDelay;

//turn on and off peristaltic pump based on string "on" or "off" and the pump number
void actuatePeristaltic(String actuateType, int pump_number){
  
  if(!actuateType.compareTo("off")){
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

  else if (!actuateType.compareTo("on")){
    switch (pump_number)
    {
    case 1:
      digitalWrite(RELAY_PIN1, LOW);
      break;
    case 2:
      digitalWrite(RELAY_PIN2, LOW);
      break;
    case 3:
      digitalWrite(RELAY_PIN3, LOW);
      break;
    case 4:
      digitalWrite(RELAY_PIN4, LOW);
      break;
    case 5:
      digitalWrite(RELAY_PIN5, LOW);
      break;
    default:
      break;
    }
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

void primePumps() {
  phUPDelay.start(40000/7);
  actuatePeristaltic("on", 1);
  phDOWNDelay.start(5625);
  actuatePeristaltic("on", 2);
  nutrientCDelay.start(7307.6923);
  actuatePeristaltic("on", 3);
  nutrientBDelay.start(5555.5555);
  actuatePeristaltic("on", 4);
  nutrientADelay.start(5735.2941);
  actuatePeristaltic("on", 5);
}

void newCrop(StaticJsonDocument<256> doc,  byte* payload, unsigned int inputLength) {

  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  // const char* pod_name = doc["pod_name"]; // "hyd-1"

  air_humidity_min = doc["air_humidity"][0];
  air_humidity_max = doc["air_humidity"][1];

  air_temperature_min = doc["air_temperature"][0];
  air_temperature_max = doc["air_temperature"][1];

  ec_reading_min = doc["ec_reading"][0];
  ec_reading_max = doc["ec_reading"][1];

  ph_reading_min = doc["ph_reading"][0];
  ph_reading_max = doc["ph_reading"][1];

  waterPumpDelay.start(30000);    // pump 30 sec interval
  mqttDelay.start(5000);          // mqtt 5 sec send delay
  phRoutineDelay.start(1800000);  //ph check routine for pumping ph up and down
  ecRoutineDelay.start(1800000);  //ec routine for pumping nutrients
  initialized = true;
}

void changePH(StaticJsonDocument<48> doc,  byte* payload, unsigned int inputLength) {

  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  ph_reading_min = doc["ph_reading"][0];
  ph_reading_max = doc["ph_reading"][1];
}

void changeEC(StaticJsonDocument<48> doc,  byte* payload, unsigned int inputLength) {

  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  ec_reading_min = doc["ec_reading"][0];
  ec_reading_max = doc["ec_reading"][1]; 
}

void changeHUM(StaticJsonDocument<48> doc,  byte* payload, unsigned int inputLength) {

  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  air_humidity_min = doc["air_humidity"][0];
  air_humidity_max = doc["air_humidity"][1];
}

void changeTEMP(StaticJsonDocument<48> doc,  byte* payload, unsigned int inputLength) {

  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  air_humidity_min = doc["air_humidity"][0];
  air_humidity_max = doc["air_humidity"][1];
}

/* TODO:
    1.) initial 12mL for each nutrient
    2.) payload to int for checking single character callback like harvest and init pumps
*/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println("");

  //newcrop callback routine
  if(!initialized){
    if(!strcmp(topic, command_new_crop)){
      StaticJsonDocument<256> doc;
      newCrop(doc, payload, length);
    }
  }
  //change value callback routines
  if(initialized){
    if(!strcmp(topic, change_value_ph)){
      StaticJsonDocument<48> doc;
      changePH(doc, payload, length);
    }
    if(!strcmp(topic, change_value_ec)){
      StaticJsonDocument<48> doc;
      changeEC(doc, payload, length);
    }
    if(!strcmp(topic, change_value_air_hum)){
      StaticJsonDocument<48> doc;
      changeHUM(doc, payload, length);
    }
    if(!strcmp(topic, change_value_air_temp)){
      StaticJsonDocument<48> doc;
      changeTEMP(doc, payload, length);
    }
  }

  //if primetube command is recieved
  // if(!strcmp(topic, topic_init_pumps)){
  //   char command_value[0];
  //   command_value[0] = (char)payload[0];
  //   if (command_value[0] == '1'){
  //   primePumps();
  //   }
  // }

  //if harvest command is recieved
  // if(!strcmp(topic, harvest_command)){
  //   char harvest_command_value[0];
  //   harvest_command_value[0] = (char)payload[0];
  //   if (harvest_command_value[0] == '1'){
  //   initialized = false;
  //   }
  // }
}

// Ethernet and MQTT related objects
EthernetClient ethClient;
PubSubClient mqttClient(server, 1883, callback, ethClient);

void setup() {
  Serial.begin(115200);

  // waterPumpDelay.start(30000);    // pump 30 sec interval
  // mqttDelay.start(5000);          // mqtt 5 sec send delay
  // phRoutineDelay.start(1800000);  //ph check routine for pumping ph up and down
  // ecRoutineDelay.start(1800000);  //ec routine for pumping nutrients

  Ethernet.begin(mac, ip, myDns, gateway, subnet);

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
  // gravityTds.setPin(TdsSensorPin);
  // gravityTds.setAref(3.5);  //reference voltage on ADC, 3.5V on Arduino DUE
  // gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC

  ph.begin();           //PH sensor
  ec.begin();           //EC sensor
  dht.begin();          //hum&temp sensor
  probeSensor.begin();  //probe sensor
  // gravityTds.begin();   //TDS sensor

  //Check UV/IR/Vis light sensor readiness for I2C
  if (! uv.begin(&Wire1)) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
  mqttClient.subscribe(command_new_crop);
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("HYD-1")) {
      Serial.println("connected");
      // ...resubscribe
      mqttClient.subscribe(change_value_ph);
      mqttClient.subscribe(change_value_ec);
      mqttClient.subscribe(change_value_air_hum);
      mqttClient.subscribe(change_value_air_temp);
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

bool contactless_liquid_level(){
  bool liqsenseor;
  liqsenseor=digitalRead(Contact_less_sensor);
    if (liqsenseor==HIGH) { 
      return true;
    } 
    else { 
      return false;
    }
}

int reservoir_level(){
  
  int floatLow = !digitalRead(Float_Switch_Low);
  int floatHigh = !digitalRead(Float_Switch_High);
  
  if(floatHigh == 1 && floatLow == 0){
    return 100;
    }
  else if(floatHigh == 0 && floatLow == 0){
    return 50;
    }
  else if(floatHigh == 0 && floatLow == 1){
    return 0;
    }
  else {
    return -1;
   }
}

int visible_light() {
  return (uv.readVisible());
}

int infrared_light(){
  return (uv.readIR());
}

float uv_light() {
  float UVindex = uv.readUV();
  UVindex /= 100.0;
  return (UVindex);
}

float water_temperature(){
  probeSensor.requestTemperatures();
  float tempC = probeSensor.getTempCByIndex(0);
  return (tempC);
}

float ph_reading() {
  float phValue;
  float phVoltage;
  float temperature;
  temperature = water_temperature();      // read your temperature sensor to execute temperature compensation
  phVoltage = analogRead(PH_PIN)/1024.0*3500;  // read the voltage
  phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
  return (phValue);
}

float ec_reading(){
  float temperature;
  float ecValue;
  float ecVoltage;
  ecVoltage = analogRead(EC_PIN)/1024.0*3500;   // read the voltage
  temperature = water_temperature();         // read your temperature sensor to execute temperature compensation
  ecValue =  ec.readEC(ecVoltage,temperature);  // convert voltage to EC with temperature compensation
  return (ecValue);
}

// float TDS_reading() {
//   float tdsValue = 0;
//   float temperature;
//   static unsigned long timepoint = millis();
//   if(millis()-timepoint>1000U)  //time interval: 1s
//   {
//     temperature = readProbeTemperature();         // read your temperature sensor to execute temperature compensation
//     gravityTds.setTemperature(temperature);       // set the temperature and execute temperature compensation
//     gravityTds.update();                          //sample and calculate 
//     tdsValue = gravityTds.getTdsValue();          // then get the value
//     return (tdsValue);
//   }
// }

float tds_reading() {
  float tdsValue = 0;
  float Voltage = 0;
  int sensorValue = analogRead(TdsSensorPin);
  Voltage = sensorValue*3.5/1024.0; //Convert analog reading to Voltage
  tdsValue=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value
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

void run_fan(float hum_value, float temp_value) {
  if( (temp_value > air_temperature_max) || (hum_value > air_humidity_max) ){
    digitalWrite(RELAY_PIN6, LOW);
  } else {
    digitalWrite(RELAY_PIN6, HIGH);
  }
}

void phCheck() {
  float ph_value = ph_reading();
  if(ph_reading_min > ph_value){
    phUPDelay.start((1250/7)*10);
    actuatePeristaltic("on", 1);
  }
  if(ph_reading_max < ph_value){
    phDOWNDelay.start((156.25)*10);
    actuatePeristaltic("on", 2);
  }
}

void ecCheck() {
  float ec_value = ec_reading();
  if(ec_reading_min > ec_value){
    nutrientCDelay.start((2500/13)*10);
    nutrientBDelay.start((1250/9)*10);
    nutrientADelay.start((2500/17)*10);
    actuatePeristaltic("on", 3);
    actuatePeristaltic("on", 4);
    actuatePeristaltic("on", 5);
  }
}

float air_humidity() {
  float hum = dht.readHumidity();
  if (isnan(hum)) 
  {
    Serial.println("Failed to read from DHT");
  }
  return (hum);
}

float air_temperature(){
  float temp = dht.readTemperature();
  if (isnan(temp)) 
  {
    Serial.println("Failed to read from DHT");
  }
  return (temp);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  if(initialized){
    mqttClient.unsubscribe(command_new_crop);
    light_check(infrared_light());
    run_fan(air_humidity(), air_temperature());

    if(phRoutineDelay.justFinished()){
      phCheck();
      phRoutineDelay.repeat();
    }

    if(ecRoutineDelay.justFinished()){
      ecCheck();
      ecRoutineDelay.repeat();
    }

    char buffer[256];
    if(mqttDelay.justFinished()){
      //sensor data json
      StaticJsonDocument<128> sensor_data;
      StaticJsonDocument<64> probe_data;

      sensor_data["air_humidity"] = air_humidity();
      sensor_data["air_temperature"] = air_temperature();
      sensor_data["contactless_liquid_level"] = contactless_liquid_level();
      sensor_data["uv_light"] = uv_light();
      sensor_data["infrared_light"] = infrared_light();
      sensor_data["visible_light"] = visible_light();
      sensor_data["reservoir_level"] = reservoir_level();
      probe_data["water_temperature"] = water_temperature();
      probe_data["tds_reading"] = tds_reading();
      probe_data["ec_reading"] = ec_reading();
      probe_data["ph_reading"] = ph_reading();
      //publish data
      size_t sens_dat = serializeJson(sensor_data, buffer);
      mqttClient.publish(topic_sensor_data, buffer, sens_dat);
      size_t probe_dat = serializeJson(probe_data, buffer);
      mqttClient.publish(topic_probe_data, buffer, probe_dat);
      mqttDelay.repeat();
    }

    // disabled for testing purposes(working)
    // if(waterPumpDelay.justFinished()){
    //   waterPumpActuate();
    //   waterPumpDelay.repeat();
    // }
  }

  //Ppump turn off
  if(phUPDelay.justFinished()){
    actuatePeristaltic("off", 1);
  }
  if (phDOWNDelay.justFinished()){
    actuatePeristaltic("off", 2);
  }
  if (nutrientCDelay.justFinished()){
    actuatePeristaltic("off", 3);
  }
  if (nutrientBDelay.justFinished()){
    actuatePeristaltic("off", 4);
  }
  if (nutrientADelay.justFinished()){
    actuatePeristaltic("off", 5);
  }
}