//Arduino framework
#include <Arduino.h>          // Arduino framework for CPP
// #include "header.h"           //Important headers for dependencies
#include "pin_constants.h"    //header for pin assignment constants
#include "mqttConstants.h"    //for mqtt constants

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

// priming delay inits
millisDelay primeDelay1, primeDelay2, primeDelay3, primeDelay4, primeDelay5;

//Global vars
char* new_crop_data;

//init indication
bool initialized;

//tolerance variables
int air_humidity_min;
int air_humidity_max;

int air_temperature_min;
int air_temperature_max;

float ec_reading_min;
int ec_reading_max;

float ph_reading_min;
float ph_reading_max;

void actuatePeristaltic() {
  digitalWrite(RELAY_PIN1, LOW);
  digitalWrite(RELAY_PIN2, LOW);
  digitalWrite(RELAY_PIN3, LOW);
  digitalWrite(RELAY_PIN4, LOW);
  digitalWrite(RELAY_PIN5, LOW);
}

void actuatePeristaltic(int pump_number){
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

void primePumps() {
  primeDelay1.start(40000/7);
  primeDelay2.start(5625);
  primeDelay3.start(7307.6923);
  primeDelay4.start(5555.5555);
  primeDelay5.start(5735.2941);
  actuatePeristaltic();
}

void newCrop(char* input) {
  size_t inputLength;
  StaticJsonDocument<256> doc;

  DeserializationError error = deserializeJson(doc, input, inputLength);

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
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println("");

  if(!strcmp(topic, command_new_crop)){
    for(int i = 0; i>length;i++){
      new_crop_data[i] = (char)payload[i];
    }
    newCrop(new_crop_data);
    initialized = true;
  }
  //if primetube command is recieved
  if(!strcmp(topic, topic_init_pumps)){
    char command_value[0];
    command_value[0] = (char)payload[0];
    if (command_value[0] == '1'){
    primePumps();
    }
  }
  /*To do:
  make routine for te following:
  1.) other specific commands
  2.) initialize plant/crop
  3.) harvest command
  */
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
StaticJsonDocument<256> sensor_data;
StaticJsonDocument<256> probe_data;
StaticJsonDocument<256> new_crop;

void setup() {
  Serial.begin(115200);

  waterPumpDelay.start(30000);  // pump 30 sec interval
  mqttDelay.start(5000);        // mqtt 5 sec send delay

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
  float phValue;
  float phVoltage;
  float temperature;
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U){                  //time interval: 1s
    timepoint = millis();
    temperature = readProbeTemperature();      // read your temperature sensor to execute temperature compensation
    phVoltage = analogRead(PH_PIN)/1024.0*3500;  // read the voltage
    phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
    return (phValue);
  }   
}

float EC_reading(){
  static unsigned long timepoint = millis();
  float temperature;
  float ecValue;
  float ecVoltage;
  if(millis()-timepoint>1000U)  //time interval: 1s
  {
    timepoint = millis();
    ecVoltage = analogRead(EC_PIN)/1024.0*3500;   // read the voltage
    temperature = readProbeTemperature();         // read your temperature sensor to execute temperature compensation
    ecValue =  ec.readEC(ecVoltage,temperature);  // convert voltage to EC with temperature compensation
    return (ecValue);
  }
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

float TDS_reading() {
  float tdsValue = 0;
  float Voltage = 0;
  static unsigned long timepoint = millis();
  if(millis()-timepoint>1000U)  //time interval: 1s
  {
    timepoint = millis();
    int sensorValue = analogRead(TdsSensorPin);
    Voltage = sensorValue*3.5/1024.0; //Convert analog reading to Voltage
    tdsValue=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value
    return (tdsValue);
  }
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
  if((air_humidity_min < hum_value) && (hum_value > air_humidity_max)){
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
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();
  if(initialized){
    light_check(infrared_light());
    run_fan(Air_humidity());

    char buffer[256];
    if(mqttDelay.justFinished()){
      sensor_data["air_humidity"] = Air_humidity();
      sensor_data["air_temperature"] = Air_temperature();
      sensor_data["contactless_liquid_level"] = contactless_liquid_level();
      sensor_data["uv_light"] = UV_light();
      sensor_data["infrared_light"] = infrared_light();
      sensor_data["visible_light"] = visible_light();
      sensor_data["reservoir_level"] = reservoir_level();
      probe_data["water_temperature"] = readProbeTemperature();
      probe_data["tds_reading"] = TDS_reading();
      probe_data["ec_reading"] = EC_reading();
      probe_data["ph_reading"] = PH_reading();
      size_t sens_dat = serializeJson(sensor_data, buffer);
      mqttClient.publish(topic_sensor_data, buffer, sens_dat);
      size_t probe_dat = serializeJson(probe_data, buffer);
      mqttClient.publish(topic_probe_data, buffer, probe_dat);
      mqttDelay.repeat();
    }

    // disabled for testing purposes(working)
    // if(waterPumpDelay.justFinished()){
    //   waterPumpActuate();
    // }

    //Ppump turn off
    if(primeDelay1.justFinished()){
      actuatePeristaltic(1);
    }
    if (primeDelay2.justFinished()){
      actuatePeristaltic(2);
    }
    if (primeDelay3.justFinished()){
      actuatePeristaltic(3);
    }
    if (primeDelay4.justFinished()){
      actuatePeristaltic(4);
    }
    if (primeDelay5.justFinished()){
      actuatePeristaltic(5);
    }
  }
}