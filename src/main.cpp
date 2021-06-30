//Arduino framework
#include <Arduino.h>          // Arduino framework for CPP
#include "pin_constants.h"    //header for pin assignment constants
#include "mqttConstants.h"    //for mqtt constants
#include "constants.h"

//Sensor Initialization
DFRobot_PH ph;
DFRobot_EC ec;
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

//routine check for EC and PH level
millisDelay phRoutineDelay, ecRoutineDelay, ECPHFirstRunDelay;

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

//manual control of prming the peristaltic pump
void primePumps(byte* payload, unsigned int inputLength) {
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  if(doc["manual_prime"] == true){
    JsonArray pump_select = doc["pump_select"];
    if(pump_select[0] == true){
      phUPDelay.start(40000/7);
      actuatePeristaltic("on", 1);
    }
    if(pump_select[1] == true){
      phDOWNDelay.start(5625);
      actuatePeristaltic("on", 2);
    }
    if(pump_select[2] == true){
      nutrientCDelay.start(7307.6923);
      actuatePeristaltic("on", 3);
    }
    if(pump_select[3] == true){
      nutrientBDelay.start(5555.5555);
      actuatePeristaltic("on", 4);
    }
    if(pump_select[4] == true){
      nutrientADelay.start(5735.2941);
      actuatePeristaltic("on", 5);
    }
  }
}

//water pump/sprinkler actuation
void waterPumpActuate(){
  if(digitalRead(RELAY_PIN8) == HIGH){
    digitalWrite(RELAY_PIN8, LOW);
  }
  else if (digitalRead(RELAY_PIN8) == LOW){
    digitalWrite(RELAY_PIN8, HIGH);
  }
}

void ph_ec_actuationRun(){
  phRoutineDelay.start(600000);     //ph check routine for pumping ph up and down (10min)
  ecRoutineDelay.start(600000);     //ec routine for pumping nutrients (10min)
}

void firstRun() {
  ECPHFirstRunDelay.start(300000); //5min wait time to fill up the psuedo reservoir
}

//new crop routine for initializing new crop to the Pod
void newCrop(byte* payload, unsigned int inputLength) {

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  //naming is to be implemented
  // const char* pod_name = doc["pod_name"]; // "hyd-1"

  tolerance.air_humidity_min = doc["air_humidity"][0];
  tolerance.air_humidity_max = doc["air_humidity"][1];

  tolerance.air_temperature_min = doc["air_temperature"][0];
  tolerance.air_temperature_max = doc["air_temperature"][1];

  tolerance.ec_reading_min = doc["ec_reading"][0];
  tolerance.ec_reading_max = doc["ec_reading"][1];

  tolerance.ph_reading_min = doc["ph_reading"][0];
  tolerance.ph_reading_max = doc["ph_reading"][1];

  //check if prime tubes is true if not just pump 12mL of nutrients(A B C)
  if(!primed){
    if(doc["init_pump"] == true){
      phUPDelay.start(40000/7);
      actuatePeristaltic("on", 1);
      phDOWNDelay.start(5625);
      actuatePeristaltic("on", 2);
      nutrientCDelay.start(395000/13);
      actuatePeristaltic("on", 3);
      nutrientBDelay.start(200000/9);
      actuatePeristaltic("on", 4);
      nutrientADelay.start(397500/17);
      actuatePeristaltic("on", 5);
      mqttClient.publish(pumps_primed, "true", true);
    }
    if(doc["init_pump"] == false){
      nutrientCDelay.start(((2500/13)*10)*12);
      actuatePeristaltic("on", 3);
      nutrientBDelay.start(((1250/9)*10)*12);
      actuatePeristaltic("on", 4);
      nutrientADelay.start(((2500/17)*10)*12);
      actuatePeristaltic("on", 5);
      mqttClient.publish(pumps_primed, "true", true);
    }
  }

  //subscribe to change value topics and unsubscribe to new crop for protection of reinitializing
  mqttClient.subscribe(change_value_ph);
  mqttClient.subscribe(change_value_ec);
  mqttClient.subscribe(change_value_air_hum);
  mqttClient.subscribe(change_value_air_temp);
  mqttClient.subscribe(harvest_command);
  mqttClient.subscribe(manual_prime);
  mqttClient.subscribe(EC_PH_time);
  mqttClient.unsubscribe(command_new_crop);
  mqttClient.unsubscribe(pumps_primed);
  //change intialization state
  initialized = true;
  if(!first_run){
    ph_ec_actuationRun();
  }
  else {
    firstRun();
    mqttClient.publish(EC_PH_time, "false", true);
  }
}
//changing the value of the tolerance codeblock
/*---------------------------------------START OF CHANGE VALUE CODE BLOCK-----------------------------------------*/
void changePH(byte* payload, unsigned int inputLength) {
  StaticJsonDocument<48> doc;
  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  tolerance.ph_reading_min = doc["ph_reading"][0];
  tolerance.ph_reading_max = doc["ph_reading"][1];
}

void changeEC(byte* payload, unsigned int inputLength) {
  StaticJsonDocument<48> doc;
  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  tolerance.ec_reading_min = doc["ec_reading"][0];
  tolerance.ec_reading_max = doc["ec_reading"][1]; 
}

void changeHUM(byte* payload, unsigned int inputLength) {
  StaticJsonDocument<48> doc;
  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  tolerance.air_humidity_min = doc["air_humidity"][0];
  tolerance.air_humidity_max = doc["air_humidity"][1];
}

void changeTEMP(byte* payload, unsigned int inputLength) {
  StaticJsonDocument<48> doc;
  DeserializationError error = deserializeJson(doc, payload, inputLength);

  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  tolerance.air_temperature_min = doc["air_temperature"][0];
  tolerance.air_temperature_max = doc["air_temperature"][1];
}
/*-------------------------------------END OF CHANGE VALUE CODE BOCK-------------------------------------------*/

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println("");

  if(!primed){
    if(!strcmp(topic, pumps_primed)){
      if (!strncmp((char *)payload, "true", length)) {
        primed = true;
      }
    }
  }

  if(first_run){
    if(!strcmp(topic, EC_PH_time)){
      if (!strncmp((char *)payload, "false", length)) {
        first_run = false;
      }
    }
  }

  //newcrop callback routine to assign tolerance
  if(!initialized){
    if(!strcmp(topic, command_new_crop)){
      newCrop(payload, length);
    }
  }
  //change value callback routines for tolerance change value after initialization
  if(initialized){
    if(!strcmp(topic, change_value_ph)){
      changePH(payload, length);
    }
    if(!strcmp(topic, change_value_ec)){
      changeEC(payload, length);
    }
    if(!strcmp(topic, change_value_air_hum)){
      changeHUM(payload, length);
    }
    if(!strcmp(topic, change_value_air_temp)){
      changeTEMP(payload, length);
    }

    if(!strcmp(topic, manual_prime)){
      primePumps(payload, length);
    }

    // if harvest command is recieved
    if(!strcmp(topic, harvest_command)){
      StaticJsonDocument<16> doc;
      deserializeJson(doc, payload, length);
      if(doc["harvest"] == true){
        // reset initialized state and prime state
        initialized = false;
        primed = false;
        // clear retained value of self-notes topic
        mqttClient.publish(pumps_primed, "", true);
        mqttClient.publish(EC_PH_time, "", true);
        //turn all 12v relay connection
        digitalWrite(RELAY_PIN8, HIGH);
        digitalWrite(RELAY_PIN7, HIGH);
        digitalWrite(RELAY_PIN6, HIGH);
        digitalWrite(RELAY_PIN5, HIGH);
        digitalWrite(RELAY_PIN4, HIGH);
        digitalWrite(RELAY_PIN3, HIGH);
        digitalWrite(RELAY_PIN2, HIGH);
        digitalWrite(RELAY_PIN1, HIGH);
        // unsubscribe to all commands except new_crop
        mqttClient.unsubscribe(harvest_command);
        mqttClient.unsubscribe(change_value_ph);
        mqttClient.unsubscribe(change_value_ec);
        mqttClient.unsubscribe(change_value_air_hum);
        mqttClient.unsubscribe(change_value_air_temp);
        mqttClient.unsubscribe(manual_prime);
        mqttClient.unsubscribe(EC_PH_time);
        //resubscribe to newcrop command
        mqttClient.subscribe(command_new_crop);
      }
    }
  }
}

void reconnect() {
  // Loop until we're reconnected to the broker
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("HYD-1", connection, 2, false, LWAT)) {
      Serial.println("connected");
      mqttClient.subscribe(pumps_primed);
      mqttClient.subscribe(EC_PH_time);
      //resubscribe to topics acording to state of initialization
      if (!initialized){
        mqttClient.subscribe(command_new_crop);
      }
      if(initialized){
        mqttClient.subscribe(change_value_ph);
        mqttClient.subscribe(change_value_ec);
        mqttClient.subscribe(change_value_air_hum);
        mqttClient.subscribe(change_value_air_temp);
        mqttClient.subscribe(harvest_command);
        mqttClient.subscribe(manual_prime);
      }
      //publish connection status
      mqttClient.publish(connection,"connected");
    } 
    else { //if connection failed print to Serial of the reason of failure then try again
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
  temperature = water_temperature();           // read your temperature sensor to execute temperature compensation
  phVoltage = analogRead(PH_PIN)/1024.0*3500;  // read the voltage
  phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
  return (phValue);
}

float ec_reading(){
  float temperature;
  float ecValue;
  float ecVoltage;
  ecVoltage = analogRead(EC_PIN)/1024.0*3500;   // read the voltage
  temperature = water_temperature();            // read your temperature sensor to execute temperature compensation
  ecValue =  ec.readEC(ecVoltage,temperature);  // convert voltage to EC with temperature compensation
  return (ecValue);
}

float tds_reading() {
  float tdsValue;
  float Voltage;
  float tdsValueTemp;
  double tdsFactor = 0.5;
  float temperature = water_temperature();
  int sensorValue = analogRead(TdsSensorPin);
  Voltage = sensorValue*3.5/1024.0;                                                         //Convert analog reading to Voltage
  tdsValue=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*1.0;  //Convert voltage value to TDS value
  tdsValueTemp = tdsValue / (1.0+0.02*(temperature-25.0));                                  //temperature compensation
  tdsValue = tdsValueTemp * tdsFactor;
  return (tdsValue);
}

void light_check(int ir_value){
  if(ir_value < 260){
    digitalWrite(RELAY_PIN7, LOW);
  } else {
    digitalWrite(RELAY_PIN7, HIGH);
  }
  delay(1000);
}

void run_fan(float hum_value, float temp_value) {
  if( (temp_value > tolerance.air_temperature_max) || (hum_value > tolerance.air_humidity_max) ){
    digitalWrite(RELAY_PIN6, LOW);
  } else {
    digitalWrite(RELAY_PIN6, HIGH);
  }
}

// the ph actuation
void phCheck() {
  float ph_value = ph_reading();
  if(tolerance.ph_reading_min > ph_value){
    phUPDelay.start((1250/7)*20); //time for 0.1mL *20 = 2mL
    actuatePeristaltic("on", 1);
  }
  if(tolerance.ph_reading_max < ph_value){
    phDOWNDelay.start((156.25)*20); // time for 0.1mL * 20 = 2mL
    actuatePeristaltic("on", 2);
  }
}

//the ec actuation
void ecCheck() {
  float ec_value = ec_reading();
  if(tolerance.ec_reading_min > ec_value){
    nutrientCDelay.start((2500/13)*20);
    actuatePeristaltic("on", 3);
    nutrientBDelay.start((1250/9)*20);
    actuatePeristaltic("on", 4);
    nutrientADelay.start((2500/17)*20);
    actuatePeristaltic("on", 5);
  }
}

//function that returns the air humidity from the sensor reading
float air_humidity() {
  float hum = dht.readHumidity();
  if (isnan(hum)) 
  {
    Serial.println("Failed to read from DHT");
  }
  return (hum);
}

//function that returns the air temp from the sensor reading
float air_temperature(){
  float temp = dht.readTemperature();
  if (isnan(temp)) 
  {
    Serial.println("Failed to read from DHT");
  }
  return (temp);
}

//setup run once only once when the Arduinis turned on have been reset
void setup() {
  Serial.begin(115200); //for debug pruposes, this will print errors and info to the serial port

  waterPumpDelay.start(30000);      // pump 30 sec interval
  mqttDelay.start(5000);            // mqtt 5 sec send delay

  //initialize ethernet parameters
  ethClientSSL.setMutualAuthParams(mTLS);
  Ethernet.begin(mac, ip, myDns, gateway, subnet);

  // set pinmodes assignments for input and output
  pinMode(Float_Switch_Low, INPUT_PULLUP);
  pinMode(Float_Switch_High, INPUT_PULLUP);
  pinMode(Contact_less_sensor,INPUT_PULLUP);
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

  // initialize sensors and probes
  ph.begin();           //PH sensor
  ec.begin();           //EC sensor
  dht.begin();          //hum&temp sensor
  probeSensor.begin();  //probe sensor

  //Check UV/IR/Vis light sensor readiness for I2C
  if (! uv.begin(&Wire1)) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
  //set mqtt callback funtion (the part where "do this" when arduino receives mqtt from subscription)
  mqttClient.setCallback(callback);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  if(initialized){
    light_check(infrared_light());                //actuate growlight depending on light condition
    run_fan(air_humidity(), air_temperature());   //actuate fan depending on temp or humidity

    if(ECPHFirstRunDelay.justFinished()){
      ph_ec_actuationRun();
    }

    //20min check for ph level and add buffers acordingly
    if(phRoutineDelay.justFinished()){
      phCheck();
      phRoutineDelay.repeat();
    }
    //20min check for ec level and add nutrients acordingly
    if(ecRoutineDelay.justFinished()){
      ecCheck();
      ecRoutineDelay.repeat();
    }
    
    // char buffer container for json data to be sent
    char buffer[256];
    if(mqttDelay.justFinished()){
      //sensor data json
      StaticJsonDocument<32> sensor_data;
      StaticJsonDocument<64> probe_data;
      //sensor data being deserialized to json
      sensor_data["air_humidity"] = air_humidity();
      sensor_data["air_temperature"] = air_temperature();
      sensor_data["contactless_liquid_level"] = contactless_liquid_level();
      sensor_data["reservoir_level"] = reservoir_level();
      probe_data["water_temperature"] = water_temperature();
      probe_data["tds_reading"] = tds_reading();
      probe_data["ec_reading"] = ec_reading();
      probe_data["ph_reading"] = ph_reading();
      // serialize data and publish data
      size_t sens_dat = serializeJson(sensor_data, buffer);
      mqttClient.publish(topic_sensor_data, buffer, sens_dat);
      size_t probe_dat = serializeJson(probe_data, buffer);
      mqttClient.publish(topic_probe_data, buffer, probe_dat);
      mqttDelay.repeat();
    }

    // water sprinkler actuation timed for 30sec off and on
    if(waterPumpDelay.justFinished()){
      waterPumpActuate();
      waterPumpDelay.repeat();
    }
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