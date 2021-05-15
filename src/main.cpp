//Arduino framework
#include <Arduino.h>
#include "header.h" //Important headers for dependencies
#include "pin_constants.h" //header for pin assignment constants

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

//Ethernet Variable for connecting
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 117);
IPAddress myDns(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT server IP
const char* server = "192.168.1.65";

// Ethernet and MQTT related objects
EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

//sensor data json
DynamicJsonDocument sensor_data(1024);

void setup() {
  Serial.begin(115200);

  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  mqttClient.setServer(server, 1883);

  pinMode(Float_Switch_Low, INPUT_PULLUP);
  pinMode(Float_Switch_High, INPUT_PULLUP);
  pinMode(Contact_less_sensor,INPUT);
  pinMode(RELAY_PIN7, OUTPUT);
  pinMode(RELAY_PIN6, OUTPUT);

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

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("arduinoClient")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      mqttClient.publish("outTopic","hello world");
      // ... and resubscribe
      mqttClient.subscribe("inTopic");
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
  liqsenseor = digitalRead(Contact_less_sensor);

  if (liqsenseor == HIGH) { 
    return true;
  } 
  else { 
    return false;
  }
}

String reservoir_level(){
  
  int floatLow = !digitalRead(Float_Switch_Low);
  int floatHigh = !digitalRead(Float_Switch_High);
  
  if(floatHigh == 1 && floatLow == 0){
    return ("100%");
    }
  else if(floatHigh == 0 && floatLow == 0){
    return ("50%");
    }
  else if(floatHigh == 0 && floatLow == 1){
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
    static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U){                  //time interval: 1s
        timepoint = millis();
        phVoltage = analogRead(PH_PIN)/1024.0*3500;  // read the voltage
        phValue = ph.readPH(phVoltage,temperature);  // convert voltage to pH with temperature compensation
        return (phValue);
    }   
}

float EC_reading(){
  static unsigned long timepoint = millis();
    if(millis()-timepoint>1000U)  //time interval: 1s
    {
      timepoint = millis();
      ecVoltage = analogRead(EC_PIN)/1024.0*3500;   // read the voltage
      ecValue =  ec.readEC(ecVoltage,temperature);  // convert voltage to EC with temperature compensation
      return (ecValue);
    }
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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void loop() {
  light_check(infrared_light());
  run_fan(Air_humidity());
  temperature = readProbeTemperature();
  phValue = PH_reading();
  ecValue = EC_reading();
  tdsValue = TDS_reading();
  liqsenseor = contactless_liquid_level();

  if (!mqttClient.connected()) {
    reconnect();
  }
  mqttClient.loop();

  char buffer[256];
  mqttClient.subscribe("HYD-1/sensor_data");
  mqttClient.subscribe("HYD-1/commands");

  sensor_data["Air_Humidity"] = Air_humidity();
  sensor_data["Air Temperature"] = Air_temperature();
  sensor_data["contactless_liquid_level"] = liqsenseor;
  sensor_data["TDS_reading"] = tdsValue;
  sensor_data["EC_reading"] = ecValue;
  sensor_data["PH_reading"] = phValue;
  sensor_data["water_temp"] = temperature;
  sensor_data["UV_light"] = UV_light();
  sensor_data["infrared_light"] = infrared_light();
  sensor_data["visible_light"] = visible_light();
  sensor_data["reservoir_level"] = reservoir_level();

  // serializeJsonPretty(sensor_data, Serial);
  serializeJson(sensor_data, buffer);
  mqttClient.publish("HYD-1/sensor_data", buffer);

  // temperature = readProbeTemperature();
  // Serial.print("\n============================================\n");
  // Serial.print("Humidity: "); Serial.print(Air_humidity()); Serial.print("%\n");
  // Serial.print("Air Temperature: "); Serial.print(Air_temperature()); Serial.print("*C\n");
  // Serial.print("Contactless: "); Serial.print(contactless_liquid_level());
  // Serial.print("\nTDS: "); Serial.print(TDS_reading()); Serial.print("ppm\n");
  // Serial.print("EC level: "); Serial.print(EC_reading(),2); Serial.print("mS/cm\n");
  // Serial.print("PH level: ");Serial.print(PH_reading(),2);
  // Serial.print("\nWater temperatre: "); Serial.print(temperature); Serial.print("*C\n");
  // Serial.print("UV level: "); Serial.print(UV_light());
  // Serial.print("\nInfrared level: ") ;Serial.print(infrared_light());
  // Serial.print("\nVisible light(lux): "); Serial.print(visible_light());
  // Serial.print("\nReservior level: "); Serial.print(reservior_level());
  delay(5000);
}