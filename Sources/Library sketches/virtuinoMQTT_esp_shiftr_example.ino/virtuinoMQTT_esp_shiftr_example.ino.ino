//---------------------------------------------------------------
// This example uses an ESP8266 or ESP32 Board to connect to shiftr.io.
// Modified by Ilias Lamprou at sep/25/2018
// You can check on your device after a successful connection here: https://shiftr.io/try.
// Original example by Joël Gähwiler https://github.com/256dpi/arduino-mqtt
//---------------------------------------------------------------

#include <ESP8266WiFi.h>  // ESP8266
//#include <WiFi.h>       // for ESP32 enable this line and disable the previous
#include <MQTTClient.h>

//------ wifi settings
char ssid[] = "WIFI NETWORK"; //  Change this to your network SSID (name).
char pass[] = "PASSWORD";  // Change this your network password

//------ MQTT broker settings and topics
const char* broker = "broker.shiftr.io"; 
char mqttUserName[] = "USERSNAME";         
char mqttPass[] = "PASSWORD";               

const char* topic_pub_temperature = "temperature"; 
const char* topic_pub_humidity = "humidity"; 
const char* topic_sub_motor_command = "motor_command"; 


WiFiClient net;
MQTTClient client;
unsigned long lastMillis = 0;

//========================================= connect
void connect() {
  Serial.print("\nconnecting to wifi.");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(5000);
  }
  //--- create a random client id
  char clientID[] ="ESP3200000000000000";  // For random generation of client ID.
  for (int i = 5; i <18 ; i++) clientID[i]=  char(48+random(10));
    
  Serial.print("\nconnecting to broker...");
  while (!client.connect(clientID,mqttUserName,mqttPass)) {
    Serial.print(".");
    delay(5000);
  }
  Serial.println("\nconnected!");
  client.subscribe(topic_sub_motor_command);
  // client.unsubscribe("motor_command");
}

//========================================= messageReceived
void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  /*if (topic==topic_sub_motor_command){
    int v = payload.toInt();
    if (v==1) digitalWrite(D4,HIGH);
    else digitalWrite(D4,LOW);
  }
  */
}

//========================================= setup
//=========================================
void setup() {
  Serial.begin(9600);
  WiFi.begin(ssid, pass); // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported by Arduino. You need to set the IP address directly.
  client.begin(broker, net);
  //pinMode(D4,OUTPUT);
  client.onMessage(messageReceived);
  connect();
}
//========================================= loop
//=========================================
void loop() {
  client.loop();
  delay(10);  // <- fixes some issues with WiFi stability
  if (!client.connected())connect();
 
  if (millis() - lastMillis > 5000) {
    int sensorValue_1=random(100);  // replace with your sensor value
    int sensorValue_2=random(100);  // replace with your sensor value
    client.publish(topic_pub_temperature, String(sensorValue_1),true,1);
    client.publish(topic_pub_humidity, String(sensorValue_1),true,1);
    lastMillis = millis();
  }
}
