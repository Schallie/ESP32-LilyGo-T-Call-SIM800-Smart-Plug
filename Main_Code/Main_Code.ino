//---Defines, Includes, & Misc.----------------------------
//Custom
#define buttonPin 35
#define redLED 32
#define greenLED 33
#define blueLED 25
#define relayPin 14
#define sensePin 12
#define ACPin 13
// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800      // Modem is SIM800
#define TINY_GSM_RX_BUFFER   1024  // Set RX buffer to 1Kb
// TTGO T-Call pins
#define MODEM_RST            5
#define MODEM_PWKEY          4
#define MODEM_POWER_ON       23
#define MODEM_TX             27
#define MODEM_RX             26
#define I2C_SDA              21
#define I2C_SCL              22
// Set serial for debug console (to Serial Monitor, default speed 115200)
#define SerialMon Serial
// Set serial for AT commands (to SIM800 module)
#define SerialAT Serial1
//#define DUMP_AT_COMMANDS  //enables sim800 communication to serial monitor
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT); //starts modem from serial
#endif
//TinyGsmClient gsmClient(modem); // TinyGSM Client for Internet connection
#define uS_TO_S_FACTOR        1000000     /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP         600        /* Time ESP32 will go to sleep (in seconds) for 600 seconds = 10 minutes */
#define IP5306_ADDR           0x75
#define IP5306_REG_SYS_CTL0   0x00

#include <Wire.h>
#include <WiFi.h>
#include <TinyGsmClient.h>
#include <EEPROM.h>
#include <MQTTClient.h>
#include "Virtuino_ESP_WifiServer.h"
//-----------------------------------------------------------------
//---EEPROM SETTINGS------------------------------------------------
#define EEPROM_SIZE = 500
EEPROM.begin(EEPROM_SIZE);
//topic pos     =  0      1     2     3           4         5         6           7         8           9
String[] topic  = {ssid,  pass, apn,  gprsUser,   gprsPass, simPin,   mqttName,   mqttPass, lastState,  smsnum};
int[] addr      = {0,     32,   64,   96,         128,      160,      168,        200,      232,        233};
int[] size      = {32,    32,   32,   32,         32,       8,        32,         32,       1,          10};
//------------------------------------------------------------------
//--- WIFI SETTINGS ------------------------------------------------
const char ssid[]     = readEEPROM(0);            // WIFI network SSID
const char password[] = readEEPROM(1);            // WIFI network PASSWORD
//------------------------------------------------------------------
//---Your GPRS credentials (leave empty, if not needed)-------------
const char apn[]      = readEEPROM(2); //"lte.vodacom.za" APN (example: internet.vodafone.pt) use https://wiki.apnchanger.org
const char gprsUser[] = readEEPROM(3); // GPRS User
const char gprsPass[] = readEEPROM(4); // GPRS Password
const char simPIN[]   = readEEPROM(5);
#define SMS_TARGET    = readEEPROM(9);
//------------------------------------------------------------------
//------ MQTT broker settings and topics----------------------------
const char* broker  = "broker.shiftr.io";
char mqttUserName[] = readEEPROM(6);
char mqttPass[]     = readEEPROM(7);

const char* topic_pub_temperature   =   "temperature";
const char* topic_pub_humidity      =   "humidity";
const char* topic_sub_motor_command =   "motor_command";

MQTTClient client;
char clientID[] = "ESP3200000000000000"; // For random generation of client ID.
for (int i = 5; i < 18 ; i++) clientID[i] =  char(48 + random(10));
//------------------------------------------------------------------

void setup() {
  SerialMon.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);  // Start I2C communication
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);  // Set GSM module baud rate and UART pins

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  //custom pindefs
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(sensePin, INPUT);
  pinMode(ACPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  digitalWrite(relayPin, LOW);


}

void loop() {
  if (!hasAC) noAC;
  else if (hasAC) {
    if (EEPROM.read(232) == 0) {
      sendSMS(1);
      EEPROM.write(232, 1);
    }

  }
}

boolean hasAC() return digitalRead(ACPin);

void noAC() {
  if (analogRead(ACPin) > 500) {
    EEPROM.write(232, 1);
    return false;
  }
  SerialMon.println("no AC detected");
  bool isOk = setPowerBoostKeepOn(1);  // Keep power when running from battery
  SerialMon.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));
  boolean gprsConnected = connect(2);
  if (gprsConnected && analogRead(ACPin) < 500) {
    if (EEPROM.read(232) == 1) {
      sendSMS(0);
      EEPROM.write(232, 0);
    }
    pub();
    sub();
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);  // Configure the wake up source as timer wake up
    esp_deep_sleep_start();  // Put ESP32 into deep sleep mode (with timer wake up)
  }
  else if (!gprsConnected && analogRead(ACPin) < 500) connect(3);
}


boolean sendSMS(String msg) {
  if (modem.isGprsConnected()) {
    return modem.sendSMS(SMS_TARGET, String(msg) + modem.getIMEI());
    else return false;
  }
}

bool setPowerBoostKeepOn(int en) {
  Wire.beginTransmission(IP5306_ADDR);
  Wire.write(IP5306_REG_SYS_CTL0);
  if (en) Wire.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  else Wire.write(0x35); // 0x37 is default reg value
  return Wire.endTransmission() == 0;
}

bool connectToWiFiNetwork() {
  if (EEPROM.read[addr[0]] == 0 || EEPROM.read[addr[1]] == 0) return false;
  SerialMon.println("Connecting to " + String(ssid));
  WiFi.mode(WIFI_STA);      // Config module as station only.
  WiFi.begin(ssid, password);
  for (int i = 0; WiFi.status() != WL_CONNECTED && i < 10; i++) {
    delay(500);
    SerialMon.print(".");
  }
  SerialMon.println("");
  if (WiFi.status() == WL_CONNECTED) {
    SerialMon.println("WiFi connected");
    SerialMon.println(WiFi.localIP());
    return true;
  }
  else if (WiFi.status() != WL_CONNECTED) {
    return false;
  }
}

bool connectToGSMNetwork() {
  modem.init();  //or modem.restart();
  if (strlen(simPIN) && modem.getSimStatus() != 3 ) modem.simUnlock(simPIN); // Unlock your SIM card with a PIN if needed
  SerialMon.print("Connecting to APN: ");
  if (EEPROM.read[addr[2]] == 0 || EEPROM.read[addr[3]] == 0 || EEPROM.read[addr[4]] == 0) return false;
  else if (modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("APN connected");
    return true
  }
}

bool connectSoftAP() {
  WiFi.disconnect();
  WiFi.mode(WIFI_AP);   // Config module as Access point only. Default IP: 192.168.4.1
  boolean result = WiFi.softAP("Smart Plug", "1234");      // SSID, Password
  if (result == true)  {
    Serial.println("Server Ready");
    Serial.println(WiFi.softAPIP());
    return true;
  }
  else Serial.println("Failed!");
  return false;
}

char [] readEEPROM (int pos) {
  char[] temp = char[size[pos]];
  for (int i = addr[pos]; i < (size[i] + addr[pos]); i++) {
    byte readValue = EEPROM.read(i);
    if (readValue == 0) return temp;
    temp[i] = char(readValue);
  }
}

void writeEEPROM (int pos, int tempsize, String str) {
  char[] temp = str;
  for (int i = addr[pos]; i < (tempsize + addr[pos]); i++) EEPROM.write(i, temp[i]);
  EEPROM.write((i + addr[pos]), '\0');
  EEPROM.commit();
}

void connect(int s) {
  if (s == 1)  {//WiFi
    connectToWiFiNetwork();
    client.begin(broker, WiFi);
    Serial.print("\nconnecting to broker...");
    while (!client.connect(clientID, mqttUserName, mqttPass)) {
      Serial.print(".");
      delay(500);
    }
    Serial.println("\nconnected!");
  }
  if (s == 2) {//GSM
    connectToGSMNetwork();
    client.begin(broker, modem);
    Serial.print("\nconnecting to broker...");
    while (!client.connect(clientID, mqttUserName, mqttPass)) {
      Serial.print(".");
      delay(500);
    }
    Serial.println("\nconnected!");
  }
  if (s == 3) {//softAP
    Virtuino_ESP_WifiServer virtuino(&server);
    if (connectSoftAP()) {
      virtuino.password = "1234";
      server.begin;
      while (1) {
        virtuino.run();
        virtuino.vDelay(500);
      }
    }
    else {
      SerialMon.println("SoftAP failed");
    }
  }
}

boolean pub() {
  if (!client.connected())connect();
  client.publish(topic_pub_temperature, String(sensorValue_1), true, 1);
  client.publish(topic_pub_humidity, String(sensorValue_1), true, 1);
}

boolean sub() {
  client.subscribe(topic_sub_motor_command);
}


Still needs:
pubsub
rgbled
