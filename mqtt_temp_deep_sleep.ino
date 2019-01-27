/*

Wake up.
Connect to WiFi.
Connect to MQTT.
Read temperature using OneWire protocol from Dallas 18B20 sensor.
Publish temperatur to MQTT channel.
Deep sleep.

*/

#include <ESP8266WiFi.h>
#include <DNSServer.h>            //Local DNS Server used for redirecting all requests to the configuration portal
#include <ESP8266WebServer.h>     //Local WebServer used to serve the configuration portal
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <WiFiUdp.h>              //to set time from NTP server
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <TimeLib.h>

// B1820 digital thermometer uses onewire protocol
#define ONE_WIRE_BUS 13
#define ONE_WIRE_MAX_DEV 15
//#define ANALOG_PIN_0 36
#define BUILTIN_LED 2

// Update these with values suitable for your network.


const char* mqtt_server = "192.168.1.16";
const char* mqtt_user = "jordan";
const char* mqtt_password = "ra66lhk";
const int mqtt_port = 1883;
const int sleep_seconds = 300;

// wifi setup
WiFiClient espClient;
WiFiManager wifiManager;

// mqtt setup
PubSubClient client(espClient);
char clientId[] = "ESP-XXXXXXXXXXXX"; // XX will be replaced by MAC address
char outTopic[] = "ESP-XXXXXXXXXXXX/sensors";
char inTopic[] = "ESP-XXXXXXXXXXXX/received";

// Sensor stuff
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
long lastMsg = 0;
char msg[50];
int value = 0;
ADC_MODE(ADC_VCC);  // for battery votage measurement

// Time stuff:
static const char ntpServerName[] = "us.pool.ntp.org";
//static const char ntpServerName[] = "time.nist.gov";
//static const char ntpServerName[] = "time-a.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-b.timefreq.bldrdoc.gov";
//static const char ntpServerName[] = "time-c.timefreq.bldrdoc.gov";

const int timeZone = 0;     // GMT aka CUT
//const int timeZone = -5;  // Eastern Standard Time (USA)
//const int timeZone = -4;  // Eastern Daylight Time (USA)
//const int timeZone = -8;  // Pacific Standard Time (USA)
//const int timeZone = -7;  // Pacific Daylight Time (USA)

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

WiFiUDP Udp;
unsigned int localPort = 8888;  // local port to listen for UDP packets

time_t getNtpTime();
void sendNTPpacket(IPAddress &address);

//flag for saving data
bool shouldSaveConfig = false;

// this is the layout of persistent memory
uint16_t first_to_write;	// index within array of readings to write next entry
uint16_t first_to_send;		// index within array of next entry to send over network
struct {
  uint32_t timestamp;		// unix epoch time (seconds since 1/1/1970
  int16_t temp;			// temperature in degrees C / 100
  uint16_t millivolts;		// Vcc measurement
} readings[511];		// total persistent storage size is 4096 bytes

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());

  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup_wifi() {
  char wifi_ssid[16] = "";
  char *wifi_password = "0123456789";
  
  delay(10);
  
  sprintf(wifi_ssid, "ESP%06x", ESP.getChipId());
  wifiManager.setAPCallback(configModeCallback);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.autoConnect(wifi_ssid, wifi_password);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup_time() {
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300); // In practice it won't run for 300 seconds, it will re-sync on each esp8266 restart
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

}

//double read_TMP36() {
//  // read the input on analog pin 0:
//  (void)analogRead(A0); // discard first reading because it tends to be inaccurate
//  int sensorValue = analogRead(A0);
//  double Voltage = (sensorValue / 1023.0) * 3200.0; // 3.2V range.
//  double tempC = (Voltage-500.0) * 0.1; // 500 is the offset
//
//  // print out the value you read:
//  Serial.println("Sensor: ");
//  Serial.println(sensorValue);
//  Serial.println("Voltage: ");
//  Serial.println(Voltage);
//  Serial.println("Temp deg C: ");  
//  Serial.println(tempC);  
//
//  return(tempC);
//}

float read_18B20()
{
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("Got 18B20 temperatures");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));  
  return(sensors.getTempCByIndex(0));
}

void reconnect() {
  int rc = 0;
  byte mac[6];
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Client id will be "ESP-" + MAC address
    WiFi.macAddress(mac);
    sprintf(clientId, "ESP-%02x%02x%02x%02x%02x%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    sprintf(outTopic, "%s/sensors", clientId);
    sprintf(inTopic, "%s/received", clientId);
    // Attempt to connect
    Serial.println(clientId);
    Serial.println(mqtt_user);
    Serial.println(mqtt_password);
    if (client.connect(clientId, mqtt_user, mqtt_password)) {
      Serial.println("connected");
      Serial.println(client.connected());
      Serial.println("client.state(): ");
      Serial.println(client.state());
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}

void setup() {
  char msg[32];
  pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  setup_time();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqtt_callback);
  reconnect();
  client.subscribe(inTopic);
  // One Wire Dallas
  sensors.begin();
  sprintf(msg, "{\"time\": %d,\n\"temp\": %f,\n\"vcc\": %f}", now(), read_18B20(), ESP.getVcc()/1000.0);
  client.publish(outTopic, msg);
  delay(3000);
  ESP.deepSleep(sleep_seconds * 1000000);
}

void loop() {

//  client.setCallback(callback);
//  if (!client.connected()) {
//    reconnect();
//  }
//  client.loop();
//
//  long now = millis();
//  if (now - lastMsg > 2000) {
//    lastMsg = now;
//    ++value;
//    snprintf (msg, 50, "hello world #%ld", value);
//    Serial.print("Publish message: ");
//    Serial.println(msg);
//    client.publish("outTopic", msg);
//  }
}
