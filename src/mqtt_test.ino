#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <key_file.h>

byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
IPAddress iotIP (10, 0, 1, 7);
IPAddress dnsIP (10, 0, 1, 1);
IPAddress gateway (10, 0, 1, 1);
IPAddress subnet (255, 255, 255, 0);

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "rfrei"
//#define AIO_KEY       "AIO_KEY moved to library and added to .gitignore"

#define LED 8 // digital output pin 2 (D2) - the on off button feed turns this LED on/off
#define PWMOUT 6 // digital output pin 5 (D5) - the slider feed sets the PWM output of this pin
#define PHOTO 0 // analog input pin 23 (A0) - photoresistor
#define TEMP 1 // analog input pin 24 (A1) - TMP36 temperature sensor
#define BUTTON 7 // digital input pin 6 (D4) - switch toggling relay on zigbee network

//Set up the ethernet client
EthernetClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
const char MQTT_SERVER[] PROGMEM = AIO_SERVER;

// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM = __TIME__ AIO_USERNAME;
const char MQTT_USERNAME[] PROGMEM = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM = AIO_KEY;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

#define halt(s) { Serial.println(F( s )); while(1);  }

// Setup feeds for publishing
const char PHOTOCELL_FEED[] PROGMEM = AIO_USERNAME "/feeds/photocell";
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, PHOTOCELL_FEED, MQTT_QOS_1);
const char TEMPSENSOR_FEED[] PROGMEM = AIO_USERNAME "/feeds/tempsensor";
Adafruit_MQTT_Publish tempsensor = Adafruit_MQTT_Publish(&mqtt, TEMPSENSOR_FEED, MQTT_QOS_1);

// Setup a feeds for subscriptions
const char ONOFF_FEED[] PROGMEM = AIO_USERNAME "/feeds/onoff";
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED);
const char SLIDER_FEED[] PROGMEM = AIO_USERNAME "/feeds/slider";
Adafruit_MQTT_Subscribe slider = Adafruit_MQTT_Subscribe(&mqtt, SLIDER_FEED);
const char RELAY_FEED[] PROGMEM = AIO_USERNAME "/feeds/relay";
Adafruit_MQTT_Subscribe relay = Adafruit_MQTT_Subscribe(&mqtt, RELAY_FEED);

uint32_t ADC0 = 0;
uint32_t ADC1 = 0;
float volts = 0.0;
float degC = 0.0;
float units = 0.0;
int remoteActuator = false;
int lastRemoteActuator = false;
unsigned long lastSent = 0;

void setup() {

  pinMode(LED, OUTPUT);
  pinMode(PWMOUT, OUTPUT);

  Serial.begin(115200);
  Serial1.begin(9600);

  Serial.println(F("\nMQTT Exploratory Dashboard"));

  // Initialise the Client
  Serial.print(F("\nInitiate the Ethernet Client ..."));
  Ethernet.begin(mac, iotIP, dnsIP, gateway, subnet);
  delay(1000); //give the ethernet a second to initialize

  // subscriptions
  mqtt.subscribe(&onoffbutton);
  mqtt.subscribe(&slider);
  mqtt.subscribe(&relay);
}

void loop() {

  MQTT_connect(); // make and maintain connection to MQTT server

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {

    // Check if its the onoff button feed
    if (subscription == &onoffbutton) {
      Serial.print(F("\nLED Switch: "));
      Serial.println((char *)onoffbutton.lastread);
      if (strcmp((char *)onoffbutton.lastread, "ON") == 0) {
        digitalWrite(LED, HIGH);
      }
      if (strcmp((char *)onoffbutton.lastread, "OFF") == 0) {
        digitalWrite(LED, LOW);
      }
    }

    // check if its the relay feed
    if (subscription == &relay) {
      Serial.print(F("\nRelay: "));
      Serial.println((char *)relay.lastread);
      if (strcmp((char *)relay.lastread, "ON") == 0) {
        setRemoteState((byte)0x04);
        lastRemoteActuator = remoteActuator;
        remoteActuator = true;
      }
      if (strcmp((char *)relay.lastread, "OFF") == 0) {
        setRemoteState((byte)0x05);
        lastRemoteActuator = remoteActuator;
        remoteActuator = false;
      }
    }

    // check if its the slider feed
    if (subscription == &slider) {
      Serial.print(F("\nLED PWM: "));
      Serial.println((char *)slider.lastread);
      uint16_t sliderval = atoi((char *)slider.lastread); // convert to a number
      analogWrite(PWMOUT, sliderval);
    }
  }

  // publish photoresistor data (analog)
  ADC0 = analogRead(PHOTO);
  Serial.print(F("\nPhotocell: "));
  Serial.print(ADC0);
  Serial.print(" ... ");
  if (!photocell.publish(ADC0)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK"));
  }

  // publish temperature sensor data (analog)
  ADC1 = analogRead(TEMP);
  volts = ADC1 * .0049; // convert analog units to degrees C
  degC = (volts * 108.7) - 59.78 + 5; // convert voltage to degrees C
  Serial.print(F("\nTemperature: "));
  Serial.print(degC);
  Serial.print(" ... ");
  if (!tempsensor.publish(degC)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // hardware button press
  if (digitalRead(BUTTON) == HIGH) {
    if (remoteActuator == true) {
      Serial.print(F("\nRelay Hardware Switch: OFF\n"));
      setRemoteState((byte)0x05);
      lastRemoteActuator = remoteActuator;
      remoteActuator = false;
    }
    else if (remoteActuator == false) {
      Serial.print(F("\nRelay Hardware Switch: ON\n"));
      setRemoteState((byte)0x04);
      lastRemoteActuator = remoteActuator;
      remoteActuator = true;
    }
  }

  // ping the server to keep the mqtt connection alive
  if(!mqtt.ping()) {
    mqtt.disconnect();
  }

  // reset the actuator occasionally in case its out of sync
  if (millis() - lastSent > 10000) {
    if (remoteActuator == true) {
      Serial.print(F("\nRelay Sync: ON\n"));
      setRemoteState((byte)0x04);
    }
    if (remoteActuator == false) {
      Serial.print(F("\nRelay Sync: OFF\n"));
      setRemoteState((byte)0x05);
    }
    lastSent = millis();
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print(" Connecting to MQTT ... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

// build and pass the message from the coordinator to the router
// pass either a 0x4 or a 0x5 to turn the actuator (transistor) on/off
void setRemoteState(int value) {
  Serial1.write((byte)0x7E);   // start byte
  Serial1.write((byte)0x00);   // high part of length (always zero)
  Serial1.write((byte)0x10);   // low part of length (exclude checksum)
  Serial1.write((byte)0x17);   // remote AT command
  Serial1.write((byte)0x00);   // frame ID set to zero for no reply
  Serial1.write((byte)0x00);
  Serial1.write((byte)0x13);
  Serial1.write((byte)0xA2);
  Serial1.write((byte)0x00);
  Serial1.write((byte)0x40);
  Serial1.write((byte)0xC1);
  Serial1.write((byte)0xAF);   // 0xFF for broadcast
  Serial1.write((byte)0x5A);   // 0xFF for broadcast
  Serial1.write((byte)0xFF);   // 16-bit of recipiet, or use 0xFFFE if unknown
  Serial1.write((byte)0xFE);
  Serial1.write((byte)0x02);   // 0x02 to apply changes immediately on remote
  Serial1.write((byte)0x44);   // AT ommand in ASCII
  Serial1.write((byte)0x31);
  Serial1.write(value);        // AT command parameter

  // checksum is all bytes after length bytes
  long sum = 0x17 + 0x13 + 0xA2 + 0x40 + 0xC1 + 0xAF + 0x5A + 0xFF + 0xFE + 0x02 + 0x44 + 0x31 + value;
  Serial1.write(0xFF - (sum & 0xFF)); // write checksum
  delay(10); // pause to avoid overwhelming the serial port
}
