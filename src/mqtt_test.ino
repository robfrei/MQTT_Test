/***************************************************
  Adafruit MQTT Library Ethernet Example

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Alec Moore
  Derived from the code written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <Ethernet.h>
#include <EthernetClient.h>
#include <Dns.h>
#include <Dhcp.h>
#include <key_file.h>

/************************* Ethernet Client Setup *****************************/
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
IPAddress iotIP (10, 0, 1, 7);
IPAddress dnsIP (10, 0, 1, 1);
IPAddress gateway (10, 0, 1, 1);
IPAddress subnet (255, 255, 255, 0);

/************************* Adafruit.io Setup *********************************/

#define AIO_SERVER      "io.adafruit.com"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "rfrei"
//#define AIO_KEY       "AIO_KEY moved to library and added to .gitignore"

/************************* dashboard controls ********************************/

#define LED 2 // digital output pin 2 (D2) - the on off button feed turns this LED on/off
#define PWMOUT 5 // digital output pin 5 (D5) - the slider feed sets the PWM output of this pin
#define PHOTO 0 // analog input pin 23 (A0) - photoresistor
#define TEMP 1 // analog input pin 24 (A1) - TMP36 temperature sensor

/************ Global State (you don't need to change this!) ******************/

//Set up the ethernet client
EthernetClient client;

// Store the MQTT server, client ID, username, and password in flash memory.
// This is required for using the Adafruit MQTT library.
const char MQTT_SERVER[] PROGMEM    = AIO_SERVER;
// Set a unique MQTT client ID using the AIO key + the date and time the sketch
// was compiled (so this should be unique across multiple devices for a user,
// alternatively you can manually set this to a GUID or other random value).
const char MQTT_CLIENTID[] PROGMEM  = __TIME__ AIO_USERNAME;
const char MQTT_USERNAME[] PROGMEM  = AIO_USERNAME;
const char MQTT_PASSWORD[] PROGMEM  = AIO_KEY;

Adafruit_MQTT_Client mqtt(&client, MQTT_SERVER, AIO_SERVERPORT, MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD);

#define halt(s) { Serial.println(F( s )); while(1);  }


/****************************** Feeds ***************************************/

// Setup a feed called 'photocell' for publishing.
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname>
const char PHOTOCELL_FEED[] PROGMEM = AIO_USERNAME "/feeds/photocell";
//Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, PHOTOCELL_FEED);
Adafruit_MQTT_Publish photocell = Adafruit_MQTT_Publish(&mqtt, PHOTOCELL_FEED, MQTT_QOS_1);
const char TEMPSENSOR_FEED[] PROGMEM = AIO_USERNAME "/feeds/tempsensor";
Adafruit_MQTT_Publish tempsensor = Adafruit_MQTT_Publish(&mqtt, TEMPSENSOR_FEED, MQTT_QOS_1);

// Setup a feed called 'onoff' for subscribing to changes.
const char ONOFF_FEED[] PROGMEM = AIO_USERNAME "/feeds/onoff";
Adafruit_MQTT_Subscribe onoffbutton = Adafruit_MQTT_Subscribe(&mqtt, ONOFF_FEED);
const char SLIDER_FEED[] PROGMEM = AIO_USERNAME "/feeds/slider";
Adafruit_MQTT_Subscribe slider = Adafruit_MQTT_Subscribe(&mqtt, SLIDER_FEED);

/*************************** Sketch Code ************************************/

void setup() {

  pinMode(LED, OUTPUT);
  pinMode(PWMOUT, OUTPUT);

  Serial.begin(115200);

  Serial.println(F("Adafruit MQTT demo"));

  // Initialise the Client
  Serial.print(F("\nInit the Client..."));
  Ethernet.begin(mac, iotIP, dnsIP, gateway, subnet);
  delay(1000); //give the ethernet a second to initialize

  // subscriptions
  mqtt.subscribe(&onoffbutton);
  mqtt.subscribe(&slider);
}

//uint32_t x = 0;
uint32_t ADC0 = 0;
uint32_t ADC1 = 0;
float volts = 0.0;
float degC = 0.0;
float units = 0.0;

void loop() {

  // Ensure the connection to the MQTT server is alive (this will make the first
  // connection and automatically reconnect when disconnected).  See the MQTT_connect
  // function definition further below.
  MQTT_connect();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(3000))) {

    // Check if its the onoff button feed
    if (subscription == &onoffbutton) {
      Serial.print(F("LED #1 Switch: "));
      Serial.println((char *)onoffbutton.lastread);

      if (strcmp((char *)onoffbutton.lastread, "ON") == 0) {
        digitalWrite(LED, HIGH);
      }
      if (strcmp((char *)onoffbutton.lastread, "OFF") == 0) {
        digitalWrite(LED, LOW);
      }
    }

    // check if its the slider feed
    if (subscription == &slider) {
      Serial.print(F("LED #2 Slider: "));
      Serial.println((char *)slider.lastread);
      uint16_t sliderval = atoi((char *)slider.lastread); // convert to a number
      analogWrite(PWMOUT, sliderval);
    }
  }

  // Now we can publish stuff!
  ADC0 = analogRead(PHOTO);
  Serial.print(F("\nSending photocell value "));
  Serial.print(ADC0);
  Serial.print(" ... ");
  if (! photocell.publish(ADC0)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // publish temperature sensor data
  ADC1 = analogRead(TEMP);
  volts = ADC1 * .0049; // convert analog units to degrees C
  degC = (volts * 108.7) - 59.78 + 5; // convert voltage to degrees C
  Serial.print(F("\nSending temperature value "));
  Serial.print(degC);
  Serial.print(" ... ");
  if (! tempsensor.publish(degC)) {
    Serial.println(F("Failed"));
  } else {
    Serial.println(F("OK!"));
  }

  // ping the server to keep the mqtt connection alive
  if(! mqtt.ping()) {
    mqtt.disconnect();
  }

}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}
