// ESP32 Fenstersensor, angelehnt an das Projekt Türsensor aus der c't / Merlin Schumacher

// Aktueller Stand vom 2018-08-16

//Define DEBUG to get the Output from DEBUG_PRINTLN
#define DEBUG 0

// BME280 Sensor ist angeschlossen
//#define BME 

// SHT11 Sensor ist angeschlossen
#define sht11

//-------- Benoetigte Librarys -------------
#include <Wire.h>
#include <Basecamp.hpp>


#ifdef BME
  #include "Adafruit_BME280.h"
#endif

#ifdef sht11
  #include <SHT1x.h>
#endif
// ----------------------------------------


// ----- Allgemeine Deklarationen ---------
static const int SensorPin = 33; // Pin fuer den Reed-Kontakt
static const int BatteryPin = 34; // Pin fuer den Check der Batterie

float feuchte = 0;
float druck = 0;
float temp = 0;

float temp_c;
float temp_f;
float humidity;

bool status; // allgemeiner Status
int sensorValue = 0; // Sensorwert Batteriepin
uint16_t statusPacketIdSub = 0;

int batteryLimit = 3300; //batteryLimit Endpunkt an dem eine Batterie als leer angesehen wird.

bool delaySleep = false; //Über eine MQTT Nachricht kann der DeepSleep des ESP32 verhindert werden


#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  120        /* Time ESP32 will go to sleep (in seconds) */
// -----------------------------------------

// ----- MQTT Topics ---------
String geraet = "Badfenster";
String mytopic = "/ESP32/DG/";

String delaySleepTopic;
String statusTopic;
String batteryTopic;
String batteryValueTopic;
String temperaturTopic;
String humidityTopic;
String pressureTopic;

// -----------------------------------------

// ---- Definition Bus und benoetigte Adressen ---------  
#ifdef BME
  #define I2C_SDA 27
  #define I2C_SCL 26
  #define SEALEVELPRESSURE_HPA (1013.25)
  #define BME280_ADD 0x77
#endif

#ifdef sht11
  // Initialisierung SHT1x object incl data und clockpin
  #define dataPin  26
  #define clockPin 27
#endif
// --------------------------------------------------------



// ---- WIFI, MQTT, Webserver definieren ---------------------
Basecamp iot;
// --------------------------------------------------------



// ----------------- Sensor initialisieren -----------------
#ifdef BME
  Adafruit_BME280 bme(I2C_SDA, I2C_SCL);
#endif

#ifdef sht11
  SHT1x sht1x(dataPin, clockPin);
#endif
// ---------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(2000); //Zeit zum initialisieren des seriellen Monitors
  DEBUG_PRINTLN("Program Start");
  
  pinMode(SensorPin, INPUT_PULLDOWN);
  pinMode(BatteryPin, INPUT);
  
  //Triggerstatus früh auslesen
  sensorValue = digitalRead(SensorPin);  


// ---- MQTT Nachrichten definieren
delaySleepTopic = mytopic + geraet + "/cmd" + "/delaysleep";
statusTopic = mytopic + geraet + "/stat" + "/status";
batteryTopic = mytopic + geraet + "/stat" + "/battery";
batteryValueTopic = mytopic + geraet + "/stat" + "/batteryvalue";
temperaturTopic = mytopic + geraet + "/stat" + "/temp";
humidityTopic = mytopic + geraet + "/stat" + "/hum";
pressureTopic = mytopic + geraet + "/stat" + "/pressure";
  
  //Init Basecamp
  iot.begin();
  
  //Set up der Callbacks 
  iot.mqtt.onConnect(onMqttConnect);
  iot.mqtt.onPublish(suspendESP);
    
// ----- Pruefen ob ein BME280 Sensor angeschlossen ist, sonst Programm beenden -----
#ifdef BME
  status = bme.begin(BME280_ADD);
  if (!status) {
    DEBUG_PRINTLN("Kein BME280 Sensor angeschlossen");
	statusPacketIdSub = iot.mqtt.publish(statusTopic.c_str(), 1, true, "Kein BME280 Sensor angeschlossen" );
    while (1);
  }
#endif
// ------------------------------------------------------------------------- 



#ifdef BME
  Serial.println("Start incl BME280 Option");
#endif

#ifdef sht11
  Serial.println("Start incl SHT11 Option");
#endif



// ------- Status Türsensor lesen ---------
  sensorValue = digitalRead(SensorPin);  
  DEBUG_PRINTLN("Sensor:  " + sensorValue);

// ----------- Definition wann der Status periodisch gesendet wird --------------
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  DEBUG_PRINTLN("ESP32 wird fuer " + String(TIME_TO_SLEEP) +
  " Sekunden schlafen");
// ------------------------------------------------------------------------------  
 } 
 
 // ---------- Bei MQTT-Server verbindung aufrufen -----------
void onMqttConnect(bool sessionPresent) {
  //Status senden
  transmitStatus();
}


// ----- Hier werden alle Sensordaten erfasst und an den MQTT-Broker gesendet --------
void transmitStatus() {
  if (sensorValue == 0) {
    DEBUG_PRINTLN("Fenster offen");
    
    //Status an MQTT-Broker senden 
    statusPacketIdSub = iot.mqtt.publish(statusTopic.c_str(), 1, true, "open" );
    
    //Configure the wakeup pin to wake if the door is closed
    esp_sleep_enable_ext0_wakeup((gpio_num_t)SensorPin, 1);
    
  } else {
    DEBUG_PRINTLN("Fenster geschlossen");
   //Status an MQTT-Broker senden 
    statusPacketIdSub = iot.mqtt.publish(statusTopic.c_str(), 1, true, "closed" );
    
    //Configure the wakeup pin to wake if the door is open
    esp_sleep_enable_ext0_wakeup((gpio_num_t)SensorPin, 0);
  }

  //------- Analogen Wert der Batterie lesen --------
  sensorValue = analogRead(BatteryPin);
  
  //sensorC stores the battery value as a char
  char sensorC[6];
  
  //convert the sensor value to a string
  sprintf(sensorC, "%04i", sensorValue);
  
  //Batteriestatus an MQTT-Broker senden 
  if (sensorValue < batteryLimit) {
    DEBUG_PRINTLN(sensorValue);
	  DEBUG_PRINTLN(batteryTopic);
    DEBUG_PRINTLN("Batterie leer");
    iot.mqtt.publish(batteryTopic.c_str(), 1, true, "leer" );
  } else {
    DEBUG_PRINTLN(sensorValue);
    DEBUG_PRINTLN("Batterie voll");
    iot.mqtt.publish(batteryTopic.c_str(), 1, true, "voll" );
  }
  DEBUG_PRINTLN("Daten gesendet");

  

  // ------- Luftfeuchtigkeit -------------------
    #ifdef BME
      feuchte = bme.readHumidity();
    #endif

    #ifdef sht11
      feuchte = sht1x.readHumidity();
    #endif
      // ------ Serielle Ausgabe -----------
      DEBUG_PRINT("Humidity = ");
      DEBUG_PRINT(feuchte);
      DEBUG_PRINT(" %");
        // -- Sensorwert Feuchte an MQTT-Broker senden 
        statusPacketIdSub = iot.mqtt.publish(humidityTopic.c_str(), 1, true, String(feuchte).c_str() );
        DEBUG_PRINTLN(" gesendet");


  // ------- Temperatur -------------------
    #ifdef BME
        temp = bme.readTemperature();
    #endif

    #ifdef sht11
        temp = sht1x.readTemperatureC() - 2; //Korrekturfaktor -2 nach Messung 
    #endif

     // ------ Serielle Ausgabe -----------
     DEBUG_PRINT("Temperatur = ");
     DEBUG_PRINT(temp);
     DEBUG_PRINT(" ℃");
        // -- Sensorwert Temp an MQTT-Broker senden 
        statusPacketIdSub = iot.mqtt.publish(temperaturTopic.c_str(), 1, true, String(temp).c_str() );
        DEBUG_PRINTLN(" gesendet");
        

    #ifdef BME        
      // ------- Druck -------------------
      druck = bme.readPressure() / 100.0F;
        // ------ Serielle Ausgabe -----------
        DEBUG_PRINT("Druck = ");
        DEBUG_PRINT(druck);
        DEBUG_PRINT(" hPa");
            // -- Sensorwert Druck an MQTT-Broker senden 
            statusPacketIdSub = iot.mqtt.publish(pressureTopic.c_str(), 1, true, String(druck).c_str() );
            DEBUG_PRINTLN(" gesendet");
    #endif
  
}

void suspendESP(uint16_t packetId) {
  //Check if the published package is the one of the door sensor
  if (packetId == statusPacketIdSub) {
   
    if (delaySleep == true) {
      DEBUG_PRINTLN("Verzoegere Tiefschlaf");
      return;
    }
    DEBUG_PRINTLN("Starte Tiefschlaf via SuspendESP");
    //Vom MQTT-Broker trennen
    //iot.mqtt.disconnect(); // Aktuell auskommentiert, da hier eine Endlosresettschleife erzeugt wird.

    
    //Schicke ESP32 in den Tiefschlaf
    esp_deep_sleep_start();
  }
}

void loop() {
  // Wenn der ESP32 mehr als 35 sec läuft, in den Tiefschlaf senden
  if (millis() > 35000) {
        DEBUG_PRINTLN("Starte Tiefschlaf nach 30s leerlauf");
    esp_deep_sleep_start();
  }
}
