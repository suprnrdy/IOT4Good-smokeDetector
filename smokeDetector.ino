#include "LoRaWAN.h"
#include <TinyGPS++.h>
#include <CayenneLPP.h>
#include <MQUnifiedsensor.h>


// LoraWAN Defines
const char *devEui = "FILL_ME_IN";
const char *appEui = "FILL_ME_IN";
const char *appKey = "FILL_ME_IN";
// Depending on the LoRa frequency plan and data rate used, the maximum payload varies. 
// It's safe to send up to 51 bytes of payload.
CayenneLPP lpp(51);

// GPS Defines
TinyGPSPlus gps;
DynamicJsonDocument jsonBuffer(4096);
static const uint32_t GPSBaud = 4800;
double dev_lat = 0.0, dev_lon = 0.0;
bool first_lock = true;

// MQ-135 Defines
//Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 3.6
#define analog_pin A2 // B-L072Z-LRWAN1 has only 3 Analog pins exposed, so use A2 in arduino library == PA_4 on board
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//Declare Sensor
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, analog_pin, type);

float CO = 0.0;
float Alcohol = 0.0;
float CO2 = 0.0;
float Tolueno = 0.0;
float NH4 = 0.0;
float Acetona = 0.0;

// Timers
static const unsigned long REFRESH_INTERVAL = 5000; // ms
static unsigned long lastRefreshTime = 0;
static unsigned long lostGPSTime = 0;
static unsigned long dataTimer = 0;

void setup( void )
{
    Serial.begin(9600);
    Serial1.begin(GPSBaud);
    
    while (!Serial) { }

    // Setup MQ-135
    //Set math model to calculate the PPM concentration and the value of constants
    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ135.init(); 
    MQ135.setRL(1);
    MQ135.setR0(45.5);
  
    // US Region
    LoRaWAN.begin(US915);
    // Helium SubBand
    LoRaWAN.setSubBand(2);
    // Disable Adaptive Data Rate
    LoRaWAN.setADR(false);
    // Set Data Rate 1 - Max Payload 53 Bytes
    LoRaWAN.setDataRate(1);
    // Device IDs and Key
    LoRaWAN.joinOTAA(appEui, appKey, devEui);

    Serial.println("JOIN( )");
}

void getAirData() {
  // Collect data from Air Quality Sensor
  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
  MQ135.setA(605.18); MQ135.setB(-3.937); // Configurate the ecuation values to get CO concentration
  CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ135.setA(77.255); MQ135.setB(-3.18); // Configurate the ecuation values to get Alcohol concentration
  Alcohol = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ135.setA(110.47); MQ135.setB(-2.862); // Configurate the ecuation values to get CO2 concentration
  CO2 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ135.setA(44.947); MQ135.setB(-3.445); // Configurate the ecuation values to get Tolueno concentration
  Tolueno = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ135.setA(102.2 ); MQ135.setB(-2.473); // Configurate the ecuation values to get NH4 concentration
  NH4 = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  MQ135.setA(34.668); MQ135.setB(-3.369); // Configurate the ecuation values to get Acetona concentration
  Acetona = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
}

void sendPacket() {
  if (LoRaWAN.joined() && !LoRaWAN.busy())
  {
    if(lpp.getSize() > 0) {
      Serial.println("sendPacket");
      Serial.println("setting up packet");
       
      Serial.print("TRANSMIT( ");
      Serial.print("TimeOnAir: ");
      Serial.print(LoRaWAN.getTimeOnAir());
      Serial.print(", MaxPayloadSize: ");
      Serial.print(LoRaWAN.getMaxPayloadSize());
      Serial.print(", DR: ");
      Serial.print(LoRaWAN.getDataRate());
      Serial.print("dbm, UpLinkCounter: ");
      Serial.print(LoRaWAN.getUpLinkCounter());
      Serial.print(", RSSI: ");
      Serial.print(LoRaWAN.lastRSSI());
      Serial.println(" )");
//      Serial.println(lpp.getSize());
      // Send Packet
      LoRaWAN.sendPacket(1, lpp.getBuffer(), lpp.getSize(), true);  
    }
  }
}

void airqualityMessage() {
  getAirData();
  // Send air quality data every x minutes
  // Repackage air quality data
  lpp.addAnalogInput(3, CO);
  lpp.addAnalogInput(4, Alcohol);
  lpp.addAnalogInput(5, CO2);
  lpp.addAnalogInput(6, Tolueno);
  lpp.addAnalogInput(7, NH4);
  lpp.addAnalogInput(8, Acetona);
  
  Serial.print("CO: ");
  Serial.print(CO);
  Serial.print(", Alcohol: ");
  Serial.print(Alcohol);
  Serial.print(", CO2: ");
  Serial.print(CO2);
  Serial.print(", Tolueno: ");
  Serial.print(Tolueno);
  Serial.print(", NH4: ");
  Serial.print(NH4);
  Serial.print(", Acetona: ");
  Serial.println(Acetona);
}

void gpsMessage() {
  if (gps.location.isValid())
  { 
    dev_lat = gps.location.lat();
    dev_lon = gps.location.lng();
    lpp.addGPS(1, gps.location.lat(), gps.location.lng(), gps.altitude.meters());
  }
}
  
void loop( void )
{
  while (Serial1.available() > 0)
  {
    // Gather our GPS data and encode it
    gps.encode(Serial1.read());
  }

  lpp.reset();
  // This is our heartbeat that sends us the RSSI from acked messages (probably don't need this, but nice to have right now while DC are free)
  if(millis() - lastRefreshTime > 600000) { //1m = 60000, 1hr = 36000000, 10m = 600000
    lastRefreshTime = millis();
    lpp.addAnalogInput(2, LoRaWAN.lastRSSI());
    // Build packet here, setup LPP Message
  }
  
  //get and send air quality data every x seconds
  if(millis() - dataTimer > 600000) { //1m = 60000, 1hr = 36000000, 10m = 600000
    dataTimer = millis();
    airqualityMessage();
  }
  //get and send gps data once, then every x minutes if it changes. if it stops moving, don't send any updated packets. 
  // If this is the first time to get a GPS Fix, lets set this location
  if(first_lock) {
    gpsMessage();
    first_lock = false;
  } else {
    // We check if we've moved more than 10 m
    double changed_distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), dev_lat, dev_lon);
    if(changed_distance > 10) {
      // Update coordinates
      gpsMessage();
      // Report every minute/hour/etc....
      if(millis() - lostGPSTime > 1200000) { //1m = 60000, 1hr = 36000000, 20m = 1200000
        lostGPSTime = millis();
        lpp.addGPS(1, gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      }
    }
  }
  
  // complete our packets here, and send. 
  sendPacket();
  
}
 
