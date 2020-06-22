#include "LoRaWAN.h"
#include <TinyGPS++.h>
#include <CayenneLPP.h>
#include <MQUnifiedsensor.h>


// LoraWAN Defines
const char *devEui = "";
const char *appEui = "";
const char *appKey = "";
// Depending on the LoRa frequency plan and data rate used, the maximum payload varies. 
// It's safe to send up to 51 bytes of payload.
CayenneLPP lpp(51);

// GPS Defines
TinyGPSPlus gps;
DynamicJsonDocument jsonBuffer(4096);
static const uint32_t GPSBaud = 4800;
// A sample NMEA stream.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// MQ-135 Defines
//Definitions
#define placa "Arduino UNO"
#define Voltage_Resolution 3.5
#define analog_pin A2 //Analog input 0 of your arduino
#define type "MQ-135" //MQ135
#define ADC_Bit_Resolution 12 // For arduino UNO/MEGA/NANO
#define RatioMQ135CleanAir 3.6//RS / R0 = 3.6 ppm  
//Declare Sensor
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, analog_pin, type);


void setup( void )
{
    Serial.begin(9600);
    Serial1.begin(GPSBaud);
    
    while (!Serial) { }

    // Setup MQ-135
    //Set math model to calculate the PPM concentration and the value of constants
    MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ135.setA(605.18); MQ135.setB(-3.937); // Configurate the ecuation values to get CO concentration
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

float CO = 0.0;

void sendPacket() {
  if (LoRaWAN.joined() && !LoRaWAN.busy())
  {
    Serial.println("setting up packet");
    lpp.reset();
    CO = MQ135.readSensor(); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
    lpp.addAnalogInput(3, LoRaWAN.lastRSSI());
    lpp.addAnalogInput(2, CO);
    // Setup LPP Message
    if (gps.location.isValid())
    { 
      Serial.print(gps.location.lat(), 6);
      Serial.print(F(","));
      Serial.println(gps.location.lng(), 6);
      lpp.addGPS(1, gps.location.lat(), gps.location.lng(), gps.altitude.meters());
      JsonArray root = jsonBuffer.to<JsonArray>();
      lpp.decode(lpp.getBuffer(), lpp.getSize(), root);
      serializeJsonPretty(root, Serial);
      Serial.println();
    } 
    
    Serial.print("TRANSMIT( ");
    Serial.print("TimeOnAir: ");
    Serial.print(LoRaWAN.getTimeOnAir());
    Serial.print(", NextTxTime: ");
    Serial.print(LoRaWAN.getNextTxTime());
    Serial.print(", MaxPayloadSize: ");
    Serial.print(LoRaWAN.getMaxPayloadSize());
    Serial.print(", DR: ");
    Serial.print(LoRaWAN.getDataRate());
    Serial.print(", TxPower: ");
    Serial.print(LoRaWAN.getTxPower(), 1);
    Serial.print("dbm, UpLinkCounter: ");
    Serial.print(LoRaWAN.getUpLinkCounter());
    Serial.print(", DownLinkCounter: ");
    Serial.print(LoRaWAN.getDownLinkCounter());
    Serial.print(", RSSI: ");
    Serial.print(LoRaWAN.lastRSSI());
    Serial.print(", CO: ");
    Serial.print(CO);
    
    Serial.println(" )");

    // Send Packet
    LoRaWAN.sendPacket(1, lpp.getBuffer(), lpp.getSize(), true);
  }
}

static const unsigned long REFRESH_INTERVAL = 5000; // ms
static unsigned long lastRefreshTime = 0;

void loop( void )
{
  while (Serial1.available() > 0)
  {
    gps.encode(Serial1.read());
  }

  
  if(millis() - lastRefreshTime == REFRESH_INTERVAL) {
    lastRefreshTime += REFRESH_INTERVAL;
    MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
    sendPacket();
  } 
}
 
