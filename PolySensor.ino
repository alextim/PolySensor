#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

#include <DallasTemperature.h>
#include <OneWire.h>

#include "printf.h"

#include <my_sensor.h>
  
#include <Wire.h>
#include <Adafruit_BMP085.h>

#include <DHT.h>  
#define HUMIDITY_SENSOR_DIGITAL_PIN 4

#define ABOVE_SEALEVEL 100
Adafruit_BMP085 bmp = Adafruit_BMP085(); 
float lastPressure_bmp = UNDEFINDED_VALUE;
float lastTemp_bmp = UNDEFINDED_VALUE;
//bool bmp_started = false;

DHT dht;
float lastTemp_dht = UNDEFINDED_VALUE;
float lastHum_dht = UNDEFINDED_VALUE;
//bool dht_started = false;

#define ONE_WIRE_BUS 3 // Pin where dallase sensor is connected 
#define TEMPERATURE_PRECISION 10
#define MAX_ATTACHED_DS18B20 16

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
//bool dallas_started = false;

void printAddress(DeviceAddress deviceAddress, bool bHex=false);
struct my_thermometer {
	uint8_t sensor;
	DeviceAddress address;
	float lastTemp;
	bool found;
};

my_thermometer thermometerSensor[] = {
  { S_T_OUTSIDE,      {0x28, 0xFF, 0x8E, 0xC5, 0x53, 0x14, 0x01, 0x8F}, UNDEFINDED_VALUE, false },
  { S_T_INDOOR,       {0x28, 0xFF, 0x15, 0xF5, 0x53, 0x14, 0x01, 0x00}, UNDEFINDED_VALUE, false },
  { S_T_BOILER_OUT,   {0x28, 0xF5, 0x65, 0xD5, 0x58, 0x19, 0x06, 0xD8}, UNDEFINDED_VALUE, false },
  { S_T_BOILER_RET,   {0x28, 0xF4, 0x64, 0xD4, 0x57, 0x18, 0x05, 0xD7}, UNDEFINDED_VALUE, false },  
  { S_T_RADIATOR_INP, {0x28, 0xF3, 0x63, 0xD3, 0x56, 0x17, 0x04, 0xD6}, UNDEFINDED_VALUE, false },   
  { S_T_RADIATOR_RET, {0x28, 0x3F, 0x1C, 0x31, 0x02, 0x00, 0x03, 0x02}, UNDEFINDED_VALUE, false },
  { S_T_FLOOR_INP,    {0x28, 0xF1, 0x62, 0xD8, 0x53, 0x14, 0x02, 0xD1}, UNDEFINDED_VALUE, false },
  { S_T_FLOOR_RET,    {0x28, 0xFF, 0x0D, 0xC3, 0x53, 0x14, 0x01, 0x03}, UNDEFINDED_VALUE, false },
};
#define TEMP_SENSOR_COUNT sizeof(thermometerSensor)/sizeof(my_thermometer)

boolean isMetric = true; 

my_msg msg;
my_sensor radio;

const int sendPeriod = 10;
int mNbrSendTicks = 0;

void getDHTdata(bool bSend = false);
void getBMPdata(bool bSend = false);
void getTempSensorData(bool bSend = false);

void onewire_begin() {
  sensors.begin();
  
  discoverOneWire();
  
  Serial.println("Init temp sensors...");

  for(uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++) {
	printf("Sensor %i of %i, %s", i, TEMP_SENSOR_COUNT, my_msg::getSensorName(thermometerSensor[i].sensor));
	if (!sensors.validAddress(thermometerSensor[i].address))
	  printf(" - Not valid address!\n");
    else if (sensors.isConnected(thermometerSensor[i].address)) {
	  int prec = sensors.getResolution(thermometerSensor[i].address);
	  if (prec != TEMPERATURE_PRECISION) {
		sensors.setResolution(thermometerSensor[i].address, TEMPERATURE_PRECISION);
		prec = sensors.getResolution(thermometerSensor[i].address);
	  }
      printf(", Resolution: %i\n", prec);
	  thermometerSensor[i].found = true;
	  if (OneWire::crc8( thermometerSensor[i].address, 7) != thermometerSensor[i].address[7])
        printf(" CRC is not valid!\n");
	}
	else 
      printf(" - not found!\n");
  }
}

void discoverOneWire() {
  DeviceAddress addr;
  int8_t numTemperatureSensors;
  
  // Fetch the number of attached temperature sensors  
  numTemperatureSensors = sensors.getDeviceCount();
  printf("\nDiscover OneWire...\nnumSensors: %i, Parasite power: %s\n", numTemperatureSensors, (sensors.isParasitePowerMode() ? "ON" : "OFF"));

  for (uint8_t i=0; i<numTemperatureSensors && i<MAX_ATTACHED_DS18B20; i++) {
    if (!sensors.getAddress(addr, i))
	  printf("Unable to find address for Device %i\n", i);
	else {
	  printf("DS18B20 %i ", i);
	  Serial.print(" Address: ");
      printAddress(addr, true);
      printf(", Resolution: %i\n", sensors.getResolution(addr));
	}
  }
   printf("\n");
}

void setup(void){
  Serial.begin(57600);
  Serial.println("Start setup...");
  
  printf_begin();  
  
  if (!bmp.begin())
	Serial.println("Could not find BMP085!");

  dht.setup(HUMIDITY_SENSOR_DIGITAL_PIN);

  onewire_begin();

  radio.begin();
  
  Serial.println("End setup\n");
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress, bool bHex) {
  if(bHex) Serial.print("{");
  for( uint8_t i=0; i < 8; i++) {                        
      if(bHex) Serial.print("0x");
      if (deviceAddress[i] < 16)
        Serial.print('0');                        
      Serial.print(deviceAddress[i], HEX);                
      if (bHex && i < 7) Serial.print(", ");
    }
  if(bHex) Serial.print("}");
}

void getDHTdata(bool bSend) {
  Serial.print("\nDHT ");

  delay(dht.getMinimumSamplingPeriod());

  float temperature = dht.getTemperature();
  if (isnan(temperature))
      Serial.println("- Reading t failed");
  else if (bSend || temperature != lastTemp_dht) {
    lastTemp_dht = temperature;
    if (!isMetric) 
      temperature = dht.toFahrenheit(temperature);
    Serial.print("t = "); Serial.print(temperature); Serial.print(" ");
  }
  
  float humidity = dht.getHumidity();
  if (isnan(humidity))
      Serial.println("- Reading H failed");
  else if (bSend || humidity != lastHum_dht) {
      lastHum_dht = humidity;
      Serial.print("H = "); Serial.println(humidity);
	  radio.sendMsg(msg.setSensor(S_H_OUTSIDE).setType(V_HUM).set(lastHum_dht,0));
  }
  Serial.print("\n");
}

void getBMPdata(bool bSend) {
  Serial.print("\nBMP ");

  float pressure = bmp.readSealevelPressure(ABOVE_SEALEVEL)/100; //meters above sealevel
  float temperature = bmp.readTemperature();
  
  if (!isMetric) // Convert to fahrenheit
    temperature = temperature * 9.0 / 5.0 + 32.0;
  
  Serial.print("t = "); Serial.print(temperature);
  Serial.print(", P = "); Serial.println(pressure);

  //Serial.println(weather[forecast]);

  if (bSend || temperature != lastTemp_bmp) {
    lastTemp_bmp = temperature;
  }

  if (bSend || pressure != lastPressure_bmp) {
    lastPressure_bmp = pressure;
	radio.sendMsg(msg.setSensor(S_P_INDOOR).setType(V_PRESSURE).set(lastPressure_bmp,0));
  }
  Serial.print("\n");
}

void getTempSensorData(bool bSend) {
  Serial.println("\nDS18B20");
  
  sensors.requestTemperatures();
  
  for (uint8_t i = 0; i < TEMP_SENSOR_COUNT; i++)
    if (thermometerSensor[i].found) {
		float temperature = sensors.getTempC(thermometerSensor[i].address);
		printf("%s, t = ", my_msg::getSensorName(thermometerSensor[i].sensor)); Serial.println(temperature);
		if ((bSend || thermometerSensor[i].lastTemp != temperature) && temperature != -127.00) { // UNDEFINDED_VALUE
		   thermometerSensor[i].lastTemp = temperature;
		   radio.sendMsg(msg.setSensor(thermometerSensor[i].sensor).setType(V_TEMP).set(temperature,2));
		}
	}
  Serial.println("\n");
}

/*
void getDS18B20data() {
  // Read temperatures and send them to controller 
  sensors.requestTemperatures();
  for (uint8_t i=0; i<numTemperatureSensors && i<MAX_ATTACHED_DS18B20; i++) {
    
	float temperature = sensors.getTempC(thermometerAddress[i]);
    // Fetch and round temperature to one decimal
    //float temperature = sensors.getTempCByIndex(i); //static_cast<float>(static_cast<int>((isMetric?sensors.getTempCByIndex(i):sensors.getTempFByIndex(i)) * 10.)) / 10.;
    printf("DS18B20 #%i, t = ", i); Serial.println(temperature);
	
    // Only send data if temperature has changed and no error
    if (lastTemp_DS18B20[i] != temperature && temperature != -127.00) {
       lastTemp_DS18B20[i] = temperature;
	   radio.send(msg.setSensor(thermometerSensor[i]).setType(V_TEMP).set(temperature,2));
    }
  }
}
*/

void loop(void) {
  mNbrSendTicks++;
  bool bSend = false;
  if (mNbrSendTicks > sendPeriod) {
    bSend = true; 
    mNbrSendTicks = 0;
	Serial.println("\n\n--FORCED SEND START--");
  }

    getDHTdata(bSend);
    getBMPdata(bSend);
    getTempSensorData(bSend);
	if (bSend)
		Serial.println("--FORCED SEND END--\n\n");	
	radio.powerDown(); 
    delay(1000);
    radio.powerUp();
}
