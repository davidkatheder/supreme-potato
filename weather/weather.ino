/**
  Sketch for reading and writing temperature and humidity measurements and the current timestamp
  to the 23LC1024 SRAM via SPI with the Arduino SPI libary on an Arduino Uno.
  For the measurements of tempereature and himidity the DHT22 sensor is used.
  The unix timestamp is provided by DS3231 RTC clock module.

  dependencies on external libaries:
  Adafruit Unified Sensor libary: https://github.com/adafruit/Adafruit_Sensor
  Adafruit DHT-sensor-libary: https://github.com/adafruit/DHT-sensor-library
  Adafruit RTClib : https://github.com/adafruit/RTClib.git

  The code concerning the DHT22 sensor and the RTC module is heavily based on the example code provided with the adafruit libaries.
  
  Author: David Katheder
  Date: 30.06.2020
**/

#include <SPI.h>
#include <Wire.h>

#include <DHT.h>
#include "RTClib.h"

// Define constants                                                      

// read/write instruction bytes
const byte READ = 0x03;             // READ instruction
const byte WRITE = 0x02;            // WRITE instruction

// read/write mode register instruction bytes
const byte RDMR = 0x05;             // READ mode register instruction
const byte WRMR = 0x01;             // WRITE mode register instruction

// read/write modes
const byte ByteMode = 0x00;         // R/W single byte
const byte SequentialMode = 0x40;   // R/W sequentially from a starting address
const byte PageMode = 0x80;         // R/W from starting address on 32 kB page

// chip select pin number
const int CS = 10;        

// DHT pin number and type
const int DHTPIN = 2;
const int DHTTYPE = DHT22;

// bool indication measuring loop
bool MEASURE = true;
bool syncOnFirstStart = false;

// current address pointer
uint32_t address = 0;

// current measurement counter
uint32_t measurements = 0;

// Initialize DHT sensor
DHT dht(DHTPIN, DHTTYPE);

// declare RTC module
RTC_DS3231 rtc;

// SPI settings: 
// Sets maximum clock frequency to 20 MHz
// Bitorder to most significant bit first
// Clock polarity and phase to 0
SPISettings settings = SPISettings(20000000, MSBFIRST, SPI_MODE0);

/****************************************************************************/

void WriteModeRegister(byte mode);

void WriteInt(byte mode, uint32_t address, int* data, uint32_t len);
void ReadInt(byte mode, uint32_t address, int* buff, uint32_t len);

void WriteFloat(byte mode, uint32_t address, float* data, uint32_t len);
void ReadFloat(byte mode, uint32_t address, float* buff, uint32_t len);

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect
  }

  SPI.begin(); // initializes SPI bus by setting SCK, MOSI, and SS to outputs, sets CS to HIGH
  dht.begin();

  if (! rtc.begin()) {
    Serial.println("Cant find RTC module");
    while (1);
  }
  
  if (rtc.lostPower() || syncOnFirstStart) {
    Serial.println("RTC lost power, the time will be freshly synchonized.");
    // adjust date and time of RTC the the compilation time of the code
    // this means the RTC module is going to be behind some seconds
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println("Setup complete");
}

void loop() {
  // Check for input from serial monitor
  if(Serial.available()>=1) {
      char i = Serial.read();
      Serial.print("Recieved input: ");
      Serial.println(i);
      
      // Start corresponding routine
      switch(i) {

        case '0':
                // Stop measuring loop and print out the data 
                MEASURE = false;
                Serial.println("Stopped measuring, read data");
                Serial.println("Measurement | Temperature in Celsius | Humidity in percent | Unix timestamp");

                // for loop for reading recorded data or whole chip if storage was exeeded.
                for(uint32_t i=0; (i<measurements && i<0x00020000); i++) {
                  // create buffer arrays for floats and ints
                  float buff[2];
                  uint32_t ibuff[2];
                  
                  // Read floats from SRAM
                  ReadFloat(SequentialMode, 4*i*sizeof(float), buff, 2);

                  // Read ints from SRAM
                  ReadInt(SequentialMode, 4*i*sizeof(float)+8, ibuff, 2);

                  // Print measurements in columns
                  Serial.print("\t");
                  Serial.print(ibuff[1]);
                  Serial.print("\t\t");
                  Serial.print(buff[0]);
                  Serial.print("\t\t\t");
                  Serial.print(buff[1]);
                  Serial.print("\t\t");
                  Serial.print(ibuff[0]);
                  Serial.print("\n");
                }
                break;
       case '1':
                Serial.println("restart measuring");
                MEASURE = true;
               break;
       default: 
              Serial.print("Enter valid routine number\n");
      }
  }

  if(MEASURE) {
      // Wait a few seconds between measurements.
      Serial.print("Measurement ");
      Serial.print(measurements);
      Serial.print("\n");
      delay(2000);

      // Reading temperature or humidity takes about 250 milliseconds!
      // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
      float data[2];
      // Read temperature as Celsius (the default)
      data[0] = dht.readTemperature();
      data[1] = dht.readHumidity();

      // get current unix timestamp
      DateTime now = rtc.now();
      uint32_t time = now.unixtime();

      // Write temperature and humidity data to SRAM
      WriteFloat(SequentialMode, address, data, 2);
      // Increment address by the size of the written data
      address +=8;
      
      // Write timestamp
      WriteInt(SequentialMode, address, &time, 1);
      // Increment address
      address += 4;

      // Write measurement number
      WriteInt(SequentialMode, address, &measurements, 1);
      // Increment address
      address += 4;

      // Increment measurement number
      measurements++;      
  }

}

void WriteModeRegister(byte mode) {
  SPI.beginTransaction(settings);   // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);            // assert chip select
  
  SPI.transfer(WRMR);               // transfer write mode register instruction
  SPI.transfer(mode);               // transfer mode byte
  
  digitalWrite(CS, HIGH);           // deassert chip select
  SPI.endTransaction();             // release SPI bus
}

/** Write 32 bit integer arrays **/
void WriteInt(byte mode, uint32_t address, uint32_t * data, uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select
  
  SPI.transfer(WRITE);                    // transfer WRITE instruction byte

  SPI.transfer((byte) (address >> 16));   // transfer most sigificant byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lest significant byte

for(int i=0; i<len; i++){
  for(int j=sizeof(uint32_t)-1; j>=0; j--) {
      SPI.transfer((byte) (data[i] >> 8*j));     // transfer bytes
  }
}
  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}


/** Read 32 bit integer arrays **/
void ReadInt(byte mode, uint32_t address, uint32_t *buff,  uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select

  SPI.transfer(READ);                     // transfer WRITE instruction byte
  SPI.transfer((byte) (address >> 16));   // transfer most sigificant byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lest significant byte
  
  for (int i = 0; i < len; i++) {
    buff[i] = 0;
    buff[i] = SPI.transfer16(0x0000);     // read two bytes at once into buffer array
    buff[i] = (buff[i] << 16);
    buff[i] += SPI.transfer16(0x0000);
  }


  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}

/** Write 32 bit floating point number arrays**/
void WriteFloat(byte mode, uint32_t address, float* data, uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode
  
  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select
  
  SPI.transfer(WRITE);                    // transfer WRITE instruction byte

  SPI.transfer((byte) (address >> 16));   // transfer most sigificant address byte
  SPI.transfer((byte) (address >> 8));    // transfer middle address byte
  SPI.transfer((byte) address);           // transfer lest significant address byte

  for (int i = 0; i < len; i++) {
    uint32_t n;                           // define unsigned int for storing float bits
    memcpy(&n, (data+i), sizeof(float));  // copy float bits to unsigned integer memory address

    for(int j=sizeof(float)-1; j>=0; j--) {
      SPI.transfer((byte) (n >> 8*j));     // transfer bytes
    }
  }

  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}


/** Read 32 bit floating point number arrays**/
void ReadFloat(byte mode, uint32_t address, float* buff, uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select
  
  SPI.transfer(READ);
  
  SPI.transfer((byte) (address >> 16));   // transfer most sigificant byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lest significant byte

  for (int i = 0; i < len; i++) {
    uint32_t n = 0;

    // example of adding and shifting 
    // n = 0x00000000;
    // ret = SPI.transfer(0x00);      ret == 0xFF
    // n += ret;                      n == 0x000000FF
    // n = (n << 8);                  n == 0x0000FF00
    // ret = SPI.transfer(0x00);      ret == 0x01
    // n += ret;                      n == 0x0000FF00
    // n = (n << 8);                  n == 0x0000FF01
    // and so on
    
    n += SPI.transfer16(0x0000);              // read 16 bits and add them to n
    n = (n << 16);                            // shift by 16 bits to the left
    n += SPI.transfer16(0x0000);              // read 16 bits and add them to n

    memcpy((buff+i), &n, sizeof(float));  // copy the 32 bits to float buffer array
  }
  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus  
}
