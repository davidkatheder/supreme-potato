/**
  Sketch for reading and writing various data types to the 23LC1024 SRAM via SPI 
  with the Arduino SPI libary on an Arduino Uno.
  
  Author: David Katheder
  Date: 24.06.2020
**/

#include <SPI.h>

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

// SPI settings: 
// Sets maximum clock frequency to 20 MHz
// Bitorder to most significant bit first
// Clock polarity and phase to 0
SPISettings settings = SPISettings(20000000, MSBFIRST, SPI_MODE0);

/****************************************************************************/

void WriteModeRegister(byte mode);
byte ReadModeRegister();

void WriteByte(uint32_t address, byte data);
byte ReadByte(uint32_t address);

void WriteChar(byte mode, uint32_t address, char* data, uint32_t len);
void ReadChar(byte mode, uint32_t address, char* buff, uint32_t len);

void WriteInt(byte mode, uint32_t address, int* data, uint32_t len);
void ReadInt(byte mode, uint32_t address, int* buff, uint32_t len);

void WriteFloat(byte mode, uint32_t address, float* data, uint32_t len);
void ReadFloat(byte mode, uint32_t address, float* buff, uint32_t len);

void Routine1();  // Read and write example char array
void Routine2();  // Read and write example int array
void Routine3();  // Read and write example float array

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    ; // wait for serial port to connect
  }

  SPI.begin(); // initializes SPI bus by setting SCK, MOSI, and SS to outputs, sets CS to HIGH
  
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
        case '1': 
              Routine1();
              break;
       case '2':
              Routine2();
              break;
       case '3':
              Routine3();
              break;
       default: 
              Serial.print("Enter valid routine number\n");
      }
  }

}

byte ReadModeRegister() {
  SPI.beginTransaction(settings);   // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);            // assert chip select
  
  SPI.transfer(RDMR);               // transfer read mode register instruction
  byte mode = SPI.transfer(0x00);   // recieve byte
  
  digitalWrite(CS, HIGH);           // deassert chip select
  SPI.endTransaction();             // release SPI bus
 
  return mode;                      // return data
}

void WriteModeRegister(byte mode) {
  SPI.beginTransaction(settings);   // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);            // assert chip select
  
  SPI.transfer(WRMR);               // transfer write mode register instruction
  SPI.transfer(mode);               // transfer mode byte
  
  digitalWrite(CS, HIGH);           // deassert chip select
  SPI.endTransaction();             // release SPI bus
}

/** Read a byte of data from an address **/
byte ReadByte(uint32_t address) {
  WriteModeRegister(ByteMode);            // write register to ByteMode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select

  SPI.transfer(READ);                     // transfer READ instruction byte
  
  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte
  
  byte data = SPI.transfer(0x00);         // read data
  
  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus

  return data;                            // return data
}

/** Writing a byte of data to an address **/
void WriteByte(uint32_t address, byte data) {
  WriteModeRegister(ByteMode);            // write register to ByteMode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select

  SPI.transfer(WRITE);                    // transfer WRITE instruction byte

  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte

  SPI.transfer(data);                     // write single data byte

  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}

/** Write char arrays **/
void WriteChar(byte mode, uint32_t address, char * data, uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select

  SPI.transfer(WRITE);                    // transfer WRITE instruction byte

  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte

 for (int i = 0; i < len; i++) {
   SPI.transfer(data[i]);                // write bytes from data array
 }
 
  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}

/** Read char arrays **/
void ReadChar(byte mode, uint32_t address,  char *buff, uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select

  SPI.transfer(READ);                     // transfer READ instruction byte

  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte

 for (int i = 0; i < len; i++) {
   buff[i] = SPI.transfer(0x00);         // read bytes into buffer array
 }

  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}


/** Write 16 bit integer arrays **/
void WriteInt(byte mode, uint32_t address, int * data, uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select
  
  SPI.transfer(WRITE);                    // transfer WRITE instruction byte

  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte

  for (int i = 0; i < len; i++) {
    SPI.transfer16(data[i]);              // transfer two bytes
  }
  
  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus
}


/** Read 16 bit integer arrays **/
void ReadInt(byte mode, uint32_t address, int *buff,  uint32_t len) {
  WriteModeRegister(mode);                // write mode register to provided mode

  SPI.beginTransaction(settings);         // assert SPISettings, gain control of SPI bus
  digitalWrite(CS, LOW);                  // assert chip select

  SPI.transfer(READ);                     // transfer WRITE instruction byte
  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte
  
  for (int i = 0; i < len; i++) {
    buff[i] = SPI.transfer16(0x0000);     // read two bytes at once into buffer array
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

  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte

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
  
  SPI.transfer((byte) (address >> 16));   // transfer highest byte
  SPI.transfer((byte) (address >> 8));    // transfer middle byte
  SPI.transfer((byte) address);           // transfer lowest byte

  for (int i = 0; i < len; i++) {
    uint32_t n = 0;

    // example of adding and shifting 
    // n = 0x00000000;
    // ret = SPI.transfer(0x00);      ret == 0xFF
    // n += ret;                      n == 0x000000FF
    // n = (n << 8);                  n == 0x0000FF00
    // ret = SPI.transfer(0x00);      ret == 0x01
    // n += ret;                      n == 0x0000FF01
    // n = (n << 8);                  n == 0x00FF0100
    // and so on to the last byte
    
    n += SPI.transfer(0x00);              // read 8 bits and add them to n
    n = (n << 8);                         // shift by 8 bits to the left
    n += SPI.transfer(0x00);              // read 8 bits and add them to n
    n = (n << 8);                         // shift by 8 bits to the left
    n += SPI.transfer(0x00);              // read 8 bits and add them to n
    n = (n << 8);                         // shift by 8 bits to the left
    n += SPI.transfer(0x00);              // read 8 bits and add them to n

    memcpy((buff+i), &n, sizeof(float));  // copy read 32 bits to float buffer array
  }
  digitalWrite(CS, HIGH);                 // deassert chip select
  SPI.endTransaction();                   // release SPI bus  
}

void Routine1(){
  // Write some sentences to the SRAM and read them back.
  char data[12] = {"Hello World!"};       // define data
  char buff[12];                          // initialize buffer array
  
  Serial.println("Writing char array to SRAM in sequential mode.");
  Serial.print("Data to be written to SRAM: ");
  for(int i=0; i<12; i++) {
    Serial.print(data[i]);                // print data array data to serial monitor
  }
  WriteChar(SequentialMode, 0, data, 12); // Write data to chip at address 0
  Serial.print("\ndone!\n");
  Serial.println("Reading char array.");  
  ReadChar(SequentialMode, 0, buff, 12);  // Read data from chip starting at address 0
  for(int i=0; i<12; i++) {
    Serial.print(buff[i]);                // print buffer array data to serial monitor
  }
  Serial.print("\n");                     // end line
}

void Routine2(){
  // Write some 16 bit integers and read them back
  int len = 17;

  // define some data
  int data[len];
  for(int i=0; i<len; i++) {
    data[i] = -8+i;
  }

  // declare buffer array
  int buff[len];
  
  Serial.print("\nSEQUENTIAL MODE\n");  
  Serial.println("Writing int array to SRAM at address 0.");
  Serial.print("Data to be written to SRAM: ");
  for(int i=0; i<17; i++) {
    Serial.print(data[i]);                // print data array data to serial monitor
    Serial.print(" ");
  }
  WriteInt(SequentialMode, 0, data, len); // Write data to chip at address 0
  Serial.print("\ndone!\n");

  Serial.println("Reading char array.");  
  ReadInt(SequentialMode, 0, buff, len);  // Read data from chip starting at address 0

  for(int i=0; i<len; i++) {
    Serial.print(buff[i]);                // print buffer array data to serial monitor
    Serial.print(" ");
  }
  Serial.print("\n");                     // end line

  Serial.print("\n\PAGE MODE\n");  
  Serial.println("Writing char array to SRAM at address 0.");
  WriteInt(PageMode, 0, data, len);       // Write data to chip at address 0
  Serial.println("done!");
  Serial.println("Reading char array.");  
  ReadInt(PageMode, 0, buff, len);        // Read data from chip starting at address 0
  
  for(int i=0; i<len; i++) {
    Serial.print(buff[i]);                // print buffer array data to serial monitor
    Serial.print(" ");
  }
  Serial.print("\n");                     // end line
}


/* Read and write float array to SRAM */
void Routine3() {

  int len = 9;          // length of data and buffer array

  // initialize data and buffer array
  float data[len]= {0.0 ,0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8};   
  float buff[len];

  // Print routine description on serial monitor
  Serial.print("Read and write ");
  Serial.print(len);
  Serial.print(" floating point numbers (");
  Serial.print(sizeof(float)*8);
  Serial.print(" bit) to SRAM and read them back\n");

  // Print initial data on serial monitor
  Serial.println("Float data to be wrote to SRAM");
  for(int i=0; i<len; i++) {
    Serial.print(data[i]);
    Serial.print(" ");
  }

  // Write in sequential mode
  Serial.print("\n\nSEQUENTIAL MODE\n");  
  Serial.print("Write float array in sequential mode starting at address 0:\n");
  WriteFloat(SequentialMode, 0, data, len);
  Serial.println("done!");
  
  // Read data in sequential mode into buffer array
  ReadFloat(SequentialMode, 0, buff, len);
  Serial.print("Read data from address 0 in sequential mode:\n");
  
  // Print buffer array to serial monitor
  for(int i=0; i<len; i++) {
    Serial.print(buff[i]);
    Serial.print(" ");
  }
  
  // Write in page mode
  Serial.println("\n\nPAGE MODE");  
  Serial.println("Write float array in page mode starting at address 0:");
  WriteFloat(PageMode, 0, data, len);
  Serial.println("done!");
  
  // Read in page mode into buffer array
  ReadFloat(PageMode, 0, buff, len);
  Serial.println("Read data from address 0:");
  
  // Print buffer array to serial monitor
  for(int i=0; i<len; i++) {
    Serial.print(buff[i]);
    Serial.print(" ");
  }
  Serial.print("\n");
  
}