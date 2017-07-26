/* 
  

//  Created by Leonardo Vinicius Kaminski Ferreira.
//  Copyright © 2015 Leonardo Vinicius Kaminski Ferreira. All rights reserved.


REVISIONS:
1-17-11 
changed values to RXPIN = 2 and TXPIN = to correspond with
hardware v14+. Hardware v13 used RXPIN = 3 and TXPIN = 2.
  
*/ 

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>

#define RXPIN 2
#define TXPIN 3

//#define RXPIN2 0
//#define TXPIN2 1
//Set this value equal to the baud rate of your GPS
#define GPSBAUD 4800

TinyGPS gps;

//SoftwareSerial uart_gps(RXPIN2, TXPIN2);
SoftwareSerial xbee(RXPIN, TXPIN);

void getgps(TinyGPS &gps);
void accelerometer();
void magnetometer();
void barometer();
void cardiacMonitor();

void setup()
{
  
  Serial.begin(GPSBAUD);
  //Sets baud rate of your GPS
  
  
  
  Serial.println("");
  Serial.println("GPS");
  Serial.println("       ...waiting for lock position...           ");
  Serial.println("");
  
  
  Wire.begin();
  
  Serial.println("started accelerometer");
  // enable to measute g data
  int ADXAddress = 0xA7 >> 1;
  Wire.beginTransmission(ADXAddress);
  Wire.write(0x2D);
  Wire.write(8);                //measuring enable
  Wire.endTransmission();     // stop transmitting

  Serial.println("ended accelerometer");
  
  
  #define address 0x1E
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  

  #define HRMI_I2C_ADDR      127
  #define HRMI_HR_ALG        1   // 1= average sample, 0 = raw sample
  setupHeartMonitor(HRMI_HR_ALG); // POLAR cardiac sensor ***
  
  xbee.begin(38400);
  
  Serial.println("");
  Serial.println("Xbee OK");
  xbee.print("       Welcome, Leonardo V. Kaminski Ferreira           ");
  xbee.print("\n");
  xbee.print("       We've started OK, all behaviours working properly...           ");
  xbee.print("\n");
  Serial.println("       ...waiting for lock position...           ");
  Serial.println("");
 
  
  
}

void loop()
{
  
  
  
  while(Serial.available())     
  {
      int c = Serial.read();  
      if(gps.encode(c))      
      {
        xbee.print("Waiting...\n");
        getgps(gps);         // then grab the data.
        float latitude, longitude;
        gps.f_get_position(&latitude, &longitude);
        xbee.print("\n");
        xbee.print("Lat: ");
        xbee.print(latitude, 5);
        xbee.print("\n");
        xbee.print("Long: ");
        xbee.print(longitude, 5);
        xbee.print("\n");
        xbee.print("Altitude(mt): "); 
        xbee.print(gps.f_altitude()); 
        xbee.print("\n");
        xbee.print("Course(dgre): ");
        xbee.print(gps.f_course());
        xbee.print("\n");
        xbee.print("Speed(kmph): "); 
        xbee.print(gps.f_speed_kmph());
        xbee.print("\n");
        int year;
        byte month, day, hour, minute, second, hundredths;
        gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
        xbee.print("Date: "); 
        xbee.print(month, DEC);
        xbee.print("/"); 
        xbee.print(day, DEC);
        xbee.print("/"); 
        xbee.print(year);
        xbee.print("  Time: "); 
        xbee.print(hour, DEC); 
        xbee.print(":"); 
        xbee.print(minute, DEC);
        xbee.print(":"); 
        xbee.print(second, DEC); 
        xbee.print(".");
        xbee.print(hundredths, DEC);
        xbee.print("\n");
        
        unsigned long chars;
        unsigned short sentences, failed_checksum;
        gps.stats(&chars, &sentences, &failed_checksum);
        xbee.print("Failed Checksums: ");
        xbee.print(failed_checksum);
        xbee.print("\n\n");
        
        acelerometro();
        magnetometro();
        barometro();
        monitorCardiaco();
        delay(700);
        xbee.print("Waiting...\n");
     
      }
  }
  
  
}

void getgps(TinyGPS &gps)
{
  
  float latitude, longitude;
 
  gps.f_get_position(&latitude, &longitude);

  Serial.print("Lat/Long: "); 
  Serial.print(latitude); 
  Serial.print(", "); 
  Serial.println(longitude,5);
  

  
  int year;
  byte month, day, hour, minute, second, hundredths;
  gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
  
  Serial.print("Date: "); Serial.print(month, DEC); Serial.print("/"); 
  Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
  Serial.print("  Time: "); Serial.print(hour, DEC); Serial.print(":"); 
  Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC); 
  Serial.print("."); Serial.println(hundredths, DEC);
  
 
  Serial.print("Altitude(meters): "); Serial.println(gps.f_altitude());  
  // Same goes for course
  Serial.print("Course(degrees): "); Serial.println(gps.f_course()); 
  // And same goes for speed
  Serial.print("Speed(kmph): "); Serial.println(gps.f_speed_kmph());
  Serial.println();
  
  //-----------------------
  
  xbee.print("Altitude(mt): "); xbee.print(gps.f_altitude());  
  // Same goes for course
  xbee.print("Course(dgre): "); xbee.print(gps.f_course()); 
  // And same goes for speed
  xbee.print("Speed(kmph): "); xbee.print(gps.f_speed_kmph());
  xbee.print("\n");
  
  //xbee.print("Date: "); xbee.print(month, DEC); xbee.print("/"); 
  //xbee.print(day, DEC); xbee.print("/"); xbee.print(year);
  //xbee.print("  Time: "); xbee.print(hour, DEC); 
  //xbee.print("\n");
  
  
  xbee.print("Lat/Long: "); 
  xbee.print(latitude,5); 
  xbee.print(", "); 
  xbee.print(longitude,5);
  xbee.print("\n");
  
  
  //-----------------------
  
  unsigned long chars;
  unsigned short sentences, failed_checksum;
  gps.stats(&chars, &sentences, &failed_checksum);
  Serial.print("Failed Checksums: ");
  Serial.print(failed_checksum);
  Serial.println(); Serial.println();
  
}


void accelerometer() {
  
  
  
  #define Register_ID 0
  #define Register_2D 0x2D
  #define Register_X0 0x32
  #define Register_X1 0x33
  #define Register_Y0 0x34
  #define Register_Y1 0x35
  #define Register_Z0 0x36
  #define Register_Z1 0x37
  
  int ADXAddress = 0xA7 >> 1;  // the default 7-bit slave address
  int reading = 0;
  int val=0;
  int X0,X1,X_out;
  int Y0,Y1,Y_out;
  int Z1,Z0,Z_out;
  double Xg,Yg,Zg;
  
  
  //--------------X
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_X0);
  Wire.write(Register_X1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    X0 = Wire.read();
    X1 = Wire.read(); 
    X1=X1<<8;
    X_out=X0+X1;   
  }

  //------------------Y
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Y0);
  Wire.write(Register_Y1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Y0 = Wire.read();
    Y1 = Wire.read(); 
    Y1=Y1<<8;
    Y_out=Y0+Y1;
  }
  //------------------Z
  Wire.beginTransmission(ADXAddress); // transmit to device
  Wire.write(Register_Z0);
  Wire.write(Register_Z1);
  Wire.endTransmission();
  Wire.requestFrom(ADXAddress,2); 
  if(Wire.available()<=2)   
  {
    Z0 = Wire.read();
    Z1 = Wire.read(); 
    Z1=Z1<<8;
    Z_out=Z0+Z1;
  }
  
  Xg=X_out/256.0;
  Yg=Y_out/256.0;
  Zg=Z_out/256.0;
  
  
  Serial.print("Accelerometer X= ");
  Serial.println(Xg);
  Serial.print("       ");
  Serial.print("Accelerometer Y= ");
  Serial.println(Yg);
  Serial.print("       ");
  Serial.print("Accelerometer Z= ");
  Serial.println(Zg);
  Serial.println("  ");
  
  
  xbee.print("Accelerometer X= ");
  xbee.print(Xg);
  xbee.print("\n");
  xbee.print("Accelerometer Y= ");
  xbee.print(Yg);
  xbee.print("\n");
  xbee.print("Accelerometer Z= ");
  xbee.print(Zg);
  xbee.print("\n");
  
  
}


void magnetometer() {
  
  int x,y,z; //triple axis data

  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  
  //Print out values of each axis
  Serial.print("Magnetometer X: ");
  Serial.println(x);
  Serial.print("Magnetometer Y: ");
  Serial.println(y);
  Serial.print("Magnetometer Z: ");
  Serial.println(z);
  
  
  
  xbee.print("Magnetometer X: ");
  xbee.print(x);
  xbee.print("\n");
  xbee.print("Magnetometer Y: ");
  xbee.print(y);
  xbee.print("\n");
  xbee.print("Magnetometer Z: ");
  xbee.print(z);
  xbee.print("\n");
  
}




// ------- ** BAROMETER

#define BMP085_ADDRESS 0x77  // I2C address of BMP085
const unsigned char OSS = 0;  // Oversampling Setting

// Calibration values
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

long b5; 


void barometer() {
  
  short temperature;
  long pressure;
  
  temperature = bmp085GetTemperature(bmp085ReadUT());
  pressure = bmp085GetPressure(bmp085ReadUP());
  
  /*
  Serial.print("Temperature: ");
  Serial.print(temperature, DEC);
  Serial.println(" *0.1 deg C");
  Serial.print("Pressure: ");
  Serial.print(pressure, DEC);
  Serial.println(" Pa");
  Serial.println();
  */
  
  
  xbee.print("Temperature: ");
  xbee.print(temperature, DEC);
  xbee.print(" *0.1 graus C");
  xbee.print("\n");
  xbee.print("Pressure: ");
  xbee.print(pressure, DEC);
  xbee.print(" Pa");
  xbee.print("\n");
  
}




void bmp085Calibration()
{
  
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
short bmp085GetTemperature(unsigned int ut)
{
  long x1, x2;
  
  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8)>>4);  
}


long bmp085GetPressure(unsigned long up)
{
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address1)
{
  unsigned char data;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address1);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address1)
{
  unsigned char msb, lsb;
  
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address1);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  
  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT()
{
  unsigned int ut;
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  delay(5);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
unsigned long bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();
  
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);
  
  return up;
}



void cardiacMonitor(){
  
  int heartRate = getHeartRate();
  Serial.println(heartRate);
  xbee.print(heartRate);
  xbee.print("\n");
  
  
}




void setupHeartMonitor(int type){
  //setup the heartrate monitor
  Wire.begin();
  writeRegister(HRMI_I2C_ADDR, 0x53, type); // Configure the HRMI with the requested algorithm mode
}

int getHeartRate(){
  //get and return heart rate
  //returns 0 if we couldnt get the heart rate
  byte i2cRspArray[3]; // I2C response array
  i2cRspArray[2] = 0;

  writeRegister(HRMI_I2C_ADDR,  0x47, 0x1); // Request a set of heart rate values 

  if (hrmiGetData(127, 3, i2cRspArray)) {
    return i2cRspArray[2];
  }
  else{
    return 0;
  }
}

void writeRegister(int deviceAddress1, byte address1, byte val1) {
  //I2C command to send data to a specific address on the device
  Wire.beginTransmission(deviceAddress1); // start transmission to device 
  Wire.write(address1);       // send register address
  Wire.write(val1);         // send value to write
  Wire.endTransmission();     // end transmission
}

boolean hrmiGetData(byte addr, byte numBytes, byte* dataArray){
  //Get data from heart rate monitor and fill dataArray byte with responce
  //Returns true if it was able to get it, false if not
  Wire.requestFrom(addr, numBytes);
  if (Wire.available()) {

    for (int i=0; i<numBytes; i++){
      dataArray[i] = Wire.read();
    }

    return true;
  }
  else{
    return false;
  }
}



