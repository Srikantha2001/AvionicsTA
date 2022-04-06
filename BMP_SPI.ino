

#include<SPI.h>
#include <SD.h>

int cs = 10;
int i = 0;
int32_t t_fine;
int32_t temperature,pressure;
float altiReading,preReading,tempReading;
SPISettings mySPISettings(500000, MSBFIRST, SPI_MODE0);

void setup() {
  // put your setup code here, to run once:
  pinMode(cs, OUTPUT);
  digitalWrite(cs, LOW);
  delay(1000);

  Serial.begin(9600);
  SPI.begin();

  delay(100);
  writeReg(0xE0, 0xB6);
  delay(100);

  delay(100);
  //writeReg(0xF4, 0x2F);
  writeReg(0xF4, 0x57);
  delay(100);

  delay(100);
  writeReg(0xF5, 0x30);
  delay(100);
}

void loop() {  
  tempReading = readTemperatureValue();
  Serial.print("Temp value : ");
  Serial.println(tempReading);

  preReading = readPressureValue();
  Serial.print("Pressure value : ");
  Serial.println(preReading);

  altiReading = readAltitudeValue(1013.25);
  Serial.print("Altitude value= ");
  Serial.println(altiReading); 
}

void writeReg(byte address, byte data) //BMP
{
  digitalWrite(cs, LOW);
  delay(100); 
  byte reg = address & 0x7F;
  SPI.beginTransaction(mySPISettings);
  SPI.transfer(reg);
  SPI.transfer(data);
  SPI.endTransaction();
  delay(100);
  digitalWrite(cs, HIGH);
}

byte readReg(byte regAddress) //BMP
{
  digitalWrite(cs, LOW);
  delay(20);
  byte regAd = regAddress | 0x80;
  SPI.beginTransaction(mySPISettings);
  SPI.transfer(regAd);
  byte val = SPI.transfer(0x00);
  SPI.endTransaction();
  delay(20);
  digitalWrite(cs, HIGH);
  return (val);
}

float readTemperatureValue() {
  int32_t var1, var2;
  int32_t adc_T = read24(0xFA);
  adc_T >>= 4 ;
  int16_t dig_T1 = 27875; // (int16_t)readReg(0x89)<<8 | (int16_t)readReg(0x88);
  int16_t dig_T2 =  26548 ;//(int16_t)readReg(0x8B)<<8 | (int16_t)readReg(0x8A);
  int16_t dig_T3 =   -1000;//(int16_t)readReg(0x8D)<<8 | (int16_t)readReg(0x8C);
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *((int32_t)dig_T2)) >>11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *((adc_T >> 4) - ((int32_t)dig_T1))) >>12) *((int32_t)dig_T3)) >>14;
  t_fine = var1 + var2;
  float T = (t_fine * 5 + 128) >> 8;
  return T/100 ; 
}


float readPressureValue() {
  int64_t var1, var2, p;
  int64_t adc_P = read24(0xF7);
  adc_P >>= 4;
  uint16_t dig_P1 =36376;//(int16_t)readReg(0x8F) << 8 | readReg(0x8E);
  int16_t dig_P2 = -10627;//(int16_t)readReg(0x91) << 8 | readReg(0x90);
  int16_t dig_P3 = 3024;//(int16_t)readReg(0x93) << 8 | readReg(0x92);
  int16_t dig_P4 = 3703;//(int16_t)readReg(0x95) << 8 | readReg(0x94);
  int16_t dig_P5 = 134;//(int16_t)readReg(0x97) << 8 | readReg(0x96);
  int16_t dig_P6 = -7;//(int16_t)readReg(0x99) << 8 | readReg(0x98);
  int16_t dig_P7 = 15500;//(int16_t)readReg(0x9B) << 8 | readReg(0x9A);
  int16_t dig_P8 = -14600;//(int16_t)readReg(0x9D) << 8 | readReg(0x9C);
  int16_t dig_P9 = 6000;//(int16_t)readReg(0x9F) << 8 | readReg(0x9E);
 
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * dig_P3) >> 8) +((var1 * (int64_t)dig_P2) << 12);
  var1 =(((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (float)p / 256;
}

int32_t spixfer(int32_t x) {
  return SPI.transfer(x);
}

int32_t read24(byte reg) {
  int32_t value;
  SPI.beginTransaction(mySPISettings);
  digitalWrite(cs, LOW);
  spixfer(reg | 0x80); // read, bit 7 high
  value = spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return value;
}
float readAltitudeValue(float seaLevelhPa) {
  float temp = preReading/100;
  float altitudeValue = 44330 * (1.0 - pow(temp / seaLevelhPa, 0.1903));
  return altitudeValue;
}
