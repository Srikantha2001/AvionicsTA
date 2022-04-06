
#include<SPI.h>
#include <SD.h>


int i = 0;
int cs_for_BMP = 10;
int chipSelectPin = 8;
int chipSelectForSDcard = 4;
int32_t t_fine;
int32_t temperature,pressure;
double accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
float altiReading,preReading,tempReading;
void matrixAdd(int r,int c,double m1[][2],double m2[][2],double res[][2]);
void matrixSubtract(int r,int c,double m1[][2],double m2[][2],double res[][2]);
void matrixMultiply(int r1, int c1, int r2, int c2,double m1[][2], double m2[][2], double res[][2]);
void matrixTranspose(int r, int c, double m1[][2],double m2[][2]);
int flagForParachuteDeployment =0;
int pinForParachuteDeployment = 2;


double temp1[2][2]={0,0,0,0};
double temp2[2][2]={0,0,0,0};
double dt = 0.02;
double stateMatrix[2][2]={0.74,0,0,0};
double stateTransitionMatrix[2][2]={1,dt,0,1};
double stateTransitionMatrixTranspose[2][2]={0,0,0,0};
double controlMatrix[2][2]={0.5*dt*dt,0,dt,0};
double H[2][2]={1,0,0,0};
double Ht[2][2]={0,0,0,0};
double Kt[2][2]={0,0,0,0};
double Q[2][2]={10,0,0,1};
double R[2][2]={1,0,0,1};
double y[2][2]={0,0,0,0};
double S[2][2]={0,0,0,0};
double I[2][2]={{1,0},{0,1}};
double kalmanGain[2][2]={0,0,0,0};
double verticalAccln[2][2]={0,0,0,0};
double currentAltitude[2][2]={0,0,0,0};
double P[2][2]={{1,0},{0,1}};

SPISettings mySPISettings(500000, MSBFIRST, SPI_MODE0);
void setup() {
  // put your setup code here, to run once:
  pinMode(cs_for_BMP, OUTPUT);
  digitalWrite(cs_for_BMP, LOW);
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

  pinMode(chipSelectPin, OUTPUT);

  writeRegister(0x1C, 0x18);
  writeRegister(0x6B, 0x00);
  // writeRegister(0x68, 0x07);
  // writeRegister(0x1D, 0x08);

  delay(100);

  pinMode(chipSelectForSDcard, OUTPUT);
  if (!SD.begin(chipSelectForSDcard)) 
  {
    Serial.println("Card failed, or not present");
    while (1);
  }

}

void loop() {  
  tempReading = readTemperatureValue();
  //Serial.print("Temp value : ");
  
  preReading = readPressureValue();
  //Serial.print("Pressure value : ");
  
  altiReading = readAltitudeValue(1013.25);
  //Serial.print("Altitude value= ");
  
  writeRegister(0x6A, 0x10);
  accelX = readRegister(0x3B) << 8 | readRegister(0x3C); //Store first two bytes into accelX
  accelY = readRegister(0x3D) << 8 | readRegister(0x3E); //Store middle two bytes into accelY
  accelZ = readRegister(0x3F) << 8 | readRegister(0x40); //Store last two bytes into accelZ
  processAccelData();

  //---------------------------------------------------------------------------------------------------
  //                                            KALMAN FILTERING                            
  //---------------------------------------------------------------------------------------------------

  // Initialising variables
  
  verticalAccln[0][0]=gForceZ;
  currentAltitude[0][0]=readAltitudeValue(1013.25);
  
  // STEP 1 Prediction
  
   
  matrixMultiply(2,2,2,1,stateTransitionMatrix,stateMatrix,temp1);//A*x
  matrixMultiply(2,1,1,1,controlMatrix,verticalAccln,temp2);//B*u
  matrixAdd(2,1,temp1,temp2,stateMatrix);// A*x + B*u

  // Calculating the Co-variance matrix => P=A*P*At +Q

  matrixMultiply(2,2,2,2,stateTransitionMatrix,P,temp1); // A*P
  matrixTranspose(2,2,stateTransitionMatrix,stateTransitionMatrixTranspose);// generating At(transpose of A)
  matrixMultiply(2,2,2,2,temp1,stateTransitionMatrixTranspose,temp2);// (A*P) * At
  matrixAdd(2,2,temp2,Q,P);// A*P*At +Q
  
  //STEP 2:Updation
  
  // calculating the difference between altitude from altimeter and accelerometer  y=z-H*x
  matrixMultiply(1,2,2,1,H,stateMatrix,temp1);//H*x
  matrixSubtract(1,1,currentAltitude,temp1,y);//y=z-(H*x)
  
  // Calculating the Kalman gain => K= P*Ht/(S) where S=H*P*Ht +R
  matrixTranspose(1,2,H,Ht); //calculating Ht(transpose of H)
  matrixMultiply(2,2,2,1,P,Ht,temp1);// P*Ht
  matrixMultiply(1,2,2,1,H,temp1,temp2);//H*P*Ht
  matrixAdd(1,1,R,temp2,S);// S = H*P*Ht +R
  S[0][0]=1/S[0][0];// changing S to 1/S 
  matrixMultiply(2,1,1,1,temp1,S,kalmanGain);// k=P*Ht*(1/S)

  // Calculating the estimated altitude => x(estimate) = x(predict) + K*y;
  matrixMultiply(2,1,1,1,kalmanGain,y,temp1);//K*y
  matrixAdd(2,1,stateMatrix,temp1,temp2);//x+Ky
  stateMatrix[0][0]=temp2[0][0];// assigning values to same state matrix;
  stateMatrix[0][1]=temp2[0][1];
  
  // calculating the updated Covariance matrix P   =>     P=[I-K*H]P
  matrixMultiply(2,1,1,2,kalmanGain,H,temp1);//K*H
  matrixSubtract(2,2,I,temp1,temp2);//I-K*H
  matrixMultiply(2,2,2,2,temp2,P,P);//[I-K*H]*P
  
  Serial.print(tempReading);
  Serial.print(" ");
  Serial.print(preReading);
  Serial.print(" ");
  Serial.print(altiReading);
  Serial.print(" ");
  Serial.print(gForceX);
  Serial.print(" ");
  Serial.print(gForceY);
  Serial.print(" ");
  Serial.print(gForceZ);
  Serial.print(" ");
  Serial.print(stateMatrix[0][0]);
  Serial.print(" ");
  Serial.println(stateMatrix[0][0]*3.28084);
  i = i + 1;
  LogData(tempReading, preReading,altiReading, gForceX, gForceY, gForceZ,stateMatrix[0][0]);

}

void writeReg(byte address, byte data) //BMP
{
  digitalWrite(cs_for_BMP, LOW);
  delay(100); 
  byte reg = address & 0x7F;
  SPI.beginTransaction(mySPISettings);
  SPI.transfer(reg);
  SPI.transfer(data);
  SPI.endTransaction();
  delay(100);
  digitalWrite(cs_for_BMP, HIGH);
}

byte readReg(byte regAddress) //BMP
{
  digitalWrite(cs_for_BMP, LOW);
  delay(20);
  byte regAd = regAddress | 0x80;
  SPI.beginTransaction(mySPISettings);
  SPI.transfer(regAd);
  byte val = SPI.transfer(0x00);
  SPI.endTransaction();
  delay(20);
  digitalWrite(cs_for_BMP, HIGH);
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
  digitalWrite(cs_for_BMP, LOW);
  spixfer(reg | 0x80); // read, bit 7 high
  value = spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  value <<= 8;
  value |= spixfer(0);
  digitalWrite(cs_for_BMP, HIGH);
  SPI.endTransaction();
  return value;
}
float readAltitudeValue(float seaLevelhPa) {
  float temp = preReading/100;
  float altitudeValue = 44330 * (1.0 - pow(temp / seaLevelhPa, 0.1903));
  return altitudeValue;
}

void writeRegister(byte thisRegister, byte thisValue) {
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(thisRegister); //Send register location
  SPI.transfer(thisValue); //Send value to record into register
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // release control of the SPI port
  SPI.endTransaction();
}

//Read from or write to register from the SCP1000:
unsigned int readRegister(byte thisRegister)
{
  byte dataToSend = thisRegister | 0x80;
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  // take the chip select low to select the device:
  digitalWrite(chipSelectPin, LOW);
  // send the device the register you want to read:
  SPI.transfer(dataToSend);
  // send a value of 0 to read the first byte returned:
  byte result = SPI.transfer(0x00);
  // decrement the number of bytes left to read:
  // take the chip select high to de-select:
  digitalWrite(chipSelectPin, HIGH);
  // release control of the SPI port
  SPI.endTransaction();
  return (result);
}

void processAccelData()
{
  if (accelX > 32768)
  {
    gForceX = 65535 - accelX;
    gForceX /= 2048.0;
  }
  else
  {
    gForceX = -accelX / 2048.0;
  }
  if (accelY > 32768)
  {
    gForceY = 65535 - accelY;
    gForceY /= 2048.0;
  }
  else
  {
    gForceY = -accelY / 2048.0;
  }
  if (accelZ > 32768)
  {
    gForceZ = 65535 - accelZ;
    gForceZ /= 2048.0;
  }
  else
  {
    gForceZ = -accelZ / 2048.0;
  }
}

void LogData(int32_t temperatureValue, int32_t pressureValue, float altitudeValue,  float gForceX, float gForceY, float gForceZ, float kf_altitude){
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
   String dataString="";
    if (dataFile) {
      
      dataString+=String(temperatureValue);
      dataString+=",";
      dataString+=String(pressureValue);
      dataString+=",";
      dataString+=String(altitudeValue);
      dataString+=",";
      dataString+=String(gForceX);
      dataString+=",";
      dataString+=String(gForceY);
      dataString+=",";
      dataString+=String(gForceZ);
      dataString+=",";
      dataString+=String(kf_altitude);
      dataString+=",";
      dataString+=String(kf_altitude*3.28084);
    }
    dataFile.println(dataString);
    dataFile.close(); 
}

void matrixAdd(int r,int c,double m1[][2],double m2[][2],double res[][2])
{
  int i,j;
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=0;
    }
  }
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=m1[i][j]+m2[i][j];
    }
  }
  return ; 
}

void matrixSubtract(int r,int c,double m1[][2],double m2[][2],double res[][2])
{
  int i,j;
  for(i=0;i<r;i++){
    for(j=0;j<c;j++){
      res[i][j]=m1[i][j]-m2[i][j];
    }
  }  
}

void matrixMultiply(int r1, int c1, int r2, int c2,double m1[][2], double m2[][2], double res[][2])
{
  int i, j, k;
  for(i = 0; i < r1; ++i){
     for(j = 0; j < c2; ++j){
        res[i][j] = 0;
     }
  }
  for(i = 0; i < r1; ++i){
    for(j = 0; j < c2; ++j){
      for(k=0; k<c1; ++k){
        res[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }

}
void matrixTranspose(int r, int c, double m1[][2],double m2[][2])
{
    int i,j;
    for(i = 0; i < r; ++i){
     for(j = 0; j < c; ++j){
        m2[i][j] = 0;
     }
  }
    for(i=0;i<r;i++){
      for(j=0;j<c;j++){
        m2[j][i]=m1[i][j];
      }
    }  
}
