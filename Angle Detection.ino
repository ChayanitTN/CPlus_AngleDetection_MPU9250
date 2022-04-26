#include<Wire.h>
const int MPU2=0x69,MPU1=0x68;
#include "MPU9250.h" 
MPU9250 mpu; // declare variable for mpu
#include <MPU9250_WE.h>
MPU9250_WE myMPU19250 = MPU9250_WE(MPU1);MPU9250_WE myMPU29250 = MPU9250_WE(MPU2);xyzFloat a1Value, a2Value ; 
float  Accel_C2, MPU1ARoll,MPU1APitch,MPU1AYaw,MPU2ARoll,MPU2APitch,MPU2AYaw,MPU1APitchrad,MPU2ARollrad,avg1num3,diff1, avg1num4, diff4, avg1num5,diff5,avg2num3,diff2, diffcer,CerAngle0, CerAngle,angle1,angle,angle0, baseline,baselineky,baselineCerv, one, two, avgcer;float numcollect13[] = {0,0,0};float numcollect14[] = {0,0,0,0};float numcollect15[] = {0,0,0,0,0};float numcollect23[] = {0,0,0};float numcollect[]   = {0,0,0,0,0,0,0,0,0,0};float prevavg1num3, prevavg1num4, prevavg1num5, prevavg2num3, prevavgcer =0; int motor = 2;float Average_Angle, SumOfAngle = 0;int Index = 0;float angle_array[5];
//=======================================================================//
uint8_t writeMPU9250Register(uint8_t reg, uint8_t val){
    Wire.beginTransmission(MPU1);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission();
}

uint8_t readMPU9250Register8(uint8_t reg){
    uint8_t regValue = 0;
    Wire.beginTransmission(MPU1);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU1,1);
    if(Wire.available()){
        regValue = Wire.read();
    }
    return regValue;
}
void setup(){ 
      Serial.begin(9600);
      Wire.begin();  //------------------------------------------setup MPU1
      Wire.beginTransmission(MPU1);Wire.write(0x6B);/*PWR_MGMT_1 register*/  Wire.write(0);/*wakes up the MPU*/  Wire.endTransmission(true); 
      Wire.begin();  //------------------------------------------setup MPU2
      Wire.beginTransmission(MPU2);Wire.write(0x6B);                         Wire.write(0);                      Wire.endTransmission(true);
      if(!myMPU19250.init()){  Serial.println("MPU19250 does not respond");  }
      if(!myMPU29250.init()){  Serial.println("MPU29250 does not respond");  }
      Serial.println("Position you MPU29250 flat and don't move it - calibrating...");
      delay(1000);
      myMPU19250.autoOffsets();
      Serial.println("Done MPU#1!");
      myMPU29250.autoOffsets();
      Serial.println("Done! MPU#2");
      myMPU19250.setGyrDLPF(MPU9250_DLPF_6); myMPU19250.setGyrRange(MPU9250_GYRO_RANGE_250); myMPU19250.setAccRange(MPU9250_ACC_RANGE_2G); myMPU19250.enableAccDLPF(true); myMPU19250.setAccDLPF(MPU9250_DLPF_6);  // lowest noise
      myMPU29250.setGyrDLPF(MPU9250_DLPF_6); myMPU29250.setGyrRange(MPU9250_GYRO_RANGE_250); myMPU29250.setAccRange(MPU9250_ACC_RANGE_2G); myMPU29250.enableAccDLPF(true); myMPU29250.setAccDLPF(MPU9250_DLPF_6);
      delay(100);
      //take the code for finding baseline out hehe
}     
//MPU1 - don't connect with ADO (C7)
//MPU2 - do    connect with ADO (Tragus)
void loop(){
  Angle_Finding();
  Red_ANGLE();
  //Red_SPEED();
  if (Index < 3){ 
    angle_array[Index] = angle;   
    Index++;  
  }else{
    for ( int i = 0; i < 3 ; i++ ){
    SumOfAngle += angle_array[ i ];
    } 
    Index = 0;
    Average_Angle = SumOfAngle/3;
  }
  //-------------------------------------- < averaged_angle >;
  Serial.print("Average_Angle = ");Serial.print(Average_Angle);
  Yellow_ANGLE();
  Average_Angle = 0;
  SumOfAngle = 0;
  //-------------------------------------- < /averaged_angle >;
  Serial.println();Serial.println("   ");
  
}
void Angle_Finding(){
      a1Value = myMPU19250.getGValues();
      a2Value = myMPU29250.getGValues();
      //finding speed
      Accel_C2 = a1Value.x * 10;
      //finding angles 
      MPU1ARoll  = atan2(a1Value.y,  sqrt(a1Value.x * a1Value.x + a1Value.z * a1Value.z)) * 180 / M_PI;
      MPU1APitch = atan2(-a1Value.x, sqrt(a1Value.y * a1Value.y + a1Value.z * a1Value.z)) * 180 / M_PI;
      MPU1AYaw = atan2(a1Value.z, sqrt(a1Value.x * a1Value.x + a1Value.y * a1Value.y)) * 180 / M_PI;
      MPU2ARoll  = atan2(a2Value.y,  sqrt(a2Value.x * a2Value.x + a2Value.z * a2Value.z)) * 180 / M_PI;
      MPU2APitch = atan2(-a2Value.x, sqrt(a2Value.y * a2Value.y + a2Value.z * a2Value.z)) * 180 / M_PI;
      MPU2AYaw = atan2(a2Value.z, sqrt(a2Value.x * a2Value.x + a2Value.y * a2Value.y)) * 180 / M_PI;
      //avgnumber of roll in mpu2
      avg1number5(); 
      avg2number3();
      if (diff2 <= 1 && diff2 >= -1){
        if (diff5 <= 1 && diff5 >= -1){
          MPU1APitchrad = (MPU1APitch* M_PI)/180.00;
          MPU2ARollrad = (MPU2ARoll* M_PI)/180.00;
          one = ((cos(MPU1APitchrad))*(cos(MPU2ARollrad))) + ((sin(MPU1APitchrad))*(sin(MPU2ARollrad)));   
          two = ((cos(MPU1APitchrad))*(sin(MPU2ARollrad))) - ((sin(MPU1APitchrad))*(cos(MPU2ARollrad)));
          angle1 = (atan2(two, one));
          angle0 = -1*((angle1 * 180)/M_PI);
          angle = angle0 - baselineCerv;
        } 
      }
      Serial.print("Related Angle = ");Serial.println(angle);
      Serial.print("Speed = ");Serial.println(Accel_C2);
      delay(39);
}
void avg1number3(){
  int sumnum13 = 0;
  numcollect13[2] = MPU1APitch;
  for (int i=0; i < 3; i++){
    sumnum13 += numcollect13[i];
  }
  avg1num3 = sumnum13 /3;
  diff1 = avg1num3 - prevavg1num3;
  prevavg1num3 = avg1num3;
  for (int j =0; j < 2; j++){ 
    numcollect13[j] = numcollect13[j+1];
  }
}     
void avg1number4(){
  int sumnum14 = 0;
  numcollect14[3] = MPU1APitch;
  for (int i=0; i < 4; i++){
    sumnum14 += numcollect14[i];
  }
  avg1num4 = sumnum14 /4;
  diff4 = avg1num4 - prevavg1num4;
  prevavg1num4 = avg1num4;
  for (int j =0; j < 3; j++){ 
    numcollect14[j] = numcollect14[j+1];
  }
}     
void avg1number5(){
  int sumnum15 = 0;
  numcollect15[4] = MPU1APitch;
  for (int i=0; i < 5; i++){
    sumnum15 += numcollect15[i];
  }
  avg1num5 = sumnum15 /5;
  diff5 = avg1num5 - prevavg1num5;
  prevavg1num5 = avg1num5;
  for (int j =0; j < 4; j++){ 
    numcollect15[j] = numcollect15[j+1];
  }
}     
//average number from transformation 
void avg2number3(){
  int sumnum23 = 0;
  numcollect23[2] = MPU2ARoll;
  for (int i=0; i < 3; i++){
    sumnum23 += numcollect23[i];
  }
  avg2num3 = sumnum23 /3;
  diff2 = avg2num3 - prevavg2num3;
  prevavg2num3 = avg2num3;
  for (int j =0; j < 2; j++){ 
    numcollect23[j] = numcollect23[j+1];
  }
}     
void avgcervical(){
  int sumnumC = 0;
  numcollect[9] = angle;
  for (int i=0; i < 5; i++){
    sumnumC += numcollect[i];
  }
  avgcer = sumnumC/5;
  diffcer = avgcer - prevavgcer;
  prevavgcer = avgcer;
  for (int j =0; j < 4; j++){ 
    numcollect[j] = numcollect[j+1];
  }
}
//=======================================================================//
//--------------------------------------------------------------------- vibrate_SPEED
void vibrate_SPEED(){
  digitalWrite(motor, HIGH);              //vibrate for 90 ms
  delay(90);   
  digitalWrite(motor, LOW);               //stop vibrating for 50 ms
  delay(50);
  digitalWrite(motor, HIGH);
  delay(90); 
  digitalWrite(motor, LOW);
  delay(100);
  digitalWrite(motor, HIGH);            
  delay(90);   
  digitalWrite(motor, LOW);              
  delay(50);
  digitalWrite(motor, HIGH);
  delay(90); 
  digitalWrite(motor, LOW);
}
//--------------------------------------------------------------------- vibrate_ANGLE
void vibrate_ANGLE(){
  
  digitalWrite(motor, HIGH);
  delay(50);   
  digitalWrite(motor, LOW);
  angle = 0;
}
//----------------------------------------------- Red_ANGLE
void Red_ANGLE(){
    if (angle <= -19 || angle >= 22){
      Serial.println("   ");
    Serial.print("-------------------------------vibrate RED");
    vibrate_ANGLE();
  }
}                                     
//--------------------------------------------------------------------- Red_SPEED
void Red_SPEED(){
  if (Accel_C2 > 8 || Accel_C2 < -8){
    Serial.println("   ");
    Serial.print("-------------------------------vibrate SPEED");
    vibrate_SPEED();
  }
}                                                                     
//--------------------------------------------------------------------- Yellow_ANGLE
void Yellow_ANGLE(){ 
    if (Average_Angle > -19 && Average_Angle <= -13){
      Serial.println("   ");
      Serial.print("-----------------------------vibrate YLW extn");
      vibrate_ANGLE();
    }else{
      Serial.println("   ");
      if (Average_Angle >= 14 && Average_Angle < 22){
      Serial.print("-----------------------------vibrate YLW extn");
      vibrate_ANGLE();
    }
  }
}
