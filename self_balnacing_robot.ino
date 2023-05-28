#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"


void motorSpeedControl(int angle);


#define Kp 10
#define Kd 5
#define Ki 1

float targetAngle = 0;
  int Len = 3;  
  int  Ren = 6; 

  int leftMotorForwardPin = 8;
  int leftMotorReversePin = 7;
  int rightMotorForwardPin = 10;
  int rightMotorReversePin = 9;



MPU6050 mpu;

int16_t accX, accY, accZ;
int16_t gyroX,gyroY,gyroZ;

volatile float accAngle, gyroAngle = 0, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
int motorspeed = 0, gyroRate;
unsigned long currTime, prevTime=0, loopTime;


 


void setup() {

  //#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
         Wire.setClock(400000);
    //#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        //Fastwire::setup(400, true);
    //#endif

    // set the status LED to output mode 
  // pinMode(13, OUTPUT);

  //  // set the motor control and PWM pins to output mode
  //  pinMode(leftMotorForwardPin, OUTPUT);
  //  pinMode(leftMotorReversePin, OUTPUT);
  //  pinMode(rightMotorForwardPin, OUTPUT);
  //  pinMode(rightMotorReversePin, OUTPUT);
  //  pinMode(Ren, OUTPUT);
  // pinMode(Len, OUTPUT);

  mpu.initialize();
   mpu.setXAccelOffset(-2453);
    mpu.setZAccelOffset(2354);
    mpu.setYGyroOffset(48);


  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  
  Serial.begin(9600);
}



  void loop(){
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  accX = mpu.getAccelerationX();

  //gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  //gyroZ = mpu.getRotationZ();

  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;

  // calculate the angle of inclination
  accAngle = atan2(accX, accZ)*(180/PI);
  //Serial.print("Pitch Acc. Angle :" );
  //Serial.println(accAngle);
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*loopTime/1000;
  //Serial.print("Pitch Gyro Angle :" );  
  //Serial.println(gyroAngle);
  currentAngle = 0.98*(prevAngle + gyroAngle) + 0.02*(accAngle);
  //Serial.print("=======\nPitch Filter Angle :" );
  //Serial.println(currentAngle);
  //Serial.println("=========");
  //delay(1000);
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  error = Kp*(error) + Ki*(errorSum)*loopTime - Kd*(error)/loopTime;
  //Serial.println(loopTime);

  motorSpeedControl(error);
  //Serial.println(error);

  prevAngle = currentAngle;










  // // set motor power after constraining it
   //motorPower = constrain(motorPower, -255, 255);
   //setMotors(motorPower, motorPower);

  // //delay(1000);
  // //Serial.println(currentAngle);
  // //Serial.println(accAngle);
  // //Serial.println(gyroAngle);
  // //Serial.println();

  
  // Serial.println(error);
  //errorSum = errorSum + error;  
  //errorSum = constrain(errorSum, -300, 300);
  // //calculate output from P, I and D values
  //motorPower = Kp*(error) + Ki*(errorSum)*loopTime - Kd*(currentAngle-prevAngle)/loopTime;
  // Serial.println(motorPower);
  // Serial.println();
  //prevAngle = currentAngle;
  }

   void motorSpeedControl(int angle){
      
         if (angle < 0){
       // Serial.println(angle);
       digitalWrite(rightMotorReversePin, LOW);
       digitalWrite(leftMotorReversePin, LOW);
       digitalWrite(leftMotorForwardPin, HIGH);
       digitalWrite(rightMotorForwardPin,HIGH);
       angle  = abs(angle);
       motorspeed = constrain(angle,0,255);
       //Serial.println(motorspeed);
       //Serial.println();
       analogWrite(Len,motorspeed);
       analogWrite(Ren,motorspeed);
   }
   else if(angle > 0){
      // Serial.println(angle);
       digitalWrite(rightMotorForwardPin, LOW);
       digitalWrite(leftMotorForwardPin, LOW);
       digitalWrite(leftMotorReversePin, HIGH);
       digitalWrite(rightMotorReversePin,HIGH);
       //angle  = abs(angle);
       //Serial.println(angle);
       motorspeed = constrain(angle, 0, 255);
       //Serial.println(motorspeed);
       //Serial.println();
       analogWrite(Len,motorspeed);
       analogWrite(Ren,motorspeed);
   }
   else{
         digitalWrite(rightMotorForwardPin, LOW);
       digitalWrite(leftMotorForwardPin, LOW);
       digitalWrite(leftMotorReversePin, LOW);
       digitalWrite(rightMotorReversePin,LOW);     
   }

 }


