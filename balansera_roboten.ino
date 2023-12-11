#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); //  biblioteket till accelerometer

#define ML_Ctrl 4     // define the direction control pin of B motor
#define ML_PWM 5   //define the PWM control pin of B motor

const double setpoint = 0.0; // Setpoint for balancing angle (adjust as needed)
const double Kp = 5.0;        // Proportional gain (adjust as needed)
const double Ki = 0.0;        // Integral gain (adjust as needed)
const double Kd = 0.0;        // Derivative gain (adjust as needed)

double prevError = 0;
double integral = 0;

void setup() {
  pinMode(ML_PWM, OUTPUT);
  pinMode(ML_Ctrl, OUTPUT);
  
  accel.begin();
  accel.setRange(ADXL345_RANGE_2_G); // Adjust the range as needed
    Serial.begin(9600);  //Den är till för att sätta igång accelerometer
   if(!accel.begin())
   {
      Serial.println("No valid sensor found");
      while(1);
   }
}

void loop() {
  sensors_event_t event;
  accel.getEvent(&event);
  double angle = atan2(event.acceleration.x, event.acceleration.z) * 180 / M_PI;

  double error = angle - setpoint;
  integral += error;
  double derivative = error - prevError;

  double pwmOutput = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds using the PWM values
  int motorSpeed = 100; // Set a base motor speed
  int motorSpeedLeft = motorSpeed + pwmOutput;


  // Ensure motor speeds are within limits
  motorSpeedLeft = constrain(motorSpeedLeft, -255, 255);
  
   accel.getEvent(&event); //Skriver ut x axeln,y axeln och z axeln i terminalen
   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");
   Serial.println("m/s^2 ");
   delay(500);


  // Control the motors
  if (motorSpeedLeft > 0) {
    analogWrite(ML_PWM, motorSpeedLeft);
    digitalWrite(ML_Ctrl, HIGH); // Forward
  } else {
    analogWrite(ML_PWM, -motorSpeedLeft);
    digitalWrite(ML_Ctrl, LOW); // Reverse  
  }

  prevError = error;
}
