#include <Wire.h>
#include <math.h>
#include <PID_v2.h> // <--- Include the PID library

// ========== Motor Pins ==========
const int ENA_R = 5, IN1_R = 7, IN2_R = 8;
const int ENB_L = 6, IN3_L = 9, IN4_L = 10;
const int MPU_ADDR = 0x68;

// ========== PID Variables ==========
double Setpoint = 0;   // We want the angle to be 0 degrees
double Input;          // The current angle of the robot
double Output;         // The output computed by the PID library

// Tuning parameters (Optimized for the fixed sensor speed)
double Kp = 55;
double Ki = 20;
double Kd = 3.5;  

// Create the PID instance (DIRECT means if it falls forward, output goes positive)
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int deadband = 65;        // Minimum PWM needed to overcome motor friction
int outputSign = -1;       // Change to -1 if wheels push the same direction as tilt

// ========== Global Variables ==========
float angle = 0, gyroRate = 0, zeroOffset = 0, gyroOffset = 0;
unsigned long lastMicros = 0;
float dt;
int motorOutput;

void setup() {
  Serial.begin(115200);
  pinMode(ENA_R, OUTPUT); pinMode(IN1_R, OUTPUT); pinMode(IN2_R, OUTPUT);
  pinMode(ENB_L, OUTPUT); pinMode(IN3_L, OUTPUT); pinMode(IN4_L, OUTPUT);
 
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00); // Wake up MPU6050
  Wire.endTransmission();
  delay(100);
 
  // ===============================================
  // PID LIBRARY CONFIGURATION
  // ===============================================
  myPID.SetMode(AUTOMATIC);         // Turn on the PID
  myPID.SetSampleTime(10);          // CRITICAL: Set loop to 100Hz (10ms) for balancing!
  myPID.SetOutputLimits(-205, 205); // Tell PID the max PWM range
 
  // Gyro offset calibration
  Serial.println("Keep robot STILL for gyro offset...");
  delay(2000);
  long gyroSum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR);
   
    Wire.write(0x43); // <--- EDITED: Changed from 0x45 to 0x43 (Gyro X)
   
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    gyroSum += Wire.read() << 8 | Wire.read();
    delay(5);
  }
  gyroOffset = gyroSum / 200.0;
  Serial.print("Gyro X offset: "); Serial.println(gyroOffset);
 
  // Accelerometer zero angle calibration
  Serial.println("Hold robot UPRIGHT for 2 seconds...");
  delay(2000);
  float sumPitch = 0;
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    int16_t ax = Wire.read() << 8 | Wire.read();  
    int16_t ay = Wire.read() << 8 | Wire.read();  
    int16_t az = Wire.read() << 8 | Wire.read();  
    sumPitch += atan2(ay, az) * 180.0 / M_PI;
    delay(10);
  }
  zeroOffset = sumPitch / 100.0;
  Serial.print("Zero offset (pitch YZ): "); Serial.println(zeroOffset);
  angle = 0;
  Serial.println("Ready! Robot will balance now.");
  delay(1000);
 
  lastMicros = micros();
}

void loop() {
  // 1. Calculate Time Step (dt) for the Complementary Filter
  unsigned long now = micros();
  dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;
  if (dt > 0.02) dt = 0.02;
 
  // 2. Read Gyro X
  Wire.beginTransmission(MPU_ADDR);
 
  Wire.write(0x43); // <--- EDITED: Changed from 0x45 to 0x43 (Gyro X)
 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gyroRaw = Wire.read() << 8 | Wire.read();
  gyroRate = (gyroRaw - gyroOffset) / 131.0;
 
  // 3. Read Accelerometer Y and Z
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t ax = Wire.read() << 8 | Wire.read();  
  int16_t ay = Wire.read() << 8 | Wire.read();  
  int16_t az = Wire.read() << 8 | Wire.read();  
 
  // 4. Calculate Angle (Complementary Filter)
  float accPitch = atan2(ay, az) * 180.0 / M_PI;
  float accAngle = accPitch - zeroOffset;
  float alpha = 0.98;
  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accAngle;
 
  // 5. UPDATE PID LIBRARY =======================================
  Input = angle; // Feed the current angle into the PID library
 
  if (abs(angle) > 45) {
    // Safety Cutoff: If falling completely over, turn off PID and motors
    myPID.SetMode(MANUAL);
    motorOutput = 0;
  } else {
    // If upright, ensure PID is on and compute
    if (myPID.GetMode() == MANUAL) myPID.SetMode(AUTOMATIC);
    myPID.Compute(); // <--- Library does all the P, I, and D math right here
   
    // Apply Output Sign
    motorOutput = outputSign * Output;
  }
 
  // 6. Motor Feed-forward Deadband & Limits ===================
  if (motorOutput > 2.5) motorOutput += deadband;
  else if (motorOutput < -2.5) motorOutput -= deadband;
  else motorOutput = 0;
 
  motorOutput = constrain(motorOutput, -255, 255);
 
  // 7. Apply to Motors ========================================
  if (motorOutput > 0) {
    digitalWrite(IN1_R, HIGH); digitalWrite(IN2_R, LOW);
    digitalWrite(IN3_L, LOW);  digitalWrite(IN4_L, HIGH);
    analogWrite(ENA_R, motorOutput);
    analogWrite(ENB_L, motorOutput);
  } else if (motorOutput < 0) {
    digitalWrite(IN1_R, LOW);  digitalWrite(IN2_R, HIGH);
    digitalWrite(IN3_L, HIGH); digitalWrite(IN4_L, LOW);
    analogWrite(ENA_R, abs(motorOutput));
    analogWrite(ENB_L, abs(motorOutput));
  } else {
    digitalWrite(IN1_R, LOW); digitalWrite(IN2_R, LOW);
    digitalWrite(IN3_L, LOW); digitalWrite(IN4_L, LOW);
    analogWrite(ENA_R, 0);
    analogWrite(ENB_L, 0);
  }
 
 // 8. Debug Print for Serial Plotter (Every 50ms)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 50) {
    Serial.print("Angle:");
    Serial.print(angle);
    Serial.print(",");            // <--- The comma is what makes the plotter work!
    Serial.print("Output:");
    Serial.println(motorOutput);
    lastPrint = millis();
  }
}
