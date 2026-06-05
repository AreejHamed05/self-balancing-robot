#include <Wire.h>
#include <math.h>

const int MPU_ADDR = 0x68;
float angle = 0, gyroRate = 0, zeroOffset = 0, gyroOffset = 0;
unsigned long lastMicros = 0;
float dt;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Gyro offset
  Serial.println("Keep robot STILL for gyro offset...");
  delay(2000);
  long gyroSum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
    Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 2, true);
    gyroSum += Wire.read() << 8 | Wire.read();
    delay(5);
  }
  gyroOffset = gyroSum / 200.0;

  // Zero angle (hold upright)
  Serial.println("Hold robot UPRIGHT for 2 seconds...");
  delay(2000);
  float sumPitch = 0;
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
    Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 6, true);
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    sumPitch += atan2(ay, az) * 180.0 / M_PI;
    delay(10);
  }
  zeroOffset = sumPitch / 100.0;
  angle = 0;
  Serial.println("Ready. Open Serial Plotter (Tools -> Serial Plotter).");
}

void loop() {
  unsigned long now = micros();
  dt = (now - lastMicros) / 1000000.0;
  lastMicros = now;
  if (dt > 0.02) dt = 0.02;

  // Gyro
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 2, true);
  int16_t gyroRaw = Wire.read() << 8 | Wire.read();
  gyroRate = (gyroRaw - gyroOffset) / 131.0;

  // Accelerometer pitch (YZ)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  Wire.endTransmission(false); Wire.requestFrom(MPU_ADDR, 6, true);
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();
  float accPitch = atan2(ay, az) * 180.0 / M_PI;
  float accAngle = accPitch - zeroOffset;

  // Complementary filter
  float alpha = 0.98;
  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accAngle;

  // Print ONLY the angle value (no labels) – Serial Plotter works best with raw numbers
  Serial.println(angle);

  delay(10);  // 100 Hz update, smooth graph
}
