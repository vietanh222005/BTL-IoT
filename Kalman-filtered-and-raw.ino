#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// =================== KHAI BÁO ===================
Adafruit_MPU6050 mpu;

// Biến cho bộ lọc Kalman (đơn giản cho từng trục)
float kalmanX = 0, kalmanY = 0, kalmanZ = 0;
float P_kx = 1, P_ky = 1, P_kz = 1;
float Q = 0.05;  // noise quá trình
float R = 0.5;   // noise đo

// =================== KALMAN FILTER ===================
float kalmanUpdate(float measurement, float &estimate, float &P_k) {
  // Bước dự đoán
  P_k += Q;

  // Bước cập nhật
  float K = P_k / (P_k + R);
  estimate = estimate + K * (measurement - estimate);
  P_k = (1 - K) * P_k;

  return estimate;
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  Serial.println("🚀 MPU6050 Kalman Filter Test");

  if (!mpu.begin()) {
    Serial.println("❌ Không tìm thấy MPU6050!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(1000);
  Serial.println("✅ Bắt đầu đo...");
  Serial.println("ax_raw,ax_filt,ay_raw,ay_filt,az_raw,az_filt,total_raw,total_filt");
}

// =================== LOOP ===================
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Raw
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  // Kalman
  float ax_f = kalmanUpdate(ax, kalmanX, P_kx);
  float ay_f = kalmanUpdate(ay, kalmanY, P_ky);
  float az_f = kalmanUpdate(az, kalmanZ, P_kz);

  // Tính tổng vector gia tốc
  float total_raw = sqrt(ax * ax + ay * ay + az * az);
  float total_filt = sqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f);

  // Xuất dữ liệu ra Serial Plotter (Arduino IDE)
  // 👉 Chọn Tools → Serial Plotter để xem đồ thị
  Serial.print(ax); Serial.print(",");
  Serial.print(ax_f); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(ay_f); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(az_f); Serial.print(",");
  Serial.print(total_raw); Serial.print(",");
  Serial.println(total_filt);

  delay(100);
}
