#define DRIFT_TEST_TIME   10    // seconds

BMI088 imu(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS);

bool debugGyro = true;

void gyro_setup() {
  // 1) Wire + basic init
  Wire.begin();
  imu.initialize();

  imu.setAccScaleRange(RANGE_6G);
  imu.setAccOutputDataRate(ODR_200);       // 200 Hz accel
  imu.setAccPoweMode(ACC_ACTIVE);

  imu.setGyroScaleRange(RANGE_2000);
  imu.setGyroOutputDataRate(ODR_400_BW_47); // 400 Hz, BW=47 Hz
  imu.setGyroPoweMode(GYRO_NORMAL);

  if (!imu.isConnection()) {
    if (debugGyro) {
      Serial.println("BMI088 connection failed!");
    }
    return;
  }

  if (debugGyro) Serial.println("Starting gyro drift calculation...");
  double start = millis();
  gyro_last_read_time = start;
  gx = 0;

  while (millis() - start < DRIFT_TEST_TIME * 1000) {
    double now = millis();
    double dt = (now - gyro_last_read_time) * 0.001; // s
    float rate = imu.getGyroscopeX();                // °/s
    gx += rate * dt;                                 // accumulate degrees
    gyro_last_read_time = now;
  }

  drifts_x = gx / DRIFT_TEST_TIME;  // average °/s
  if (debugGyro) {
    Serial.print("Drift test done! drifts_x = ");
    Serial.print(drifts_x, 6);
    Serial.println(" °/s");
  }

  // reset integration
  gx = 0;
  gyro_last_read_time = millis();
}

void read_gyro_data() {
  // call this frequently (e.g. every loop)
  double now = millis();
  double dt  = (now - gyro_last_read_time) * 0.001; // s
  float rate = imu.getGyroscopeX();              // °/s
  double corrected = rate - drifts_x;               // drift-compensated
  gx += corrected * dt;                             // integrate to degrees
  gyro_last_read_time = now;

  if (debugGyro) {
    Serial.print("Gyro angle (°): ");
    Serial.println(gx, 4);
  }
}