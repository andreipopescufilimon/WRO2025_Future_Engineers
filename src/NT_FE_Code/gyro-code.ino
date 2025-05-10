#define DRIFT_TEST_TIME 10    // seconds

BMI088 imu(BMI088_ACC_ADDRESS, BMI088_GYRO_ADDRESS);

bool debugGyro = false;

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
  gz = 0;

  while (millis() - start < DRIFT_TEST_TIME * 1000) {
    double now = millis();
    double dt = (now - gyro_last_read_time) * 0.001; // s
    float rate = imu.getGyroscopeZ();                // °/s
    gz += rate * dt;                                 // accumulate degrees
    gyro_last_read_time = now;
  }

  drifts_z = gz / DRIFT_TEST_TIME;  // average °/s
  if (debugGyro) {
    Serial.print("Drift test done! drifts_z = ");
    Serial.print(drifts_z, 6);
    Serial.println(" °/s");
  }

  // reset integration
  gz = 0;
  gyro_last_read_time = millis();
}

void read_gyro_data() {
  // call this frequently (e.g. every loop)
  double now = millis();
  double dt  = (now - gyro_last_read_time) * 0.001; // s
  float rate = imu.getGyroscopeZ();              // °/s
  double corrected = rate - drifts_z;               // drift-compensated
  gz += corrected * dt;                             // integrate to degrees
  gyro_last_read_time = now;

  if (debugGyro) {
    Serial.print("Gyro angle (°): ");
    Serial.println(gz, 4);
  }
}