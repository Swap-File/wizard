
void imu_setup(void) {
  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(IMU_INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  pinMode(A0, INPUT);
  side = digitalRead(A0);
  if (side) {
    Serial.println(F("Left..."));
    mpu.setXGyroOffset(53);
    mpu.setYGyroOffset(6);
    mpu.setZGyroOffset(0);
    mpu.setXAccelOffset(1428);
    mpu.setYAccelOffset(-296);
    mpu.setZAccelOffset(1653);
  } else {
    Serial.println(F("Right..."));
    mpu.setXGyroOffset(87);
    mpu.setYGyroOffset(-51);
    mpu.setZGyroOffset(41);
    mpu.setXAccelOffset(-1535);
    mpu.setYAccelOffset( -103);
    mpu.setZAccelOffset(1207);
  }

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(IMU_INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(IMU_INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
