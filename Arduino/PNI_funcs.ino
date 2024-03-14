/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PNI FUNCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void pni_setup() {

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  pinMode(SSN, OUTPUT);
  pinMode(DRDY, INPUT);
  digitalWrite(SSN, HIGH);
  delay(1);
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  delay(50);
  comm_seq(); // setup code
  attachInterrupt(DRDY, measure, RISING);

}

void comm_seq() {
  //Set the Cycle Count Registers
  digitalWrite(SSN, LOW);
  SPI.transfer(0x04);

  SPI.transfer(0x00);
  SPI.transfer(0xC8);
  SPI.transfer(0x00);
  SPI.transfer(0xC8);
  SPI.transfer(0x00);
  SPI.transfer(0xC8);

  digitalWrite(SSN, HIGH);

  //Setting the CMM Update Rate with TMRC
  digitalWrite(SSN, LOW);
  SPI.transfer(0x0B);
  SPI.transfer(0x92);
  digitalWrite(SSN, HIGH);

  //Initiate Continuous Measurement Mode
  digitalWrite(SSN, LOW);
  SPI.transfer(0x01);

  SPI.transfer(0x79);
  digitalWrite(SSN, HIGH);

}

void measure() {
  digitalWrite(SSN, LOW);
  SPI.transfer(0xA4);
  mag_x = ((long)(signed char)SPI.transfer(0)) << 16;
  mag_x = mag_x | (long) SPI.transfer(0) << 8;
  mag_x = mag_x | (long) SPI.transfer(0);
  mag_y = ((long)(signed char)SPI.transfer(0)) << 16;
  mag_y = mag_y | (long) SPI.transfer(0) << 8;
  mag_y = mag_y | (long) SPI.transfer(0);
  mag_z = ((long)(signed char)SPI.transfer(0)) << 16;
  mag_z = mag_z | (long) SPI.transfer(0) << 8;
  mag_z = mag_z | (long) SPI.transfer(0);
  digitalWrite(SSN, HIGH);
}

void mag_calib()
{
  mag_temp[0] = mag_x;
  mag_temp[1] = mag_y;
  mag_temp[2] = mag_z;
  for (int jj = 0; jj < 3; jj++) {
    if (mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
    if (mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
  }
  // Hard iron correction
  mag_offset[0]  = (mag_max[0] + mag_min[0]) / 2;
  mag_offset[1]  = (mag_max[1] + mag_min[1]) / 2;
  mag_offset[2]  = (mag_max[2] + mag_min[2]) / 2;
  // Get soft iron correction estimate
  avg_delta[0]  = (mag_max[0] - mag_min[0]) / 2;
  avg_delta[1]  = (mag_max[1] - mag_min[1]) / 2;
  avg_delta[2]  = (mag_max[2] - mag_min[2]) / 2;
  avg_delta_xyz = (avg_delta[0] + avg_delta[1] + avg_delta[2]) / 3;
  mag_scale[0] = avg_delta_xyz / avg_delta[0];
  mag_scale[1] = avg_delta_xyz / avg_delta[1];
  mag_scale[2] = avg_delta_xyz / avg_delta[2];

}

void print_calib_data()
{
  Serial.print(" mag_x: "); Serial.print(mag_x);
  Serial.print(" mag_y: "); Serial.print(mag_y);
  Serial.print(" mag_z: "); Serial.print(mag_z);
  Serial.print(" max_x: "); Serial.print(mag_max[0]);
  Serial.print(" max_y: "); Serial.print(mag_max[1]);
  Serial.print(" max_z: "); Serial.print(mag_max[2]);
  Serial.print(" min_x: "); Serial.print(mag_min[0]);
  Serial.print(" min_y: "); Serial.print(mag_min[1]);
  Serial.print(" min_z: "); Serial.print(mag_min[2]);
  Serial.print(" mag_offset_x: "); Serial.print(mag_offset[0]);
  Serial.print(" mag_offset_y: "); Serial.print(mag_offset[1]);
  Serial.print(" mag_offset_z: "); Serial.print(mag_offset[2]);
  Serial.print(" mag_scale_x: "); Serial.print(mag_scale[0]);
  Serial.print(" mag_scale_y: "); Serial.print(mag_scale[1]);
  Serial.print(" mag_scale_z: "); Serial.print(mag_scale[2]);
  Serial.println("\t");
}
