void assign_RX_channels() {

  crsf.loop();
  //    Serial.print(crsf.getChannel(1)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(2)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(3)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(4)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(5)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(6)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(7)); Serial.print("\t");
  //    Serial.print(crsf.getChannel(8)); Serial.print("\t");

  CRSF_ch[1] = crsf.getChannel(1);
  CRSF_ch[2] = crsf.getChannel(2);
  CRSF_ch[3] = crsf.getChannel(3);
  CRSF_ch[4] = crsf.getChannel(4);
  CRSF_ch[5] = crsf.getChannel(5);
  CRSF_ch[6] = crsf.getChannel(6);
  CRSF_ch[7] = crsf.getChannel(7);
  CRSF_ch[8] = crsf.getChannel(8);
  CRSF_ch[9] = crsf.getChannel(9);
  CRSF_ch[10] = crsf.getChannel(10);
  CRSF_ch[11] = crsf.getChannel(11);
  CRSF_ch[12] = crsf.getChannel(12);

  if (CRSF_ch[2] < 1510 && CRSF_ch[2] > 1490)
  {
    CRSF_ch[2] = 1500;
  }
  if (CRSF_ch[3] < 1510 && CRSF_ch[3] > 1490)
  {
    CRSF_ch[3] = 1500;
  }

  thr_CRSF = map(CRSF_ch[1], 998, 2011, 1000, 2000);
  thr_CRSF_01 = map(CRSF_ch[1], 998, 2011, 0.00, 100.00) / 100.00;
  rol_CRSF = map(CRSF_ch[2], 998, 2011, -100.00, 100.00) / 100.00;
  pit_CRSF = map(CRSF_ch[3], 998, 2011, -100.00, 100.00) / 100.00;


  if (rol_CRSF < 0.1 && rol_CRSF > -0.1)
  {
    rol_CRSF = 0;
  }
  if (pit_CRSF < 0.1 && pit_CRSF > -0.1)
  {
    pit_CRSF = 0;
  }

}
