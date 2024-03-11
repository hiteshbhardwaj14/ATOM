void sd_card_setup() {

  if (!SD.begin(chipSelect)) {
    Serial.println(" SD initialization failed!");
    return;
  }

  Serial.println(" SD initialization done.");

  for (int i = 0 ; i < 1000; i++) {
    sprintf(filename, "Data%d.csv", i);
    if (SD.exists(filename)) {
//      Serial.println("File %s exists, incrementing number\n", filename);
    }

    else {
      break;
    }
  }

}
