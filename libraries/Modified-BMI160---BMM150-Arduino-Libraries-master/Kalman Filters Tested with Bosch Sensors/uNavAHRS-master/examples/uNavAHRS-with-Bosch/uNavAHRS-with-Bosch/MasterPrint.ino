void masterSerialPrint(){
  // Set textLength to the number of parameters to print * 31
  int  textLength = 3 * 31;
  char text[textLength];

  double R2D = 180.0 / PI;
  double D2R = PI / 180.0;

  char pitchText[30];
  char rollText[30];
  char yawText[30];
  
  dtostrf(ypr[0], 10, 4, yawText);
  dtostrf(ypr[1], 10, 4, pitchText);
  dtostrf(ypr[2], 10, 4, rollText);
   // Create single text parameter and print it
  snprintf(text, textLength, "%s,%s,%s",  rollText, pitchText, yawText);
  Serial.println(text);
  //cout.send_now();  
} 
