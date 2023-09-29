void setup() {
  Serial5.begin(100000);
  Serial5.clear();
}

void loop() {
  // reads 18 bytes of data into tmp.
  byte tmp[18];
	Serial5.readBytes(tmp, 18); 
  
  // translates data into info about controller actions
  float r_stick_x = bounded_map(((tmp[1] & 0x07) << 8) | tmp[0], 364, 1684, -1000, 1000) / 1000.0;
	float r_stick_y = bounded_map(((tmp[2] & 0x3F) << 5) | ((tmp[1] & 0xF8) >> 3), 364, 1684, -1000, 1000) / 1000.0;
	float l_stick_x = bounded_map((((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6), 364, 1684, -1000, 1000) / 1000.0;
	float l_stick_y = bounded_map(((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1), 364, 1684, -1000, 1000) / 1000.0;
	float wheel = bounded_map((tmp[17] << 8) | tmp[16], 364, 1684, -1000, 1000) / 1000.0;
	int l_switch = (tmp[5] & 0xC0) >> 6;
	int r_switch = (tmp[5] & 0x30) >> 4;

  
  for(int i = 0; i < 18; i++) {
    Serial.print(tmp[i]);
    Serial.print("\t");
  }
  Serial.println();

}

float bounded_map(int value, int in_low, int in_high, int out_low, int out_high){
	/*
		 This is derived from sthe arduino map() function.
	*/
	value = max(min(value, in_high), in_low);
	return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

