#pragma region DR16_Data Implementation

#include "DR16_Data.hpp"

DR16_Data::DR16_Data() {
    Serial5.clear();
	Serial5.begin(100000, SERIAL_8E1_RXINV_TXINV);

    read_state();
}

void DR16_Data::read_state() {
    byte tmp[18];
    Serial5.readBytes(tmp, 18); 
    Serial5.flush();
    Serial5.clear();

    // translates raw data into info about controller actions
    data.r_stick_x = bounded_map(((tmp[1] & 0x07) << 8) | tmp[0], 364, 1684, -1000, 1000) / 1000.0;
    data.r_stick_y = bounded_map(((tmp[2] & 0x3F) << 5) | ((tmp[1] & 0xF8) >> 3), 364, 1684, -1000, 1000) / 1000.0;
    data.l_stick_x = bounded_map((((tmp[4] & 0x01) << 10) | (tmp[3] << 2)) | ((tmp[2] & 0xC0) >> 6), 364, 1684, -1000, 1000) / 1000.0;
    data.l_stick_y = bounded_map(((tmp[5] & 0x0F) << 7) | ((tmp[4] & 0xFE) >> 1), 364, 1684, -1000, 1000) / 1000.0;
    data.wheel = bounded_map((tmp[17] << 8) | tmp[16], 364, 1684, -1000, 1000) / 1000.0;
    
    // Interpreted as int types from raw data but cast to float
    data.l_switch = (float)((tmp[5] & 0xC0) >> 6);
    data.r_switch = (float)((tmp[5] & 0x30) >> 4);
}

float DR16_Data::bounded_map(int value, int in_low, int in_high, int out_low, int out_high){
	/*
		 This is derived from sthe arduino map() function.
	*/
	value = max(min(value, in_high), in_low);
	return (value - in_low) * (out_high - out_low) / (in_high - in_low) + out_low;
}

float DR16_Data::get_r_stick_x() {
    return data.r_stick_x * DATA_SCALAR;
}
    
float DR16_Data::get_r_stick_y() {
    return data.r_stick_y * DATA_SCALAR;
}

float DR16_Data::get_l_stick_x() {
    return data.l_stick_x * DATA_SCALAR;
}

float DR16_Data::get_l_stick_y() {
    return data.l_stick_y * DATA_SCALAR;
}

float DR16_Data::get_wheel() {
    return data.wheel * DATA_SCALAR
}

float DR16_Data::get_l_switch() {
    return data.l_switch;
}

float DR16_Data::get_r_switch() {
    return data.r_switch;
}