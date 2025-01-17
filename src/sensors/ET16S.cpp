#include "ET16S.hpp"

ET16S::ET16S() {}

//EDITING NOTES::
//DIAl and WHEEL are backwards left is highest number
// right is lowest number
//Probably flip that

void ET16S::init(){
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	Serial8.flush();
	Serial8.clear();
	//configure flag byte
	channel[16].kind= input_kind::FLAG;
	//configure safety switch
	//input_kind three_switch=THREE_SWITCH;
	channel[4].kind=input_kind::THREE_SWITCH;
	//Turn safety on
	channel[4].data=1;
	
	//configure sticks
	//right stick horizontal
	channel[0].kind=input_kind::STICK;
	//right stick vertical
	channel[1].kind=input_kind::STICK;
	//left stick horizontal
	channel[2].kind=input_kind::STICK;
	//right stick vertical
	channel[3].kind=input_kind::STICK;
	//configure remaining channels
	set_config();
	

    
}
void ET16S::read() {
	// Raw data stored in array
	uint8_t m_inputRaw[ET16S_PACKET_SIZE]= {0};
	if(Serial8.available()<50){
		return;
	}
	while(Serial8.peek()!= 0x0f){
		Serial8.read();
	}
	for(int i=0; i<ET16S_PACKET_SIZE;i++){
		m_inputRaw[i]=Serial8.read();
	}

	//format raw data
	format_raw(m_inputRaw);
	//set joysticks data
	channel[0].data=map_raw(channel[0]);
	channel[1].data=map_raw(channel[1]);
	channel[2].data=map_raw(channel[2]);
	channel[3].data=map_raw(channel[3]);
	//set safety data
	channel[4].data=map_raw(channel[4]);
	//set flag data
	channel[16].data=channel[16].raw_format;
	//set remaining data
	set_channel_data();
	//Check flag byte for disconnect
	test_connection();
	
	//print_raw_bin(m_inputRaw);
	//print_format_bin(16);
	//print();
	//print_raw();

		
}
void ET16S::print(){
	for (int i=0; i<=15; i++){
		Serial.print(channel[i].data);
		Serial.print(" ");
	}
	Serial.println();
}
void ET16S::print_raw() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT; i++){
		Serial.printf("%.3u ", channel[i].raw_format);
	}
	Serial.println(); 
}
void ET16S::print_raw_bin(uint8_t m_inputRaw[ET16S_PACKET_SIZE]){
	for(int i=0; i<ET16S_PACKET_SIZE; i++){
		for (int ii = 0; ii <= 7; ii++) {
			int k = m_inputRaw[i] >> ii;
			if (k & 1)
				Serial.print("1");
			else
				Serial.print("0");
		}
		Serial.printf(" ");
	}
	Serial.println();
}
void ET16S::print_format_bin(int channel_num){
	if(channel_num >16 || channel_num<0){
		Serial.print("Invalid channel used for print_format_bin. Must be 0-16");
	    return;
	}
		for (int ii = 0; ii <= 16; ii++) {
			int k = channel[channel_num].raw_format	 >> ii;
			if (k & 1)
				Serial.print("1");
			else
				Serial.print("0");
		}
		Serial.printf(" ");
	Serial.println();
}
void ET16S::format_raw(uint8_t m_inputRaw[ET16S_PACKET_SIZE]){
	uint16_t c[22]={0};
	//Channel 1
	c[0]=m_inputRaw[1];
	c[1]= m_inputRaw[2] << 5;
	channel[0].raw_format =((c[0]<<5) | (c[1]<<8));
	channel[0].raw_format>>=5;
	//Channel 2
	c[1]=m_inputRaw[2] >> 3;
	c[2]=m_inputRaw[3] << 2;
	channel[1].raw_format=(c[1]<<5) | (c[2] <<8);
	channel[1].raw_format>>=5;
	//Channel 3
	c[2]=m_inputRaw[3] >> 6;
	c[3]=m_inputRaw[4];
	c[4]=m_inputRaw[5] << 7;
	channel[2].raw_format=(c[2]<<5) | (c[3]<<7) | (c[4]<<8);
	channel[2].raw_format>>=5;
	//Channel 4
	c[4]=m_inputRaw[5] >> 1;
	c[5]=m_inputRaw[6] << 4;
	channel[3].raw_format=(c[4]<<5) | (c[5]<<8);
	channel[3].raw_format>>=5;
	//Channel 5
	c[5]=m_inputRaw[6]>>4;
	c[6]=m_inputRaw[7]<<1;
	channel[4].raw_format=(c[5]<<5) | (c[6]<<8);
	channel[4].raw_format>>=5;
	//Channel 6
	c[6]=m_inputRaw[7]>>7;
	c[7]=m_inputRaw[8];
	c[8]=m_inputRaw[9]<<6;
	channel[5].raw_format=(c[6]<<5) | (c[7]<<6) | (c[8]<<8);
	channel[5].raw_format>>=5;
	//Channel 7
	c[8]=m_inputRaw[9]>>2;
	c[9]=m_inputRaw[10]<<3;
	channel[6].raw_format=(c[8]<<5) | (c[9] <<8);
	channel[6].raw_format>>=5;
	//Channel 8
	c[9]=m_inputRaw[10]>>5;
	c[10]=m_inputRaw[11];
	channel[7].raw_format=(c[9]<<5) | (c[10]<<8);
	channel[7].raw_format>>=5;
	//Channel 9
	c[11]=m_inputRaw[12];
	c[12]=m_inputRaw[13]<<5;
	channel[8].raw_format=(c[11]<<5) | (c[12]<<8);
	channel[8].raw_format>>=5;
	//Channel 10
	c[12]=m_inputRaw[13]>>3;
	c[13]=m_inputRaw[14]<<2;
	channel[9].raw_format=(c[12]<<5) | (c[13]<<8);
	channel[9].raw_format>>=5;
	//Channel 11
	c[13]=m_inputRaw[14]>>6;
	c[14]=m_inputRaw[15];
	c[15]=m_inputRaw[16]<<7;
	channel[10].raw_format=(c[13]<<5) | (c[14]<<7) | (c[15]<<8);
	channel[10].raw_format>>=5;
	//Channel 12
	c[15]=m_inputRaw[16]>>1;
	c[16]=m_inputRaw[17]<<4;
	channel[11].raw_format=(c[15]<<5) | (c[16]<<8);
	channel[11].raw_format>>=5;
	//Channel 13
	c[16]=m_inputRaw[17]>>4;
	c[17]=m_inputRaw[18]<<1;
	channel[12].raw_format=(c[16]<<5) | (c[17]<<8);
	channel[12].raw_format>>=5;
	//Channel 14
	c[17]=m_inputRaw[18]>>7;
	c[18]=m_inputRaw[19];
	c[19]=m_inputRaw[20]<<6;
	channel[13].raw_format=(c[17]<<5) | (c[18]<<6) | (c[19]<<8);
	channel[13].raw_format>>=5;
	//Channel 15
	c[19]=m_inputRaw[20]>>2;
	c[20]=m_inputRaw[21]<<3;
	channel[14].raw_format=(c[19]<<5) | (c[20]<<8);
	channel[14].raw_format>>=5;
	//Channel 16
	c[20]=m_inputRaw[21]>>5;
	c[21]=m_inputRaw[22];
	channel[15].raw_format=(c[20]<<5) | (c[21]<<8);
	channel[15].raw_format>>=5;
	//channel 17 (Error Flag)
	c[22]=m_inputRaw[23];
	channel[16].raw_format=(c[22]<<5);
	channel[16].raw_format>>=5;
}
float ET16S::map_raw(input_channel input){
	float max_out=1;
	float min_out=-1;
    float val = input.raw_format;
	input_kind kind = input.kind;
	switch(kind){
	case input_kind::STICK:
		val=min_out+(val-min_in)*(max_out-min_out)/(max_in-min_in);
		break;
	case input_kind::TWO_SWITCH:
		if(val == max_in){
			val=2;
		}
		else{
			//Represents switch being down/towards user
			val=1;
		}
		break;
	case input_kind::THREE_SWITCH:
		if(val==max_in){
			val=3;
		}
		else if (val == min_in){
			//Represents switch being down/towards user
			val=1;
		}
		else{
			val=2;
		}
		break;
	case input_kind::DIAL:
		//Dial values go towards -1 as it is spun left
		val=max_out-(val-min_in)*(min_out-max_out)/(min_in-max_in);
		
		if(val==-0.f){val=0;} //used to prevent -0 float
		break;
	case input_kind::WHEEL:
		//Wheel values go towards -1 as it is spun down
		val=max_out-(val-min_in)*(min_out-max_out)/(min_in-max_in);
		
		if(val==-0.f){val=0;} //used to prevent -0 float
		break;
	}
	
    return val;
}
void ET16S::set_config(){
	//Valid channel types include STICK,TWO_SWITCH_THREE_SWITCH,
	//DIAL,WHEEL,TRIM,FLAG,INVALID
	//note (trim is not mapped)
		channel[5].kind= input_kind::THREE_SWITCH;
		channel[6].kind= input_kind::THREE_SWITCH;
		channel[7].kind= input_kind::THREE_SWITCH;
		channel[8].kind= input_kind::THREE_SWITCH;
		channel[9].kind= input_kind::TWO_SWITCH;
		channel[10].kind= input_kind::TWO_SWITCH;
		channel[11].kind= input_kind::THREE_SWITCH;
		channel[12].kind= input_kind::WHEEL;
		channel[13].kind= input_kind::DIAL;
		channel[14].kind= input_kind::DIAL;
		channel[15].kind= input_kind::INVALID;
	
		
}
void ET16S::set_channel_data(){
    for(int i=5;i<ET16S_INPUT_VALUE_COUNT;i++){
		channel[i].data=map_raw(channel[i]);
	}
}
void ET16S::test_connection(){
	uint16_t flag_byte=channel[16].data;
	if (flag_byte & ERROR){
    	is_connected=false;
	}
	else{
		is_connected=true;
	}

}
uint8_t ET16S::get_safety(){
	return channel[4].data;
}
float ET16S::get_r_stick_x(){
	return channel[0].data;
}
float ET16S::get_r_stick_y(){
	return channel[1].data;
}
float ET16S::get_l_stick_x(){
	return channel[2].data;
}
float ET16S::get_l_stick_y(){
	return channel[3].data;
}
float ET16S::get_channel_five(){
	return channel[5].data;
}
float ET16S::get_channel_six(){
	return channel[6].data;
}
float ET16S::get_channel_seven(){
	return channel[7].data;
}
float ET16S::get_channel_eight(){
	return channel[8].data;
}
float ET16S::get_channel_nine(){
	return channel[9].data;
}
float ET16S::get_channel_ten(){
	return channel[10].data;
}
float ET16S::get_channel_eleven(){
	return channel[11].data;
}
float ET16S::get_channel_twelve(){
	return channel[12].data;
}
float ET16S::get_channel_thirteen(){
	return channel[13].data;
}
float ET16S::get_channel_fourteen(){
	return channel[14].data;
}
float ET16S::get_channel_fifteen(){
	return channel[15].data;
}
float ET16S::get_connection_status(){
	return is_connected;
}
