#include "ET16S.hpp"

ET16S::ET16S() {}

//EDITING NOTES::
//DIAl and WHEEL are backwards left is highest number
// right is lowest number
//Probably flip that
// Need to do flag checks [3rd bit is kinda connected]
// [3rd and 4th bit is definitly not connected]
// Parity check for exception handling

void ET16S::init(){
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	Serial8.flush();
	Serial8.clear();

	//configure safety switch
	//input_kind three_switch=THREE_SWITCH;
	channel[4].kind=set_channel_kind(input_kind::THREE_SWITCH);
	//Turn safety on
	channel[4].data=0;
	
	//configure sticks
	//right stick horizontal
	channel[0].kind=set_channel_kind(input_kind::STICK);
	//right stick vertical
	channel[1].kind=set_channel_kind(input_kind::STICK);
	//left stick horizontal
	channel[2].kind=set_channel_kind(input_kind::STICK);
	//right stick vertical
	channel[3].kind=set_channel_kind(input_kind::STICK);
	//configure remaining channels
	set_config();
	

    
}
void ET16S::read() {
	uint8_t m_inputRaw[ET16S_PACKET_SIZE]= {0};
	//Serial8.readBytes(m_inputRaw,ET16S_PACKET_SIZE);
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
	//set remaining data
	set_channel_data();

	//print_raw_bin(m_inputRaw)
	//print_format_bin();
	print();
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
	for (int i = 0; i < ET16S_PACKET_SIZE; i++){
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
void ET16S::print_format_bin(){
		for (int ii = 0; ii <= 15; ii++) {
			int k = channel[0].raw_format	 >> ii;
			if (k & 1)
				Serial.print("1");
			else
				Serial.print("0");
		}
		Serial.printf(" ");
	Serial.println();
}
void ET16S::format_raw(uint8_t m_inputRaw[ET16S_PACKET_SIZE]){
	uint16_t c[21]={0};
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

	
}
float ET16S::map_raw(input_channel input){
	float max_out=1;
	float min_out=-1;
    float val = input.raw_format;
	input_kind kind = input.kind;
	if(kind == STICK){
		val=min_out+(val-min_in)*(max_out-min_out)/(max_in-min_in);
	}
	else if(kind == TWO_SWITCH){
		if(val == max_in){
			val=2;
		}
		else{
			val=1;
		}
	}
	else if (kind == THREE_SWITCH){
		if(val==max_in){
			val=3;
		}
		else if (val==min_in){
			val=1;
		}
		else
			val=2;
	}
	else if (kind == DIAL){
		val=min_out+(val-min_in)*(max_out-min_out)/(max_in-min_in);
	}
	else if (kind == WHEEL){
		val=min_out+(val-min_in)*(max_out-min_out)/(max_in-min_in);
	}
    return val;
}
input_kind ET16S::set_channel_kind(input_kind kind){
	return kind;
}
void ET16S::set_config(){
		channel[5].kind= input_kind::THREE_SWITCH;
		channel[6].kind= input_kind::THREE_SWITCH;
		channel[7].kind= input_kind::THREE_SWITCH;
		channel[8].kind= input_kind::THREE_SWITCH;
		channel[9].kind= input_kind::TWO_SWITCH;
		channel[10].kind= input_kind::TWO_SWITCH;
		channel[11].kind= input_kind::THREE_SWITCH;
		channel[12].kind= input_kind::WHEEL;
		channel[13].kind= input_kind::DIAL;
		channel[14].kind= input_kind::INVALID;
		channel[15].kind= input_kind::FLAG;
	
		
}
void ET16S::set_channel_data(){
    for(int i=5;i<ET16S_INPUT_VALUE_COUNT;i++){
		channel[i].data=map_raw(channel[i]);
	}
}
