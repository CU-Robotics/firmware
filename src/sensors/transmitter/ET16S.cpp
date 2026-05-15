#include "ET16S.hpp"
#include "sensors/RefSystem.hpp"
#include "comms/data/sendable.hpp"
#include "state.hpp"

// Allocate the static variables in memory
ET16S* ET16S::instance = nullptr;
DMAMEM uint8_t ET16S::dma_buffer_a[32] __attribute__((aligned(32)));
DMAMEM uint8_t ET16S::dma_buffer_b[32] __attribute__((aligned(32)));
volatile uint8_t* ET16S::active_buffer = nullptr;
volatile uint8_t* ET16S::dma_target_buffer = nullptr;

ET16S::ET16S(const Cfg::ET16S& config) : config(config) { }

void ET16S::init() {

	//  Hook the instance pointer to THIS object
    instance = this;
	
	Serial8.begin(100000, SERIAL_8E1_RXINV_TXINV);
	Serial8.flush();
	Serial8.clear();
	//configure safety switch;
	channel[4].kind = InputKind::THREE_SWITCH;
	channel[4].id = ChannelId::SWITCH_A;
	//Turn safety on
	channel[4].data = static_cast<float>(SwitchPos::FORWARD);

	//configure sticks
	//right stick horizontal
	channel[r_stick_x_num].kind = InputKind::STICK;
	//right stick vertical
	channel[1].kind = InputKind::STICK;
	//left stick vertical
	channel[2].kind = InputKind::STICK;
	//right stick horizontal
	channel[3].kind = InputKind::STICK;
	//configure remaining channels
	set_config();
	setup_edma_channel();
}
void ET16S::setup_edma_channel() {
	// Enable DMA on Serial8 (i.MX RT1060 manual pg 2921)
	LPUART5_BAUD |= LPUART_BAUD_RDMAE; 
	
	// Setup ping-pong buffer pointers
	dma_target_buffer = dma_buffer_a;
    active_buffer = dma_buffer_b;
	
    // Source: The physical memory address of LPUART5's Data Register
    rx_dma.source(LPUART5_DATA); 
    
    // Destination: Our cache-aligned RAM buffer, major loop of 25 bytes
    rx_dma.destinationBuffer(dma_target_buffer, ET16S_PACKET_SIZE); 
    
    // Trigger: Map the LPUART5 RX hardware event through the DMAMUX
    rx_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_LPUART5_RX);
    
    // Interrupts: Fire an ISR only when the major loop (25 bytes) completes
    rx_dma.attachInterrupt(dma_isr_wrapper);
    rx_dma.interruptAtCompletion();
    
    // Arm the DMA channel
    rx_dma.enable();
}
void ET16S::dma_isr_wrapper() {
    if (instance != nullptr) {
        // Route the execution back into the specific object instance
        instance->dma_isr();
    }
}

void ET16S::dma_isr(){
    // Clear the hardware interrupt flag
    rx_dma.clearInterrupt();

    active_buffer = dma_target_buffer;
    // Invalidate the cache for this buffer so the CPU fetches the fresh RAM
    arm_dcache_delete((void*)active_buffer, 32);
	
	// Determine target buffer to read from
	if (dma_target_buffer == dma_buffer_a) {
        dma_target_buffer = dma_buffer_b;
    }
	else {
        dma_target_buffer = dma_buffer_a;
    }
    
    // Update the hardware to point to the newly cleared buffer for the next packet
    rx_dma.destinationBuffer(dma_target_buffer, ET16S_PACKET_SIZE);
	
    // Flag the main loop to process the data
    packet_ready = true; 
}
void ET16S::resync_frame(){
	Serial.print("ET16S reframing (should only be called on startup)");
    rx_dma.disable();
    Serial8.clear();
    
    //  Wait for the frame boundary
    elapsedMillis timeout; 
    
    // We give it 10ms (enough time for ~3 full packets) to find the sync boundary
    while (timeout < 10) {
        if (Serial8.available() > 0) {
            uint8_t c = Serial8.read();
            
            // If the byte we just read was the 0x00 footer, AND the next byte 
            // sitting in the buffer is the 0x0F header, we are perfectly aligned!
            if (c == 0x00 && Serial8.peek() == 0x0F) {
                break; 
            }
        }
    }
    
    // start writing from beginning of buffer
    rx_dma.destinationBuffer(dma_target_buffer, ET16S_PACKET_SIZE);
    
    // next byte should be  0x0F.
    rx_dma.enable();
}
void ET16S::read() {
    if (packet_ready) {
        packet_ready = false; // Reset flag
        if (active_buffer[0] == 0x0F && active_buffer[24] == 0x00) {
			// Data is complete in active buffer
			format_raw((uint8_t*)active_buffer);
			//set flag data
			channel[16].data = channel[16].raw_format;
			//set remaining data
			set_channel_data();
			//Check flag byte for disconnect
			test_connection();

			mode_changed_flag = (get_safety_switch() != prev_safety_switch_pos);
			prev_safety_switch_pos = get_safety_switch();
		}
		else {
			resync_frame();
		}
	}
}

void ET16S::print() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT; i++) {
		Serial.printf("%f ", channel[i].data);
	}

	Serial.println();
}

void ET16S::print_raw() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT; i++) {
		Serial.printf("%.3u ", channel[i].raw_format);
	}

	Serial.println();
}

void ET16S::print_raw_bin(uint8_t m_inputRaw[ET16S_PACKET_SIZE]) {
	for (int i = 0; i < ET16S_PACKET_SIZE; i++) {
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
void ET16S::print_live_data() {
    Serial.printf("=== LIVE ET16S TRANSMITTER DATA ===\n");
    
    const char* mode_str = "UNKNOWN";
    if (is_safety_mode()) {
        mode_str = "SAFETY";
    } else if (is_teensy_mode()) {
        mode_str = "TEENSY";
    } else if (is_hive_mode()) {
        mode_str = "HIVE";
    }

    Serial.printf(" Control Mode: %-7s\n", mode_str);
    
    Serial.println("---------------------------------------");
    Serial.printf(" L Stick : X: %5.2f | Y: %5.2f\n", get_l_stick_x(), get_l_stick_y());
    Serial.printf(" R Stick : X: %5.2f | Y: %5.2f\n", get_r_stick_x(), get_r_stick_y());
    Serial.printf(" L Dial  :    %5.2f | R Dial  :    %5.2f\n", get_l_dial(), get_r_dial());
    Serial.printf(" L Slider:    %5.2f | R Slider:    %5.2f\n", get_l_slider(), get_r_slider());
    Serial.println("---------------------------------------");

    // Lambda to convert the float value into the SwitchPos string
    auto sw_str = [](auto val) -> const char* {
        switch (static_cast<SwitchPos>(static_cast<uint32_t>(val))) {
            case SwitchPos::FORWARD:  return "FORWARD";
            case SwitchPos::BACKWARD: return "BACKWARD";
            case SwitchPos::MIDDLE:   return "MIDDLE";
            default:                  return "INVALID";
        }
    };

    // Print using the lambda and the %-8s padding to prevent text ghosting
    Serial.printf(" SW_B: %-8s | SW_C: %-8s\n", sw_str(get_switch_b()), sw_str(get_switch_c()));
    Serial.printf(" SW_D: %-8s | SW_E: %-8s\n", sw_str(get_switch_d()), sw_str(get_switch_e()));
    Serial.printf(" SW_F: %-8s | SW_G: %-8s\n", sw_str(get_switch_f()), sw_str(get_switch_g()));
    Serial.printf(" SW_H: %-8s |\n", sw_str(get_switch_h()));
}
void ET16S::print_format_bin(int channel_num) {
	if (channel_num > ET16S_INPUT_VALUE_COUNT || channel_num < 0) {
		Serial.print("Invalid channel used for print_format_bin. Must be 0-16");
		return;
	}

	for (int ii = 0; ii <= 16; ii++) {
		int k = channel[channel_num].raw_format >> ii;
		if (k & 1)
			Serial.print("1");
		else
			Serial.print("0");
	}

	Serial.printf(" ");
	Serial.println();
}

void ET16S::format_raw(uint8_t m_inputRaw[ET16S_PACKET_SIZE]) {
	uint16_t c[23] = { 0 };

	//Channel 1
	c[0] = m_inputRaw[1];
	c[1] = m_inputRaw[2] << 5;
	channel[0].raw_format = ((c[0] << 5) | (c[1] << 8));
	channel[0].raw_format >>= 5;

	//Channel 2
	c[1] = m_inputRaw[2] >> 3;
	c[2] = m_inputRaw[3] << 2;
	channel[1].raw_format = (c[1] << 5) | (c[2] << 8);
	channel[1].raw_format >>= 5;

	//Channel 3
	c[2] = m_inputRaw[3] >> 6;
	c[3] = m_inputRaw[4];
	c[4] = m_inputRaw[5] << 7;
	channel[2].raw_format = (c[2] << 5) | (c[3] << 7) | (c[4] << 8);
	channel[2].raw_format >>= 5;

	//Channel 4
	c[4] = m_inputRaw[5] >> 1;
	c[5] = m_inputRaw[6] << 4;
	channel[3].raw_format = (c[4] << 5) | (c[5] << 8);
	channel[3].raw_format >>= 5;

	//Channel 5
	c[5] = m_inputRaw[6] >> 4;
	c[6] = m_inputRaw[7] << 1;
	channel[4].raw_format = (c[5] << 5) | (c[6] << 8);
	channel[4].raw_format >>= 5;

	//Channel 6
	c[6] = m_inputRaw[7] >> 7;
	c[7] = m_inputRaw[8];
	c[8] = m_inputRaw[9] << 6;
	channel[5].raw_format = (c[6] << 5) | (c[7] << 6) | (c[8] << 8);
	channel[5].raw_format >>= 5;

	//Channel 7
	c[8] = m_inputRaw[9] >> 2;
	c[9] = m_inputRaw[10] << 3;
	channel[6].raw_format = (c[8] << 5) | (c[9] << 8);
	channel[6].raw_format >>= 5;

	//Channel 8
	c[9] = m_inputRaw[10] >> 5;
	c[10] = m_inputRaw[11];
	channel[7].raw_format = (c[9] << 5) | (c[10] << 8);
	channel[7].raw_format >>= 5;

	//Channel 9
	c[11] = m_inputRaw[12];
	c[12] = m_inputRaw[13] << 5;
	channel[8].raw_format = (c[11] << 5) | (c[12] << 8);
	channel[8].raw_format >>= 5;

	//Channel 10
	c[12] = m_inputRaw[13] >> 3;
	c[13] = m_inputRaw[14] << 2;
	channel[9].raw_format = (c[12] << 5) | (c[13] << 8);
	channel[9].raw_format >>= 5;

	//Channel 11
	c[13] = m_inputRaw[14] >> 6;
	c[14] = m_inputRaw[15];
	c[15] = m_inputRaw[16] << 7;
	channel[10].raw_format = (c[13] << 5) | (c[14] << 7) | (c[15] << 8);
	channel[10].raw_format >>= 5;

	//Channel 12
	c[15] = m_inputRaw[16] >> 1;
	c[16] = m_inputRaw[17] << 4;
	channel[11].raw_format = (c[15] << 5) | (c[16] << 8);
	channel[11].raw_format >>= 5;

	//Channel 13
	c[16] = m_inputRaw[17] >> 4;
	c[17] = m_inputRaw[18] << 1;
	channel[12].raw_format = (c[16] << 5) | (c[17] << 8);
	channel[12].raw_format >>= 5;

	//Channel 14
	c[17] = m_inputRaw[18] >> 7;
	c[18] = m_inputRaw[19];
	c[19] = m_inputRaw[20] << 6;
	channel[13].raw_format = (c[17] << 5) | (c[18] << 6) | (c[19] << 8);
	channel[13].raw_format >>= 5;

	//Channel 15
	c[19] = m_inputRaw[20] >> 2;
	c[20] = m_inputRaw[21] << 3;
	channel[14].raw_format = (c[19] << 5) | (c[20] << 8);
	channel[14].raw_format >>= 5;

	//Channel 16
	c[20] = m_inputRaw[21] >> 5;
	c[21] = m_inputRaw[22];
	channel[15].raw_format = (c[20] << 5) | (c[21] << 8);
	channel[15].raw_format >>= 5;

	//channel 17 (Error Flag)
	c[22] = m_inputRaw[23];
	channel[16].raw_format = (c[22] << 5);
	channel[16].raw_format >>= 5;
}

float ET16S::map_raw(InputChannel input) {
	float val = input.raw_format;
	InputKind kind = input.kind;

	switch (kind) {
	case InputKind::STICK: {
		val = map_math(val,-1,1);
		break;
	}
	case InputKind::TWO_SWITCH: {
		if (val == max_in) {
			val = static_cast<float>(SwitchPos::BACKWARD);
		} else {
			val = static_cast<float>(SwitchPos::FORWARD);
		}
		break;
	}
	case InputKind::THREE_SWITCH: {
		if (val == max_in) {
			val = static_cast<float>(SwitchPos::BACKWARD);
		} else if (val == min_in) {
			val = static_cast<float>(SwitchPos::FORWARD);
		} else {
			val = static_cast<float>(SwitchPos::MIDDLE);
		}
		break;
	}
	case InputKind::DIAL: {
		//Dial values go towards -1 as it is spun left
		val = -map_math(val,-1,1);

		if (val == -0.f) { val = 0; } //used to prevent -0 float
		break;
	}
	case InputKind::SLIDER: {
		//Wheel values go towards -1 as it is spun down
		val = -map_math(val,-1,1);

		if (val == -0.f) { val = 0; } //used to prevent -0 float
		break;
	}
	case InputKind::TRIM: {
		break;
	}
	case InputKind::INVALID: {
		break;
	}
	case InputKind::FLAG: {
		break;
	}
	}

	return val;
}

float ET16S::map_math(float val,float min_out, float max_out){
	// maps input value from -1 to 1
	val = min_out + (val - min_in) * (max_out - min_out) / (max_in - min_in);
	//ensure val is in range 
	if(val>max_out){
		val=max_out;
	}
	else if (val<min_out){
		val=min_out;
	}
	return val;
}

void ET16S::set_config() {
	//Valid channel types include STICK,TWO_SWITCH_THREE_SWITCH,
	//DIAL,SLIDER,TRIM,FLAG,INVALID
	//note (trim is not mapped)
	channel[r_stick_x_num].id = ChannelId::R_STICK_X;
	channel[r_stick_y_num].id = ChannelId::R_STICK_Y;
	channel[l_stick_x_num].id = ChannelId::L_STICK_X;
	channel[l_stick_y_num].id = ChannelId::L_STICK_Y;
	
	channel[switch_b_num].id = ChannelId::SWITCH_B;
	channel[switch_c_num].id = ChannelId::SWITCH_C;
	channel[switch_d_num].id = ChannelId::SWITCH_D;
	channel[switch_e_num].id = ChannelId::SWITCH_E;
	channel[switch_f_num].id = ChannelId::SWITCH_F;
	channel[switch_g_num].id = ChannelId::SWITCH_G;
	channel[switch_h_num].id = ChannelId::SWITCH_H;
	channel[r_slider_num].id = ChannelId::R_SLIDER;
	channel[l_slider_num].id = ChannelId::L_SLIDER;
	channel[r_dial_num].id = ChannelId::R_DIAL;
	channel[l_dial_num].id = ChannelId::L_DIAL;
	channel[16].id = ChannelId::FLAG;

	for (int i = 5; i < ET16S_INPUT_VALUE_COUNT; i++){
		ChannelId id = channel[i].id;
		switch(id){
		case ChannelId::L_STICK_X:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::L_STICK_Y:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::R_STICK_X:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::R_STICK_Y:
			channel[i].kind = InputKind::STICK;
			break;
		case ChannelId::L_DIAL:
			channel[i].kind = InputKind::DIAL;
			break;
		case ChannelId::R_DIAL:
			channel[i].kind = InputKind::DIAL;
			break;
		case ChannelId::SWITCH_A:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_B:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_C:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_D:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_E:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_F:
			channel[i].kind = InputKind::TWO_SWITCH;
			break;
		case ChannelId::SWITCH_G:
			channel[i].kind = InputKind::THREE_SWITCH;
			break;
		case ChannelId::SWITCH_H:
			channel[i].kind = InputKind::TWO_SWITCH;
			break;
		case ChannelId::L_SLIDER:
			channel[i].kind = InputKind::SLIDER;
			break;
		case ChannelId::R_SLIDER:
			channel[i].kind = InputKind::SLIDER;
			break;
		case ChannelId::TRIM_1:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_2:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_3:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_4:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_5:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::TRIM_6:
			channel[i].kind = InputKind::TRIM;
			break;
		case ChannelId::UNMAPPED:
			channel[i].kind = InputKind::INVALID;
			break;
		case ChannelId::FLAG:
			channel[i].kind = InputKind::FLAG;
			break;
			
	   }
	}
}

float ET16S::average_channel(float sample, int channel_index) {
	// set the sample
	// circular buffer 
	int channel_value_index = value_indices[channel_index];
	channel_values_circular_buf[channel_index][channel_value_index] = sample;
	value_indices[channel_index] = (channel_value_index + 1) % AVERAGE_SAMPLE_COUNT;
	
	float sum = 0;
	float average = 0;
	for (int i = 0; i < AVERAGE_SAMPLE_COUNT; i++) {
		sum += channel_values_circular_buf[channel_index][i];
	}

	average = sum / AVERAGE_SAMPLE_COUNT;
	return average;
}

void ET16S::set_channel_data() {
	for (int i = 0; i < ET16S_INPUT_VALUE_COUNT;i++) {
		// we only want to average the stick, dial, and slider channels
		// the transmitter's output for continuous data is a bit noisy, enough to move the yaw back and forth
		// we average the last few samples to smoothen the output
		if (channel[i].kind == InputKind::STICK || channel[i].kind == InputKind::DIAL || channel[i].kind == InputKind::SLIDER) {
			channel[i].data = average_channel(map_raw(channel[i]), i);
		} else {
			channel[i].data = map_raw(channel[i]);
		}
	}
}

void ET16S::test_connection() {		
	uint16_t flag_byte = channel[16].raw_format;
	if (flag_byte & DISCONNECT) {
		is_connect = false;
	} else {
		is_connect = true;
	}
}

SwitchPos ET16S::get_safety_switch() {
	return static_cast<SwitchPos> (channel[4].data);
}

float ET16S::get_r_stick_x() {
	return channel[r_stick_x_num].data;
}

float ET16S::get_r_stick_y() {
	return channel[r_stick_y_num].data;
}

float ET16S::get_l_stick_x() {
	return channel[l_stick_x_num].data;
}

float ET16S::get_l_stick_y() {
	return channel[l_stick_y_num].data;
}

SwitchPos ET16S::get_switch_b(){
	return static_cast<SwitchPos> (channel[switch_b_num].data);
}

SwitchPos ET16S::get_switch_c(){
	return static_cast<SwitchPos> (channel[switch_c_num].data);
}

SwitchPos ET16S::get_switch_d(){
	return static_cast<SwitchPos> (channel[switch_d_num].data);
}

SwitchPos ET16S::get_switch_e(){
	return static_cast<SwitchPos> (channel[switch_e_num].data);
}

SwitchPos ET16S::get_switch_f(){
	return static_cast<SwitchPos> (channel[switch_f_num].data);
}

SwitchPos ET16S::get_switch_g(){
	return static_cast<SwitchPos> (channel[switch_g_num].data);
}

SwitchPos ET16S::get_switch_h(){
	return static_cast<SwitchPos> (channel[switch_h_num].data);
}

float ET16S::get_l_slider(){
	return channel[l_slider_num].data;
}

float ET16S::get_r_slider(){
	return channel[r_slider_num].data;
}

float ET16S::get_trim_one(){
	return channel[trim_one_num].data;
}

float ET16S::get_trim_two(){
	return channel[trim_two_num].data;
}

float ET16S::get_trim_three(){
	return channel[trim_three_num].data;
}

float ET16S::get_trim_four(){
	return channel[trim_four_num].data;
}

float ET16S::get_trim_five(){
	return channel[trim_five_num].data;
}

float ET16S::get_trim_six(){
	return channel[trim_six_num].data;
}

float ET16S::get_l_dial(){
	return channel[l_dial_num].data;
}

float ET16S::get_r_dial(){
	return channel[r_dial_num].data;
}

float ET16S::get_channel_data(int chan_num){
	// Will return nothing if an incorrect channel  number is given
	if ((chan_num < 0) || (chan_num > 16)){		
		return {};
	}
	return channel[chan_num].data;
}

bool ET16S::is_connected() {
	return is_connect;
}

SwitchPos ET16S::get_l_switch(){
	return static_cast<SwitchPos> (get_safety_switch());
}

SwitchPos ET16S::get_r_switch(){
	return static_cast<SwitchPos> (channel[switch_d_num].data);
}

float ET16S::get_wheel(){
	// index will need to be changed to use a different control for wheel
	return channel[l_slider_num].data;
}

void ET16S::send_to_comms() {
	Comms::Sendable<ET16SData> sendable;
	sendable.data = get_ET16S_data();
	sendable.send_to_comms();
}

bool ET16S::is_safety_mode() {
	return (get_safety_switch() == SwitchPos::FORWARD) || !is_connected();
}

bool ET16S::is_teensy_mode() {
	return get_safety_switch() == SwitchPos::MIDDLE;
}

bool ET16S::is_hive_mode() {
	return get_safety_switch() == SwitchPos::BACKWARD;
}

bool ET16S::mode_changed(){
	return mode_changed_flag;
}


ET16SData ET16S::get_ET16S_data(){
	ET16SData ET16S_data;
	ET16S_data.safety_switch = get_safety_switch();
	ET16S_data.r_stick_x = get_r_stick_x();
	ET16S_data.r_stick_y = get_r_stick_y();
	ET16S_data.l_stick_x = get_l_stick_x();
	ET16S_data.l_stick_y = get_l_stick_y();
	ET16S_data.switch_b = get_switch_b();
	ET16S_data.switch_c = get_switch_c();
	ET16S_data.switch_d = get_switch_d();
	ET16S_data.switch_e = get_switch_e();
	ET16S_data.switch_f = get_switch_f();
	ET16S_data.switch_g = get_switch_g();
	ET16S_data.switch_h = get_switch_h();
	ET16S_data.l_slider = get_l_slider();
	ET16S_data.r_slider = get_r_slider();
	ET16S_data.trim_one = get_trim_one();
	ET16S_data.trim_two = get_trim_two();
	ET16S_data.trim_three = get_trim_three();
	ET16S_data.trim_four = get_trim_four();
	ET16S_data.trim_five = get_trim_five();
	ET16S_data.trim_six = get_trim_six();
	ET16S_data.l_dial = get_l_dial();
	ET16S_data.r_dial = get_r_dial();

	return ET16S_data;
}

void ET16S::manual_controls(const RobotStateMap& estimated_state_map, RobotStateMap& target_state_map, bool not_safety_mode, float& feed, float& last_feed) {
	bool has_lower_feeder = estimated_state_map.get_state_map().find(Cfg::StateName::LowerFeeder) != estimated_state_map.get_state_map().end();
	
	float delta = control_input_timer.delta();
	
	vtm_pos_x += ref.ref_data.kbm_interaction.mouse_speed_x * 0.05 * delta;
	vtm_pos_y += ref.ref_data.kbm_interaction.mouse_speed_y * 0.05 * delta;

	float pitch_min = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.min;
    float pitch_max = estimated_state_map[Cfg::StateName::GimbalPitch].config().reference_limits.position.max;
    float pitch_average = 0.5 * (pitch_min + pitch_max);
    pitch_min -= pitch_average;
    pitch_max -= pitch_average;

	if (vtm_pos_y < pitch_min) {
		vtm_pos_y = pitch_min;
	}
	if (vtm_pos_y > pitch_max) {
		vtm_pos_y = pitch_max;
	}

	float chassis_vel_x = 0;
	float chassis_vel_y = 0;
	float chassis_pos_x = 0;
	float chassis_pos_y = 0;

	if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Velocity) { // if we should be controlling velocity

		chassis_vel_x = get_l_stick_y() * 5.4 +
						(-ref.ref_data.kbm_interaction.key_w + ref.ref_data.kbm_interaction.key_s) * 2.5;
		chassis_vel_y = -(get_l_stick_x() * 5.4) +
						(ref.ref_data.kbm_interaction.key_d - ref.ref_data.kbm_interaction.key_a) * 2.5;
		
	} else if (estimated_state_map[Cfg::StateName::ChassisX].config().governor_type == Cfg::StateOrder::Position) { // if we should be controlling position
		chassis_pos_x = get_l_stick_x() * 2 + pos_offset_x;
		chassis_pos_y = get_l_stick_y() * 2 + pos_offset_y;
	}

	float chassis_spin = get_wheel() * 25;
	float pitch_target = 1.57 + -get_r_stick_y() * 0.3 + vtm_pos_y;
	float yaw_target = -get_r_stick_x() * 1.5 - vtm_pos_x;

	float fly_wheel_target =
		(get_switch_b() == SwitchPos::FORWARD || get_switch_b() == SwitchPos::MIDDLE) ? 18 : 0; // m/s
	// if the right switch is forward, and either the left mouse button is pressed or the right switch is not
	// backward, set the feeder to something. Otherwise, set it to 0
	float feeder_target = (((ref.ref_data.kbm_interaction.button_left) &&
							get_switch_b() != SwitchPos::BACKWARD) || get_switch_b() == SwitchPos::FORWARD) ? 12 : 0;
	if (estimated_state_map[Cfg::StateName::Feeder].config().governor_type == Cfg::StateOrder::Position) {
		float dt2 = timer.delta();
		if (dt2 > 0.1) dt2 = 0;
		// check if the shooter is active
		if (not_safety_mode && ref.ref_data.robot_performance.shooter_power_active) {
			feed += feeder_target * dt2;
		}
		
		if (has_lower_feeder) {
			target_state_map[Cfg::StateName::Feeder].set_position(estimated_state_map[Cfg::StateName::LowerFeeder].get_position());
			target_state_map[Cfg::StateName::LowerFeeder].set_position((int)feed);
		} else {
			target_state_map[Cfg::StateName::Feeder].set_position((int)feed);
		}
	} else { 
		target_state_map[Cfg::StateName::Feeder].set_velocity(feeder_target);
	}
	// if (transmitter->get_r_switch() == 1 && last_switch != 1) {
	//     feed++;
	// }
	// last_switch = transmitter->get_r_switch();
	// set manual controls
	target_state_map[Cfg::StateName::ChassisX].set_position(chassis_pos_x);
	target_state_map[Cfg::StateName::ChassisX].set_velocity(chassis_vel_x);
	target_state_map[Cfg::StateName::ChassisY].set_position(chassis_pos_y);
	target_state_map[Cfg::StateName::ChassisY].set_velocity(chassis_vel_y);
	target_state_map[Cfg::StateName::ChassisHeading].set_velocity(chassis_spin);
	target_state_map[Cfg::StateName::GimbalYaw].set_position(yaw_target);
	target_state_map[Cfg::StateName::GimbalYaw].set_velocity(0);
	target_state_map[Cfg::StateName::GimbalPitch].set_position(pitch_target);
	target_state_map[Cfg::StateName::GimbalPitch].set_velocity(0);
	target_state_map[Cfg::StateName::Flywheels].set_velocity(fly_wheel_target);

	// when in teensy control mode reset hive toggle
	if (is_teensy_mode() && mode_changed()) {
		pos_offset_x = estimated_state_map[Cfg::StateName::ChassisX].get_position();
		pos_offset_y = estimated_state_map[Cfg::StateName::ChassisY].get_position();
		feed = last_feed;
	}
}
