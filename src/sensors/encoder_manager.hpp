#pragma once
#include "buff_encoder.hpp"

/*
Encoder Manager allows the buff encoders dma transfers to be daisy chained.
This allows us to read the data coming from the encoders asychronesly.
WorkFlow
1. call isr_start_transfer on encoder 1
2. On completeion of req1 call isr_stop_transfer and isr_start_transfer on enc 2
3. ...
4. On completion of req4 call isr_stop_transfer.
5. Process data in regular read statement


 */
EventResponder spi_event;
class EncoderManager {
public:
	EncoderManager();
	~EncoderManager();
	void init();
	void request_read();
	void read();
	static void encoder_isr_wrapper(EventResponderRef spi_event);
	void encoder_isr();
private:
    // Will need to figure out how to make this non-const since we can have 3 or 4 
	static const uint8_t num_encoders = 4;
	std::array<BuffEncoder*, num_encoders> encoders;
    static volatile uint8_t current_index;
    bool transfer_in_progress = false;

	/// @brief  Pointer to the singleton instance of this class
	static EncoderManager* instance;

	
};
