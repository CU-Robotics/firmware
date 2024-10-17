#include "logger.hpp"
/// @brief internal buffer with 4kb capacity
/// @note will need to change size if we every print more than 4096 characters per loop
DMAMEM char log_buffer[4096];

size_t Logger::write(const uint8_t *buffer, size_t size ){
    if(cursor+size >= sizeof(log_buffer)){return 0;}
	
    memcpy(log_buffer+cursor,buffer,size);
	char print_statement[sizeof(log_buffer)]={0}; // temp variable
	memcpy(print_statement,log_buffer+cursor,size);
#ifdef LOGGER_FLAG
	Serial.print(print_statement);
#endif
	
    cursor+=size;
    return size;
}
uint32_t Logger::grab_log_data(uint32_t size, uint8_t *data){
    if (data==nullptr){return 0;}
    
    memcpy(data,log_buffer,cursor);
    int data_size = cursor; // temp variable
    cursor=0;
    return data_size;
}
