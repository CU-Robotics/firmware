#pragma once
#include <Arduino.h>

/// @brief comment define statement to stop printing to serial monitor
#define LOGGER_FLAG

/// @brief Wrapper for storing print data from Serial
class Logger:public Print{
public:
	/// @brief Default Constructor
	Logger() = default;
	/// @brief Default Destructor
	~Logger() = default;
	
	/// @brief Neccesary function to utilize Print abstract class
	size_t write(uint8_t b){print("UNEXPECTED PRINT IN LOGGER.HPP"); return b;}
	
	/// @brief copies internal buffer to inputted location (*data) in memory
	/// @return number of bytes copied
	uint32_t grab_log_data(uint32_t size, uint8_t *data);
private:
	/// @brief copies formatted bytes to internal buffer
	/// @return number of bytes
	size_t write(const uint8_t *buffer, size_t size);
	
	/// @brief amoount of bytes currently stored in log
	/// @note also used as current position in memory
	unsigned int cursor;
};
/// @brief universal logger object to be called anywhere in codebase
extern Logger logger;
