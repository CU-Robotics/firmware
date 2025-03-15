#pragma once

#include <Arduino.h>
#include <optional>

/// @brief The enum of what type of transmitter is being used
enum class TransmitterType {
	INVALID = 0,
	DR16,
	ET16S
};

/// @brief The enum of the switch positions
enum class SwitchPos{
	INVALID = 0,
	FORWARD,
	BACKWARD,
	MIDDLE
};

/// @brief A unifying interface for all transmitters
class Transmitter {
public:
	/// @brief checks the incoming data stream to determine the transmitter type
	/// @note who_am_i assumes transmitter is ET16S, on fail it returns dr16. utilizes the packet size of each transmitter to comfirm
	/// @return Corrosponding TransmitterType object
	static TransmitterType who_am_i();

	/// @brief standard destructor
	virtual ~Transmitter() {}
	
	/// @brief Reads raw input
	virtual void read() {}
	
	/// @brief initalizes serial connection
	virtual void init() {}
	
	/// @brief prints all output values
	virtual void print() {}
	
	/// @brief zeros all buffers
	virtual void zero() {}
	
	/// @brief Prints raw input
	virtual void print_raw() {}
	
	/// @brief get right stick x axis value
	/// @return (-1 to 1)
	virtual float get_r_stick_x() { return 0; }
	
	/// @brief get right stick y axis value
	/// @return (-1 to 1)
	virtual float get_r_stick_y() { return 0; }
	
	/// @brief get left stick x axis value
	/// @return (-1 to 1)
	virtual float get_l_stick_x() { return 0; }
	
	/// @brief get left stick y axis value
	/// @return (-1 to 1)
	virtual float get_l_stick_y() { return 0; }
	
	/// @brief checks if data is valid
	/// @return returns true if data is valid false otherwise
	virtual bool is_data_valid() { return false; }
	
	/// @brief Returns the fail bit. Set only if invalid packets have been received for more then 250ms
	/// @return Failure status
	virtual uint8_t is_fail() { return 0; }
	
	/// @brief getter for connection status
	/// @return false if disconnected
	virtual bool is_connected() { return 0; }
	
	/// @brief Get mouse velocity x
	/// @return Amount of points since last read
	virtual std::optional<int> get_mouse_x() { return {}; }
	
	/// @brief Get mouse velocity y
	/// @return Amount of points since last read
	virtual std::optional<int> get_mouse_y() { return {}; }
	
	/// @brief status of left mouse button
	/// @return Is left mouse button pressed
	virtual std::optional<bool> get_l_mouse_button() { return {}; }
	
	/// @brief status of right mouse button
	/// @return Is right mouse button pressed
	virtual std::optional<bool> get_r_mouse_button() { return {};}
	
	/// @brief used for safety switch
	/// @return left most front face switch value
	virtual SwitchPos get_l_switch() { return SwitchPos::INVALID; }
	
	/// @brief used for flywheel trigger switch
	/// @return left most front face switch value
	virtual SwitchPos get_r_switch() { return SwitchPos::INVALID; }
	
	/// @brief used for spin wheel
	/// @return wheel value
	virtual float get_wheel() { return 0; }
	
	/// @brief used to get input
	/// @return pointer to input array
	virtual float* get_input() { return 0; }
	
	/// @brief used to get raw input
	/// @return pointer to raw input array
	virtual uint8_t* get_raw() { return nullptr; }
	
	/// @brief prints data in binary for a specific channel
	/// @param channel_num channel number from 0-16 inclusive
	virtual void print_format_bin(int channel_num) {}
	
	/// @brief getter for safety switch
	/// @return safety switch value
	virtual SwitchPos get_safety_switch() { return SwitchPos::INVALID; }
	
	/// @brief get switch b value on the ET16S
	/// @return switch b value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_b() { return {}; }
	
	/// @brief get switch c value on the ET16S
	/// @return switch c value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_c() { return {}; }
	
	/// @brief get switch d value on the ET16S
	/// @return switch d value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_d() { return {}; }
	
	/// @brief get switch e value on the ET16S
	/// @return switch e value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_e() { return {}; }
	
	/// @brief get switch f value on the ET16S
	/// @return switch f value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_f() { return {}; }
	
 	/// @brief get switch g value on the ET16S
	/// @return switch g value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_g() { return {}; }
	
	/// @brief get switch h value on the ET16S
	/// @return switch h value if it exists otherwise return nothing
	virtual std::optional<SwitchPos> get_switch_h() { return {}; }
	
	/// @brief get left slider value on the ET16S
	/// @return left slider value if it exists otherwise return nothing
	virtual std::optional<float> get_l_slider() { return {}; }
	
 	/// @brief get right slider value on the ET16S
	/// @return right slider value if it exists otherwise return nothing
	virtual std::optional<float> get_r_slider() { return {}; }
	
	/// @brief get left dial value on the ET16S
	/// @return left dial value if it exists otherwise return nothing
	virtual std::optional<float> get_l_dial() { return {}; }
	
	/// @brief get right dial value on the ET16S
	/// @return right dial value if it exists otherwise return nothing
	virtual std::optional<float> get_r_dial() { return {}; }
	
	/// @brief get trim one value on the ET16S
	/// @return trim one value if it exists otherwise return nothing
	virtual std::optional<float> get_trim_one() { return {}; }
	
	/// @brief get trim two value on the ET16S
	/// @return trim two value if it exists otherwise return nothing
	virtual std::optional<float> get_trim_two() { return {}; }
	
	/// @brief get trim three value on the ET16S
	/// @return trim three value if it exists otherwise return nothing
	virtual std::optional<float> get_trim_three() { return {}; }
	
	/// @brief get trim four value on the ET16S
	/// @return trim four value if it exists otherwise return nothing
	virtual std::optional<float> get_trim_four() { return {}; }
	
	/// @brief get trim five value on the ET16S
	/// @return trim five value if it exists otherwise return nothing
	virtual std::optional<float> get_trim_five() { return {}; }
	
 	/// @brief get trim six value on the ET16S
	/// @return trim six value if it exists otherwise return nothing
	virtual std::optional<float> get_trim_six() { return {}; }
	
	/// @brief Various keys that can be pressed on the transmitter
	struct Keys {
		/// @brief If the key 'w' is pressed
		bool w = 0;
		/// @brief If the key 's' is pressed
		bool s = 0;
		/// @brief if the key 'a' is pressed
		bool a = 0;
		/// @brief if the key 'd' is pressed
		bool d = 0;
		/// @brief if the key 'shift' is pressed
		bool shift = 0;
		/// @brief if the key 'ctrl' is pressed
		bool ctrl = 0;
		/// @brief if the key 'q' is pressed
		bool q = 0;
		/// @brief if the key 'e' is pressed
		bool e = 0;
		/// @brief if the key 'r' is pressed
		bool r = 0;
		/// @brief if the key 'f' is pressed
		bool f = 0;
		/// @brief if the key 'g' is pressed
		bool g = 0;
		/// @brief if the key 'z' is pressed
		bool z = 0;
		/// @brief if the key 'x' is pressed
		bool x = 0;
		/// @brief if the key 'c' is pressed
		bool c = 0;
		/// @brief if the key 'v' is pressed
		bool v = 0;
		/// @brief if the key 'b' is pressed
		bool b = 0;
	};

	/// @brief struct instance to keep track of the rm control data
	Keys keys;

	/// @brief get keys
	/// @return keys pressed
	virtual std::optional<Keys> get_keys() { return {}; }

};
