#pragma once

#include <Arduino.h>
#include <optional>
//#include "./ET16S.hpp"
//#include "./dr16.hpp"

enum class TransmitterType {
	INVALID,
	DR16,
	ET16S
};

class Transmitter {
public:

	/// @brief who_am_i assumes transmitter is ET16S, on fail it returns dr16
	/// @note utilizes the counting byte at the end of every element
	/// @return TransmitterType 
	static TransmitterType who_am_i();
	virtual void read() {}
	virtual void init() {}
	virtual void print() {}
	virtual void zero() {}
	virtual void print_raw() {}
	virtual float get_r_stick_x() { return 0; }
	virtual float get_r_stick_y() { return 0; }
	virtual float get_l_stick_x() { return 0; }
	virtual float get_l_stick_y() { return 0; }
	virtual bool is_data_valid() { return false; }
	virtual uint8_t is_fail() { return 0; }
	virtual bool is_connected() { return 0; }
	virtual int get_mouse_x() { return 0; }
	virtual int get_mouse_y() { return 0; }
	virtual bool get_l_mouse_button() { return false; }
	virtual bool get_r_mouse_button() { return false; }
	virtual float get_l_switch() { return 0; }
	virtual float get_r_switch() { return 0; }
	virtual float get_wheel() { return 0; }
	virtual float* get_input() { return 0; }
	virtual uint8_t* get_raw() { return 0; }
	virtual void print_format_bin(int channel_num) {}
	virtual uint8_t get_safety_switch() { return 0; }
	virtual std::optional<float> get_switch_b() { return {}; }
	virtual std::optional<float> get_switch_c() { return {}; }
	virtual std::optional<float> get_switch_d() { return {}; }
	virtual std::optional<float> get_switch_e() { return {}; }
	virtual std::optional<float> get_switch_f() { return {}; }
	virtual std::optional<float> get_switch_g() { return {}; }
	virtual std::optional<float> get_switch_h() { return {}; }
	virtual std::optional<float> get_l_slider() { return {}; }
	virtual std::optional<float> get_r_slider() { return {}; }
	virtual std::optional<float> get_l_dial() { return {}; }
	virtual std::optional<float> get_r_dial() { return {}; }
	virtual std::optional<float> get_trim_one() { return {}; }
	virtual std::optional<float> get_trim_two() { return {}; }
	virtual std::optional<float> get_trim_three() { return {}; }
	virtual std::optional<float> get_trim_four() { return {}; }
	virtual std::optional<float> get_trim_five() { return {}; }
	virtual std::optional<float> get_trim_six() { return {}; }
	virtual std::optional<float> get_channel_data(int chan_num) { return {}; }
	virtual ~Transmitter() {}


	struct Keys {
		// just testing with keys at the moment
		// but will eventually implement
		// the mouse functionalities.

		/// @brief If the key 'w' is pressed
		bool w=0;
		/// @brief If the key 's' is pressed
		bool s=0;
		/// @brief if the key 'a' is pressed
		bool a=0;
		/// @brief if the key 'd' is pressed
		bool d=0;
		/// @brief if the key 'shift' is pressed
		bool shift;
		/// @brief if the key 'ctrl' is pressed
		bool ctrl;
		/// @brief if the key 'q' is pressed
		bool q;
		/// @brief if the key 'e' is pressed
		bool e;
		/// @brief if the key 'r' is pressed
		bool r;
		/// @brief if the key 'f' is pressed
		bool f;
		/// @brief if the key 'g' is pressed
		bool g;
		/// @brief if the key 'z' is pressed
		bool z;
		/// @brief if the key 'x' is pressed
		bool x;
		/// @brief if the key 'c' is pressed
		bool c;
		/// @brief if the key 'v' is pressed
		bool v;
		/// @brief if the key 'b' is pressed
		bool b;
	};

	/// @brief struct instance to keep track of the rm control data
	Keys keys;

};
