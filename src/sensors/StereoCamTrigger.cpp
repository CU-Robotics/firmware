#include "StereoCamTrigger.hpp"
#include "comms/data/sendable.hpp"
#include "transmitter_utils.hpp"
#include <core_pins.h>

std::optional<RobotStateMap>* StereoCamTrigger::estimated_state_map_interrupt_safe = nullptr;

StereoCamTrigger::StereoCamTrigger(const Cfg::StereoCamTrigger& config): Sensor(), config(config), comms_data(config.camera_trigger_name) {}

void StereoCamTrigger::track_exposures() {
  // generate HIGH pulse with given width to create square wave
  digitalWrite(config.digital_trigger_pin_1, HIGH);
  digitalWrite(config.digital_trigger_pin_2, HIGH);

  delayMicroseconds(config.trigger_pulse_width);

  digitalWrite(config.digital_trigger_pin_1, LOW);
  digitalWrite(config.digital_trigger_pin_2, LOW);
  
  if(estimated_state_map_interrupt_safe != nullptr && estimated_state_map_interrupt_safe->has_value()) {
    // copy the estimated state map to the local estimated state map
    (*estimated_state_map_interrupt_safe)->fill_state_array(comms_data.state);
  }
}

void StereoCamTrigger::start(int res) {
  if (stopped) {
    // if the timer is stopped, start it again with the track_exposures callback
    timer.begin([this]{ track_exposures(); }, res);
    stopped = false;
    first_trigger = true;
  }
}

void StereoCamTrigger::stop() {
  if (!stopped) {
    // if the timer is running, stop it
    timer.end();
    stopped = true;
  }
}

void StereoCamTrigger::init() {
  // configure GPIO pin for sending output signal
  pinMode(config.digital_trigger_pin_1, OUTPUT);
  pinMode(config.digital_trigger_pin_2, OUTPUT);

  pinMode(config.camera_1_line_1_pin, OUTPUT);
  pinMode(config.camera_2_line_1_pin, OUTPUT);
  pinMode(config.camera_1_line_2_pin, OUTPUT);
  pinMode(config.camera_2_line_2_pin, OUTPUT);

  // determine timer resolution from FPS
  float spf = 1.0 / float(config.fps); // seconds per frame
  mpf = 1.0e+6 * spf; // micros per frame

  start(mpf);
}

void StereoCamTrigger::bind_isr_map(std::optional<RobotStateMap> *safe_map) {
    estimated_state_map_interrupt_safe = safe_map;
}

void StereoCamTrigger::read() {
	if (Comms::comms_layer.get_hive_data().stereo_cam_start_stop.start_received) {
		digitalWrite(config.camera_1_line_2_pin, HIGH);
		digitalWrite(config.camera_2_line_2_pin, HIGH);
		
		delayMicroseconds(config.trigger_pulse_width);
    
		digitalWrite(config.camera_1_line_2_pin, LOW);
		digitalWrite(config.camera_2_line_2_pin, LOW);
		
		Serial.printf("counter reset pin: %u triggered\n", config.camera_1_line_2_pin);
	}
	Comms::comms_layer.get_hive_data().stereo_cam_start_stop.stop_received = false;
	Comms::comms_layer.get_hive_data().stereo_cam_start_stop.start_received = false;
}

void StereoCamTrigger::send_to_comms() const {
  Comms::Sendable<StereoCamTriggerData> sendable;

  sendable.data = comms_data;
  sendable.send_to_comms();
}

void StereoCamTrigger::print_live_data() {
    Serial.printf(" [Stereo Cam]     Status: %s | Last Exposure (us): %u\n", 
                  stopped ? "STOPPED" : "RUNNING", latest_exposure_timestamp);
}
