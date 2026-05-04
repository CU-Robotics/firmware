#include "StereoCamTrigger.hpp"
#include "comms/data/sendable.hpp"

extern std::unique_ptr<RobotStateMap> estimated_state_map_interrupt_safe;

StereoCamTrigger::StereoCamTrigger(const Cfg::StereoCamTrigger& config): Sensor(), config(config), comms_data(config.camera_trigger_name) {}

void StereoCamTrigger::track_exposures() {
  // generate HIGH pulse with given width to create square wave
  digitalWrite(config.digital_trigger_pin_1, HIGH);
  digitalWrite(config.digital_trigger_pin_2, HIGH);

  delayMicroseconds(config.trigger_pulse_width);

  digitalWrite(config.digital_trigger_pin_1, LOW);
  digitalWrite(config.digital_trigger_pin_2, LOW);

  
  if(estimated_state_map_interrupt_safe != nullptr) {
    // copy the estimated state map to the local estimated state map
    estimated_state_map_interrupt_safe->fill_state_array(comms_data.state);
  }
}

void StereoCamTrigger::start(int res) {
  if (stopped) {
    // if the timer is stopped, start it again with the track_exposures callback
    timer.begin([this]{ track_exposures(); }, res);
    stopped = false;
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

  // determine timer resolution from FPS
  float spf = 1.0 / float(config.fps); // seconds per frame
  int mpf = 1.0e+6 * spf; // micros per frame
  
  // start the timer with the calculated resolution
  start(mpf);
}

void StereoCamTrigger::send_to_comms() const {
  Comms::Sendable<StereoCamTriggerData> sendable;

  sendable.data = comms_data;
  sendable.send_to_comms();
}