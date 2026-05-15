#include "StereoCamTrigger.hpp"
#include "comms/data/sendable.hpp"

void StereoCamTrigger::track_exposures() {
  // generate HIGH pulse with given width to create square wave
  digitalWrite(config.digital_trigger_pin_1, HIGH);
  digitalWrite(config.digital_trigger_pin_2, HIGH);

  delayMicroseconds(config.trigger_pulse_width);

  digitalWrite(config.digital_trigger_pin_1, LOW);
  digitalWrite(config.digital_trigger_pin_2, LOW);

  // Do state matching
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

void StereoCamTrigger::print_live_data() {
    Serial.printf(" [Stereo Cam]     Status: %s | Last Exposure (us): %u\n", 
                  stopped ? "STOPPED" : "RUNNING", latest_exposure_timestamp);
}
