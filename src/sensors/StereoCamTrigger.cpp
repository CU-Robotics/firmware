#include "StereoCamTrigger.hpp"

void StereoCamTrigger::track_exposures() {
  // disable interrupts to protect volatile access
  cli();

  // generate HIGH pulse with given width to create square wave
  digitalWrite(config.pins.digital_pin, HIGH);
  delayMicroseconds(config.trigger_pulse_width);
  digitalWrite(config.pins.digital_pin, LOW);
  
  // update timestamp
#ifdef LOG_STEREO_FPS
  uint32_t prev_timestamp = latest_exposure_timestamp;
#endif

  latest_exposure_timestamp = micros();

#ifdef LOG_STEREO_FPS
  uint32_t delta = latest_exposure_timestamp - prev_timestamp;

  // print FPS estimate
  Serial.printf("fps: %f\n", 1/(float(delta) * 1.0e-6));
#endif

  // reenable interrupts
  sei();
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
  pinMode(config.pins.digital_pin, OUTPUT);

  // determine timer resolution from FPS
  float spf = 1.0 / float(config.fps); // seconds per frame
  int mpf = 1.0e+6 * spf; // micros per frame
  
  // start the timer with the calculated resolution
  start(mpf);
}

uint32_t StereoCamTrigger::get_latest_exposure_timestamp() {
  uint32_t ret;

  cli(); // disable interrupts to protect volatile access
  ret = latest_exposure_timestamp;
  sei(); // reenable interrupts

  return ret;
}