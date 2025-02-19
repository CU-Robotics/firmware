#include "StereoCamTrigger.hpp"
#include "usb_serial.h"

void StereoCamTrigger::track_exposures() {
  // disable interrupts to protect volatile access
  cli();

  // generate HIGH pulse with given width to create square wave
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(TRIG_PULSE_WIDTH);
  digitalWrite(TRIG_PIN, LOW);
  
  // update timestamp
  uint32_t prev_timestamp = latest_exposure_timestamp;
  latest_exposure_timestamp = micros();
  uint32_t delta = latest_exposure_timestamp - prev_timestamp;

  Serial.print(delta);

#ifdef LOG_STEREO_FPS
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
  pinMode(TRIG_PIN, OUTPUT);

  // determine timer resolution from FPS
  float spf = 1.0 / float(fps); // seconds per frame
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
