// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "NewPingLite.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

#include "NewPingLite.h"


// ---------------------------------------------------------------------------
// NewPingLite constructor
// ---------------------------------------------------------------------------

NewPingLite::NewPingLite(uint8_t trigger_pin, uint8_t echo_pin, int max_cm_distance) {
	_triggerBit = digitalPinToBitMask(trigger_pin); // Get the port register bitmask for the trigger pin.
	_echoBit = digitalPinToBitMask(echo_pin);       // Get the port register bitmask for the echo pin.

	_triggerOutput = portOutputRegister(digitalPinToPort(trigger_pin)); // Get the output port register for the trigger pin.
	_echoInput = portInputRegister(digitalPinToPort(echo_pin));         // Get the input port register for the echo pin.

	_triggerMode = (uint8_t *) portModeRegister(digitalPinToPort(trigger_pin)); // Get the port mode register for the trigger pin.

	_maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.

#if DISABLE_ONE_PIN == true
	*_triggerMode |= _triggerBit; // Set trigger pin to output.
#endif
}


// ---------------------------------------------------------------------------
// Standard ping methods
// ---------------------------------------------------------------------------

unsigned int NewPingLite::ping() {
	if (!ping_trigger()) return NO_ECHO;                // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
	while (*_echoInput & _echoBit)                      // Wait for the ping echo.
		if (micros() > _max_time) return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
	return (micros() - (_max_time - _maxEchoTime) - 5); // Calculate ping time, 5uS of overhead.
}


unsigned int NewPingLite::ping_in() {
	unsigned int echoTime = NewPingLite::ping();          // Calls the ping method and returns with the ping echo distance in uS.
	return NewPingLiteConvert(echoTime, US_ROUNDTRIP_IN); // Convert uS to inches.
}


unsigned int NewPingLite::ping_cm() {
	unsigned int echoTime = NewPingLite::ping();          // Calls the ping method and returns with the ping echo distance in uS.
	return NewPingLiteConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
}


unsigned int NewPingLite::ping_median(uint8_t it) {
	unsigned int uS[it], last;
	uint8_t j, i = 0;
	uS[0] = NO_ECHO;
	while (i < it) {
		last = ping();           // Send ping.
		if (last == NO_ECHO) {   // Ping out of range.
			it--;                // Skip, don't include as part of median.
			last = _maxEchoTime; // Adjust "last" variable so delay is correct length.
		} else {                       // Ping in range, include as part of median.
			if (i > 0) {               // Don't start sort till second ping.
				for (j = i; j > 0 && uS[j - 1] < last; j--) // Insertion sort loop.
					uS[j] = uS[j - 1]; // Shift ping array to correct position for sort insertion.
			} else j = 0;              // First ping is starting point for sort.
			uS[j] = last;              // Add last ping to array in sorted position.
			i++;                       // Move to next ping.
		}
		if (i < it) delay(PING_MEDIAN_DELAY - (last >> 10)); // Millisecond delay between pings.
	}
	return (uS[it >> 1]); // Return the ping distance median.
}


// ---------------------------------------------------------------------------
// Standard ping method support functions (not called directly)
// ---------------------------------------------------------------------------

boolean NewPingLite::ping_trigger() {
#if DISABLE_ONE_PIN != true
	*_triggerMode |= _triggerBit;    // Set trigger pin to output.
#endif
	*_triggerOutput &= ~_triggerBit; // Set the trigger pin low, should already be low, but this will make sure it is.
	delayMicroseconds(4);            // Wait for pin to go low, testing shows it needs 4uS to work every time.
	*_triggerOutput |= _triggerBit;  // Set trigger pin high, this tells the sensor to send out a ping.
	delayMicroseconds(10);           // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
	*_triggerOutput &= ~_triggerBit; // Set trigger pin back to low.
#if DISABLE_ONE_PIN != true
	*_triggerMode &= ~_triggerBit;   // Set trigger pin to input (when using one Arduino pin this is technically setting the echo pin to input as both are tied to the same Arduino pin).
#endif

	_max_time =  micros() + MAX_SENSOR_DELAY;                  // Set a timeout for the ping to trigger.
	while (*_echoInput & _echoBit && micros() <= _max_time) {} // Wait for echo pin to clear.
	while (!(*_echoInput & _echoBit))                          // Wait for ping to start.
		if (micros() > _max_time) return false;                // Something went wrong, abort.

	_max_time = micros() + _maxEchoTime; // Ping started, set the timeout.
	return true;                         // Ping started successfully.
}

// ---------------------------------------------------------------------------
// Conversion methods (rounds result to nearest inch or cm).
// ---------------------------------------------------------------------------

unsigned int NewPingLite::convert_in(unsigned int echoTime) {
	return NewPingLiteConvert(echoTime, US_ROUNDTRIP_IN); // Convert uS to inches.
}


unsigned int NewPingLite::convert_cm(unsigned int echoTime) {
	return NewPingLiteConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
}
