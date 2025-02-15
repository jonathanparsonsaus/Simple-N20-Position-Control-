/***********************************************************
 * PID Position Control with Quadrature Encoder (N20 Motor)
 * and Dual PWM Motor Signals
 *
 * This sketch implements a PID controller to position an N20 
 * motor based on feedback from a quadrature encoder.
 *
 * Motor driver configuration:
 *  - For forward motion, MOTOR_PIN_A is driven with PWM and MOTOR_PIN_B is LOW.
 *  - For reverse motion, MOTOR_PIN_B is driven with PWM and MOTOR_PIN_A is LOW.
 *
 * Encoder connections:
 *  - Channel A is connected to pin D2 (INT0).
 *  - Channel B is connected to pin D3 (INT1).
 *
 * PID controller parameters (starting values for tuning):
 *  - Proportional gain (Kp): 2.0
 *  - Integral gain (Ki): 0.0 (disabled here; adjust as needed)
 *  - Derivative gain (Kd): 0.0 (disabled here; adjust as needed)
 *
 * The control loop runs every 20 ms.
 ***********************************************************/

#include <Arduino.h>

// ---------------- Pin Definitions ----------------
// Define PWM output pins for driving the motor driver.
static const uint8_t MOTOR_PIN_A = 9; // PWM output for driving motor in forward direction
static const uint8_t MOTOR_PIN_B = 8; // PWM output for driving motor in reverse direction

// Define pins for encoder channels (using hardware interrupts).
static const uint8_t PIN_ENC_A = 2;   // Encoder channel A (interrupt 0 on Arduino)
static const uint8_t PIN_ENC_B = 3;   // Encoder channel B (interrupt 1 on Arduino)

// ---------------- Encoder State ------------------
// Global variable to keep track of the encoder count (position)
// 'volatile' indicates it may be changed by an ISR.
volatile long encoderCount = 0;

// 'lastAB' stores the previous state of the encoder channels as a 2-bit number.
volatile uint8_t lastAB = 0;

// ---------------- Quadrature Decoder Lookup Table ------------------
// This 16-entry lookup table helps decode quadrature encoder transitions.
// The index is constructed as (oldState << 2) | newState (i.e. a 4-bit number).
// The table maps state transitions to a count increment of -1, 0, or +1.
static const int8_t QDEC_LOOKUP[16] = {
  0,  +1, -1,  0,   // Transitions from state 0
 -1,  0,  0,  +1,   // Transitions from state 1
 +1,  0,  0,  -1,   // Transitions from state 2
  0,  -1, +1,  0     // Transitions from state 3
};

// ---------------- PID Control Parameters -------------
// These parameters control the response of the PID controller.
// Note: Ki and Kd are set to 0 here, so the controller acts as a P-cont
