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
// Note: Ki and Kd are set to 0 here, so the controller acts as a P-controller.
// You can adjust these values as needed to add integral or derivative action.
float Kp = 2;   // Proportional gain – main factor affecting response speed.
float Ki = 0;   // Integral gain – helps eliminate steady-state error.
float Kd = 0;   // Derivative gain – helps dampen oscillations.

// 'integral' stores the accumulated error over time for the integral term.
float integral = 0.0;
// 'lastError' stores the previous error value for derivative calculation.
long lastError = 0;

// 'targetPosition' is the desired encoder count (setpoint).
// It is updated via serial commands.
long targetPosition = 0;

// ---------------- Control Loop Timing ------------------
// 'lastControlTime' holds the timestamp of the last control loop iteration.
unsigned long lastControlTime = 0;
// Define the control loop interval in milliseconds (20 ms here).
const unsigned long CONTROL_INTERVAL = 20;
// 'dt' is the time step in seconds (20 ms = 0.02 s), used in PID calculations.
const float dt = CONTROL_INTERVAL / 1000.0;

// ---------------- Serial Input Buffer ------------------
// Buffer for storing incoming serial data for setpoint commands.
#define CMD_BUFFER_SIZE 16
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

////////////////////////////////////////////////////////////
// setup()
// This function initializes serial communication, configures pins,
// initializes the encoder state, and sets up interrupts.
////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);   // Start serial communication at 115200 baud.
  Serial.println("PID Position Control for N20 Motor");
  Serial.println("Enter a setpoint (e.g. 1000) and press Enter");

  // Set motor pins as outputs and initialize them to LOW (motor off).
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  analogWrite(MOTOR_PIN_A, 0);
  analogWrite(MOTOR_PIN_B, 0);

  // Set encoder pins as inputs.
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  // Read initial state of encoder channels to initialize lastAB.
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  // Pack the two bits into one number: (a << 1) | b.
  lastAB = (a << 1) | b;

  // Attach interrupts to the encoder pins.
  // Any change on these pins triggers the isrEnc() interrupt service routine.
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEnc, CHANGE);

  // Record the current time for the control loop timer.
  lastControlTime = millis();
}

////////////////////////////////////////////////////////////
// loop()
// Main loop where the PID control is executed at fixed intervals.
// It reads new setpoints from the serial port, calculates the error,
// computes the PID output, clamps the output, and drives the motor.
////////////////////////////////////////////////////////////
void loop() {
  // 1) Handle incoming serial data to update the target position (setpoint).
  readSerial();

  // 2) Run the PID control loop at the defined interval (every 20 ms).
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = now;

    // Safely copy the volatile encoderCount into a local variable.
    long currentPos;
    noInterrupts();           // Disable interrupts to prevent data corruption.
    currentPos = encoderCount;
    interrupts();             // Re-enable interrupts.

    // ---------------- PID Calculations ------------------
    // Compute the error between desired target and current position.
    long error = targetPosition - currentPos;

    // Update the integral term (accumulated error).
    // Multiply error by dt (the time step) to approximate the integral.
    integral += error * dt;

    // Compute the derivative term (rate of change of error).
    float derivative = (error - lastError) / dt;

    // Compute the PID controller output.
    float output = Kp * error + Ki * integral + Kd * derivative;

    // Save the current error for the next iteration's derivative calculation.
    lastError = error;

    // Clamp the output to the range [-255, 255] because analogWrite() takes values in this range.
    if (output > 255.0f)  output = 255.0f;
    if (output < -255.0f) output = -255.0f;

    // Use the computed output to drive the motor.
    driveMotor(output);

    // ---------------- Debug Print ------------------
    // Print the current position, target position, error, and PID output.
    Serial.print("Pos=");
    Serial.print(currentPos);
    Serial.print("  Tgt=");
    Serial.print(targetPosition);
    Serial.print("  Err=");
    Serial.print(error);
    Serial.print("  Out=");
    Serial.println(output, 1);
  }
}

////////////////////////////////////////////////////////////
// driveMotor()
// This function applies the control output to the motor driver using two PWM channels.
// The sign of 'cmd' determines the direction: a positive value drives forward,
// and a negative value drives in reverse.
////////////////////////////////////////////////////////////
void driveMotor(float cmd) {
  // Convert the control signal to an absolute PWM value.
  int pwmVal = abs((int)cmd);
  // Ensure that the PWM value does not exceed the maximum (255 for 8-bit PWM).
  if (pwmVal > 255) pwmVal = 255;

  // If the command is positive, drive MOTOR_PIN_A and set MOTOR_PIN_B to 0.
  if (cmd >= 0) {
    analogWrite(MOTOR_PIN_A, pwmVal);
    analogWrite(MOTOR_PIN_B, 0);
  }
  // If the command is negative, drive MOTOR_PIN_B and set MOTOR_PIN_A to 0.
  else {
    analogWrite(MOTOR_PIN_A, 0);
    analogWrite(MOTOR_PIN_B, pwmVal);
  }
}

////////////////////////////////////////////////////////////
// readSerial()
// This function reads incoming serial data without blocking.
// When a newline or carriage return is received, the buffer is parsed as a long,
// and the targetPosition (setpoint) is updated.
////////////////////////////////////////////////////////////
void readSerial() {
  // Process all available serial characters.
  while (Serial.available() > 0) {
    char c = (char)Serial.read();  // Read one character from the serial buffer.
    // Check if the character is a newline or carriage return.
    if (c == '\n' || c == '\r') {
      // If we have data in the buffer, process it.
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0';  // Null-terminate the string.
        long val = atol(cmdBuffer);   // Convert string to long.
        targetPosition = val;          // Update the target position.
        Serial.print("New Target: ");
        Serial.println(val);
        cmdIndex = 0;                // Reset the buffer index for the next command.
      }
    } else {
      // If there's space in the buffer, store the character.
      if (cmdIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdIndex++] = c;
      }
    }
  }
}

////////////////////////////////////////////////////////////
// isrEnc()
// Interrupt Service Routine for encoder channels.
// It reads the current state of the encoder pins, uses the lookup table
// to determine the change in encoder count, updates encoderCount,
// and saves the current state for the next transition.
////////////////////////////////////////////////////////////
void isrEnc() {
  // Read current state of encoder channels A and B.
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  // Combine the two bits into a 2-bit number.
  uint8_t newAB = (a << 1) | b;
  // Create a 4-bit index from the previous state and the new state.
  uint8_t index = (lastAB << 2) | newAB;
  // Use the lookup table to determine how much to change encoderCount.
  // Mask index with 0x0F to ensure it is in the range 0-15.
  encoderCount += QDEC_LOOKUP[index & 0x0F];
  // Update lastAB with the new state.
  lastAB = newAB;
}
