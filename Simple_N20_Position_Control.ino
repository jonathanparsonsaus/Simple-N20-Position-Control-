/***********************************************************
 * Optimized P-Only Position Control with Quadrature Encoder
 * and Dual PWM Motor Signals
 * ---------------------------------------------------------
 * In this configuration, the motor driver uses two inputs:
 *   - MOTOR_PIN_A and MOTOR_PIN_B
 * To drive the motor:
 *   - Forward:  MOTOR_PIN_A = PWM; MOTOR_PIN_B = 0
 *   - Reverse:  MOTOR_PIN_A = 0;   MOTOR_PIN_B = PWM
 *
 * Encoder is on pins D2 (INT0) & D3 (INT1).
 *
 * The controller accepts position setpoint commands via Serial.
 ***********************************************************/

#include <Arduino.h>

// ---------------- Pin Definitions ----------------
static const uint8_t MOTOR_PIN_A = 9; // Motor signal A (PWM)
static const uint8_t MOTOR_PIN_B = 8; // Motor signal B (PWM)
static const uint8_t PIN_ENC_A     = 2; // Encoder channel A (INT0)
static const uint8_t PIN_ENC_B     = 3; // Encoder channel B (INT1)

// ---------------- Encoder State ------------------
volatile long encoderCount = 0;  
volatile uint8_t lastAB   = 0;   // last 2-bit state of encoder A/B

// A 16-entry lookup table for quadrature decoding.
// Index = (oldState << 2) | newState.
static const int8_t QDEC_LOOKUP[16] = {
  0, +1, -1,  0,   // 0..3
 -1,  0,  0, +1,   // 4..7
 +1,  0,  0, -1,   // 8..11
  0, -1, +1,  0    // 12..15
};

// ---------------- Control Parameters -------------
float Kp = 1.0;            // Proportional gain
long  targetPosition = 0;  // Set via serial commands

// Control loop timing (every 20 ms)
unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL = 20; // ms

// ---------------- Serial Input Buffer ------------
#define CMD_BUFFER_SIZE 16
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("P-Only Position Control (Dual PWM Signals)");
  Serial.println("Enter a setpoint (e.g. 1000) and press Enter");

  // Motor pins
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  analogWrite(MOTOR_PIN_A, 0);
  analogWrite(MOTOR_PIN_B, 0);

  // Encoder pins
  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);

  // Initialize encoder state
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  lastAB = (a << 1) | b;

  // Attach interrupts for encoder channels
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEnc, CHANGE);

  lastControlTime = millis();
}

// --------------------------------------------------
void loop() {
  // 1) Handle incoming serial data (new target positions)
  readSerial();

  // 2) Run P-control loop at fixed interval
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = now;

    // Copy volatile encoderCount safely
    long currentPos;
    noInterrupts();
    currentPos = encoderCount;
    interrupts();

    // Calculate error
    long error = targetPosition - currentPos;

    // Proportional output (using floating point gain)
    float output = Kp * (float)error; 

    // Clamp output to [-255, 255]
    if (output > 255.0f)  output = 255.0f;
    if (output < -255.0f) output = -255.0f;

    // Drive motor using dual PWM signals
    driveMotor(output);

    // Debug print
    Serial.print("Pos=");
    Serial.print(currentPos);
    Serial.print("  Tgt=");
    Serial.print(targetPosition);
    Serial.print("  Out=");
    Serial.println(output, 1);
  }
}

// --------------------------------------------------
// driveMotor: Drives the motor using two PWM signals.
// If the command is positive, output PWM on MOTOR_PIN_A (forward).
// If negative, output PWM on MOTOR_PIN_B (reverse).
// --------------------------------------------------
void driveMotor(float cmd) {
  int pwmVal = abs((int)cmd);
  if (pwmVal > 255) pwmVal = 255;

  if (cmd >= 0) {
    analogWrite(MOTOR_PIN_A, pwmVal);
    analogWrite(MOTOR_PIN_B, 0);
  } else {
    analogWrite(MOTOR_PIN_A, 0);
    analogWrite(MOTOR_PIN_B, pwmVal);
  }
}

// --------------------------------------------------
// readSerial: Non-blocking read of serial input for setpoint.
// Waits for newline or carriage return, then parses as long.
// --------------------------------------------------
void readSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIndex > 0) {
        cmdBuffer[cmdIndex] = '\0'; // Null-terminate
        long val = atol(cmdBuffer);
        targetPosition = val;
        Serial.print("New Target: ");
        Serial.println(val);
        cmdIndex = 0; // Reset buffer
      }
    } else {
      if (cmdIndex < CMD_BUFFER_SIZE - 1) {
        cmdBuffer[cmdIndex++] = c;
      }
    }
  }
}

// --------------------------------------------------
// isrEnc: Single ISR for both encoder channels.
// Uses the lookup table to update encoderCount.
// --------------------------------------------------
void isrEnc() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t newAB = (a << 1) | b;
  uint8_t index = (lastAB << 2) | newAB;  // Range: 0..15
  encoderCount += QDEC_LOOKUP[index & 0x0F];
  lastAB = newAB;
}
