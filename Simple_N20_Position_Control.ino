/***********************************************************
 * PID Position Control with Quadrature Encoder (N20 Motor)
 * and Dual PWM Motor Signals
 * ---------------------------------------------------------
 * For a small N20 motor, recommended starting PID values:
 *   Kp = 4.0, Ki = 0.8, Kd = 0.2
 *
 * Motor driver configuration:
 *   - Forward:  MOTOR_PIN_A = PWM; MOTOR_PIN_B = 0
 *   - Reverse:  MOTOR_PIN_A = 0;   MOTOR_PIN_B = PWM
 *
 * Encoder is on pins D2 (INT0) & D3 (INT1).
 ***********************************************************/

#include <Arduino.h>

// ---------------- Pin Definitions ----------------
static const uint8_t MOTOR_PIN_A = 9; // Motor signal A (PWM)
static const uint8_t MOTOR_PIN_B = 8; // Motor signal B (PWM)
static const uint8_t PIN_ENC_A     = 2; // Encoder channel A (INT0)
static const uint8_t PIN_ENC_B     = 3; // Encoder channel B (INT1)

// ---------------- Encoder State ------------------
volatile long encoderCount = 0;  
volatile uint8_t lastAB   = 0;   // Last 2-bit state of encoder A/B

// A 16-entry lookup table for quadrature decoding.
// Index = (oldState << 2) | newState.
static const int8_t QDEC_LOOKUP[16] = {
  0, +1, -1,  0,   // 0..3
 -1,  0,  0, +1,   // 4..7
 +1,  0,  0, -1,   // 8..11
  0, -1, +1,  0    // 12..15
};

// ---------------- PID Control Parameters -------------
// Starting values tuned for a small N20 motor
float Kp = 2;   // Proportional gain
float Ki = 0;   // Integral gain
float Kd = 0;   // Derivative gain

float integral = 0.0;
long  lastError = 0;

long targetPosition = 0;  // Set via serial commands

// Control loop timing (every 20 ms)
unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL = 20; // ms
const float dt = CONTROL_INTERVAL / 1000.0;  // dt in seconds

// ---------------- Serial Input Buffer ------------
#define CMD_BUFFER_SIZE 16
char cmdBuffer[CMD_BUFFER_SIZE];
uint8_t cmdIndex = 0;

// --------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("PID Position Control for N20 Motor");
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

void loop() {
  // 1) Handle incoming serial data (new target positions)
  readSerial();

  // 2) Run PID control loop at fixed interval
  unsigned long now = millis();
  if (now - lastControlTime >= CONTROL_INTERVAL) {
    lastControlTime = now;

    // Safely copy the volatile encoderCount
    long currentPos;
    noInterrupts();
    currentPos = encoderCount;
    interrupts();

    // PID calculations
    long error = targetPosition - currentPos;
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    lastError = error;

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
    Serial.print("  Err=");
    Serial.print(error);
    Serial.print("  Out=");
    Serial.println(output, 1);
  }
}

// --------------------------------------------------
// driveMotor: Drives the motor using two PWM signals.
// Positive command drives forward; negative drives reverse.
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
// isrEnc: ISR for encoder channels; uses lookup table to update encoderCount.
void isrEnc() {
  uint8_t a = digitalRead(PIN_ENC_A);
  uint8_t b = digitalRead(PIN_ENC_B);
  uint8_t newAB = (a << 1) | b;
  uint8_t index = (lastAB << 2) | newAB;
  encoderCount += QDEC_LOOKUP[index & 0x0F];
  lastAB = newAB;
}
