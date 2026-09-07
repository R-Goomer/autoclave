// =============================================================================
// PID_v1.h — Manual PID controller (drop-in replacement for the Arduino PID
// Library API).
//
// WHY THIS FILE EXISTS:
//   The Cirkit AI simulator has a conflicting library ("PreMo") that also
//   ships a PID_v1.h, which breaks the build. This local copy lives in the
//   sketch folder, which the Arduino build system searches FIRST, so it
//   shadows the PreMo copy and resolves the conflict.
//
//   It implements the same public API as the Arduino PID Library
//   (Brett Beauregard) so you can swap it for the real library later:
//     - PID(Input, Output, Setpoint, Kp, Ki, Kd, Direction)
//     - Compute()
//     - SetOutputLimits(Min, Max)
//     - SetSampleTime(ms)
//     - SetMode(MANUAL | AUTOMATIC)
//     - SetTunings(Kp, Ki, Kd)
//
//   Anti-windup: conditional integration — the integral term stops growing
//   once it saturates at the output limits, preventing the big overshoot you
//   get after a long 100%-power warmup.
// =============================================================================

#ifndef PID_v1_h
#define PID_v1_h

// --- Mode / direction constants (match the Arduino PID Library) ---
#define MANUAL 0
#define AUTOMATIC 1
#define DIRECT 0
#define REVERSE 1

class PID {
public:
  // Constructor. Input/Output/Setpoint are pointers to the live variables.
  PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
      double Kd, int ControllerDirection);

  // Runs one PID calculation. Returns true if a new output was computed
  // (i.e. the sample time has elapsed).
  bool Compute();

  // Clamps the output (and integral) to [Min, Max].
  void SetOutputLimits(double Min, double Max);

  // Sets the PID gains. Ki/Kd are scaled by the sample time internally.
  void SetTunings(double Kp, double Ki, double Kd);

  // Sets the fixed calculation interval in milliseconds.
  void SetSampleTime(int NewSampleTime);

  // MANUAL = output untouched; AUTOMATIC = Compute() drives the output.
  void SetMode(int Mode);

private:
  double *in;        // pointer to input (measured value)
  double *out;       // pointer to output (controller output)
  double *setpoint;  // pointer to setpoint (target)

  double kp, ki, kd; // gains (ki/kd already scaled by sample time)
  double outMin, outMax;
  double integral;   // accumulated integral term
  double lastInput;  // previous input (for derivative-on-measurement)
  unsigned long lastTime;
  unsigned long sampleTime;
  int direction;     // DIRECT or REVERSE
  bool inAuto;       // AUTOMATIC mode active?
};

// =============================================================================
// IMPLEMENTATION
// =============================================================================

PID::PID(double *Input, double *Output, double *Setpoint, double Kp, double Ki,
         double Kd, int ControllerDirection) {
  in = Input;
  out = Output;
  setpoint = Setpoint;
  inAuto = false;
  direction = (ControllerDirection == REVERSE) ? REVERSE : DIRECT;

  SetOutputLimits(0, 255); // default 8-bit range
  sampleTime = 100;        // default 100 ms
  SetTunings(Kp, Ki, Kd);
}

bool PID::Compute() {
  if (!inAuto)
    return false;

  unsigned long now = millis();
  unsigned long timeChange = now - lastTime;
  if (timeChange < sampleTime)
    return false;

  double input = *in;
  double error = *setpoint - input;
  double dInput = input - lastInput; // derivative on measurement (no kick)

  // Proportional
  double pTerm = kp * error;

  // Integral with conditional-integration anti-windup
  integral += ki * error;
  if (integral > outMax)
    integral = outMax;
  else if (integral < outMin)
    integral = outMin;

  // Derivative
  double dTerm = kd * dInput;

  double output = pTerm + integral - dTerm;
  if (output > outMax)
    output = outMax;
  else if (output < outMin)
    output = outMin;

  *out = output;
  lastInput = input;
  lastTime = now;
  return true;
}

void PID::SetOutputLimits(double Min, double Max) {
  if (Min > Max)
    return;
  outMin = Min;
  outMax = Max;

  if (inAuto) {
    if (*out > outMax)
      *out = outMax;
    else if (*out < outMin)
      *out = outMin;
    if (integral > outMax)
      integral = outMax;
    else if (integral < outMin)
      integral = outMin;
  }
}

void PID::SetTunings(double Kp, double Ki, double Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0)
    return;

  double sampleSeconds = ((double)sampleTime) / 1000.0;
  kp = Kp;
  ki = Ki * sampleSeconds;
  kd = Kd / sampleSeconds;

  if (direction == REVERSE) {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
}

void PID::SetSampleTime(int NewSampleTime) {
  if (NewSampleTime > 0) {
    double ratio = (double)NewSampleTime / (double)sampleTime;
    ki *= ratio;
    kd /= ratio;
    sampleTime = (unsigned long)NewSampleTime;
  }
}

void PID::SetMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto) {
    // Fresh start on entering AUTOMATIC: seed integral from current output
    // and reset derivative memory. This prevents windup from a prior
    // 100%-power warmup phase.
    integral = *out;
    lastInput = *in;
    if (integral > outMax)
      integral = outMax;
    else if (integral < outMin)
      integral = outMin;
    lastTime = millis();
  }
  inAuto = newAuto;
}

#endif // PID_v1_h