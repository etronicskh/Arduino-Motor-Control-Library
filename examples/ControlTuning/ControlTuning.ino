#include <MotorControl.h>
#include <DueTimer.h>

#define CON_FREQ 25

volatile float refSpeed = 0.0;

MotorControl motor(2, 3, 1, CON_FREQ, 0.1, 0.1);

void motorControlCallback()
{
  int32_t pwm = motor.controlCalc(refSpeed);
  // Set PWM to motor driver here
}

void setup() {
  Serial.begin(9600);
  motor.setMotorParameter(11, 25.0); // CPR, Gear Ratio
  motor.setInputGain(1.3);
  motor.setInputDifferentialThreshold(0.5);
  motor.setEncoderReadQuadrature(true);

  Timer1.attachInterrupt(motorControlCallback).setFrequency(CON_FREQ).start();
}

void loop() {
  // Set refSpeed here
  Serial.println(motor.getSpeed());
}