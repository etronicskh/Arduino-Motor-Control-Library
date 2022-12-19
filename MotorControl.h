#ifndef MOTORCONTROl_H
#define MOTORCONTROl_H

#include <Encoder.h>

class MotorControl{
  public:
    MotorControl(uint8_t chA, uint8_t chB, float motorGain, uint32_t controlFreq, float Kp, float Ki);
    void setMotorParameter(uint32_t cpr, float gearRatio);
    void setControlParameter(float Kp, float Ki);
    void setControlFrequency(uint32_t controlFreq);
    void setInputGain(float inputGain);
    void setInputLimit(uint32_t inputLimit);
    void setOutputLimit(uint32_t outputLimit);
    void setEncoderReadQuadrature(bool readQuadrature);
    void setInputDifferentialThreshold(float inputdifferentialThresh);
    int32_t controlCalc(volatile float &refSpeed);
    float getSpeed();
    long long getEncoder();

  private:
    Encoder encoder;
    volatile uint32_t _controlFreq;
    volatile float _motorGain;
    volatile float _kp, _ki;

    // Control Limit
    volatile float _inputGain;
    volatile float _inputLimit;
    volatile float _outputLimit;

    // Optional setup
    volatile float _readQuadrature;
    volatile float _inputDifferentialThresh;

    // Control Variables
    volatile long long _encoder, _encoderLast;
    volatile float _error;
    volatile float _eSum;
    volatile float _refSpeed, _refSpeedLast;
    volatile float _currentSpeed;
    volatile float _output, _outputLast;

    // Average speed for reading
    volatile float _meanSpeed;
    volatile float _storeCurrentSpeed[10];
    volatile int _storePosition;
};

MotorControl::MotorControl(uint8_t chA, uint8_t chB, float motorGain, uint32_t controlFreq, float Kp, float Ki):encoder(chA, chB), _motorGain(motorGain), _controlFreq(controlFreq), _kp(Kp), _ki(Ki){
  _inputGain = 1.0;
  _inputLimit = 60;
  _outputLimit = 255;
  _readQuadrature = 4.0;
  _inputDifferentialThresh = 0.0;
  _refSpeed = 0.0;
  _refSpeedLast = 0.0;
  _error = 0;
  _eSum = 0;
  
  _storePosition = 0;
  for(int i=0; i<10; i++) _storeCurrentSpeed[i]=0;
}

void MotorControl::setMotorParameter(uint32_t cpr, float gearRatio){
  _motorGain = (60.0*(float)_controlFreq)/(float)cpr*gearRatio;
}

void MotorControl::setControlParameter(float Kp, float Ki){
  _kp = Kp;
  _ki = Ki;
}

void MotorControl::setControlFrequency(uint32_t controlFreq){
  _controlFreq = controlFreq;
}

void MotorControl::setInputGain(float inputGain){
  _inputGain = inputGain;
}

void MotorControl::setInputLimit(uint32_t inputLimit){
  _inputLimit = inputLimit;
}

void MotorControl::setOutputLimit(uint32_t outputLimit){
  _outputLimit = outputLimit;
}

void MotorControl::setEncoderReadQuadrature(bool readQuadrature){
  _readQuadrature = readQuadrature? 4.0 : 1.0;
}

void MotorControl::setInputDifferentialThreshold(float inputDifferentialThresh){
  _inputDifferentialThresh = inputDifferentialThresh;
}

int32_t MotorControl::controlCalc(volatile float &refSpeed){
  _refSpeed = refSpeed;
  _refSpeed = constrain(_refSpeed, -_inputLimit, _inputLimit);
  refSpeed = _refSpeed;
  _encoder = encoder.read();
  _currentSpeed = ((float)(_encoder-_encoderLast)/_readQuadrature)*_motorGain;
  _error = _refSpeed - _currentSpeed;

  // Restore eSum if there's new refSpeed
  if(abs(_refSpeed-_refSpeedLast)>_inputDifferentialThresh){
    _eSum = 0;
  }else{
    _eSum += _error;
  }

  // Calculate PI ouput
  _output = _kp*_error + _ki*_eSum;
  _output = (_output + _outputLast)/2.0;
  _output = constrain(_output, -_outputLimit, _outputLimit);

  // Store current value for next routine
  _refSpeedLast = _refSpeed;
  _encoderLast = _encoder;
  _outputLast = _output;

  // Store current speed to array
  _storeCurrentSpeed[_storePosition] = _currentSpeed;
  _storePosition++;
  if(_storePosition>9) _storePosition=0;
  
  return (int32_t)_output;
}

float MotorControl::getSpeed(){
  _meanSpeed = 0.0;
  for(int i=0; i<10; i++) _meanSpeed += _storeCurrentSpeed[i];
  return (_meanSpeed/10.0);
}

long long MotorControl::getEncoder(){
  return encoder.read();
}

#endif