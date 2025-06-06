#include <Pololu3piPlus32U4.h>
#include "PIDcontroller.h"
using namespace Pololu3piPlus32U4;

PIDcontroller::PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i) {

    // Initialize PID variables
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->minOutput = minOutput;
  this->maxOutput = maxOutput;
  this->clamp_i = clamp_i;
  
  error = 0.0;
  output = 0.0;
  clampOut = 0.0;
  
  lastError = 0.0;
  lastTime = millis();
  firstRun = true;
  
  integral = 0.0;
}

double PIDcontroller::update(double value, double target_value){

    // Get current time and compute elapsed time in seconds
  unsigned long currentTime = millis();
  double dt = (currentTime - lastTime) / 1000.0; // convert ms to seconds
  lastTime = currentTime;
  
  // Calculate the error
  error = target_value - value;
  
  // Proportional term
  double pTerm = kp * error;
  
  // Integral term
  integral += error * dt;
  if (integral > clamp_i) integral = clamp_i;
  if (integral < -clamp_i) integral = -clamp_i;
  double iTerm = ki * integral;
  
  // Derivative term (only if not the first run)
  double dTerm = 0.0;
  if(!firstRun && dt > 0) {
    dTerm = kd * ((error - lastError) / dt);
  }
  lastError = error;
  firstRun = false;
  
  // Combine the terms and constrain the result
  output = pTerm + iTerm + dTerm;
  clampOut = constrain(output, minOutput, maxOutput);
  
  return clampOut;         
}
  

