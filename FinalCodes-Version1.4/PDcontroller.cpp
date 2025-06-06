#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h"
using namespace Pololu3piPlus32U4;

PDcontroller::PDcontroller(float kp, float kd, double minOutput, double maxOutput) {
  // initialize the private varaibles from Pcontroller.h here
  this->kp = kp;
  this->minOutput = minOutput;
  this->maxOutput = maxOutput;
  error = 0.0;
  output = 0.0;
  clampOut = 0.0;
  
  // Initialize PD-specific variables
  this->kd = kd;
  lastError = 0.0;
  lastTime = millis();
  firstRun = true;
}

double PDcontroller::update(double value, double target_value){
  //Controller math here
  /*Hints: To add damping (derivative), you must have something to
           keep track of time for the rate of change.
           
           Also note that the first time PD controller is ran, we only have
           the P component, so consider using an if-else statement.

           
          Again, you need to return actuator controller value (_clampOut)
          */
  // Get current time and compute elapsed time (dt) in seconds
  unsigned long currentTime = millis();
  double dt = (currentTime - lastTime) / 1000.0; // convert ms to seconds
  lastTime = currentTime;
  
  // Calculate the error
  error = target_value - value;
  
  // Proportional term
  double pTerm = kp * error;
  
  // Derivative term (only if not the first run)
  double dTerm = 0.0;
  if(!firstRun && dt > 0) {
    dTerm = kd * ((error - lastError) / dt);
  }
  lastError = error;
  firstRun = false;
  
  // Combine the terms and constrain the result like in Pcontroller
  output = pTerm + dTerm;
  clampOut = constrain(output, minOutput, maxOutput);
  
  return clampOut;         

}
