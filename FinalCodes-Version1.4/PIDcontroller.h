#ifndef PIDcontroller_h
#define PIDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PIDcontroller{
  public:
    PIDcontroller(float kp, float ki, float kd, double minOutput, double maxOutput, double clamp_i);
    double update(double value, double target_value);
    
  private:
    /*Add variables from your PDController, then add varaibles for ki, your clamp
    for i component, and accumulated error.*/

    float kp;
    double minOutput;
    double maxOutput;
    double error;
    double output;
    double clampOut;
    
    // PD additional variables
    float kd;
    double lastError;
    unsigned long lastTime; 
    bool firstRun; 

    float ki;
    float integral;
    float clamp_i;
};

#endif
