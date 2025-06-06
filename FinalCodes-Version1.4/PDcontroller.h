#ifndef PDcontroller_h
#define PDcontroller_h
#include <Pololu3piPlus32U4.h>
using namespace Pololu3piPlus32U4;

class PDcontroller{
  public:
    PDcontroller(float kp, float kd, double minOutput, double maxOutput);
    double update(double value, double target_value); //may need to update with additional variables passed in this function
    
  private:
    //Add private variables here
    // P controller variables
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
};

#endif
