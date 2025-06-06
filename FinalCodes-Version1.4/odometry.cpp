#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include "odometry.h"
#include "printOLED.h"
using namespace Pololu3piPlus32U4;

#define PI 3.14159

PrintOLED printOLED;

Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio, bool dead_reckoning){
  _diaL = diaL;
  _diaR = diaR;
  _w = w;
  _nL = nL;
  _nR = nR;
  _gearRatio = gearRatio;
  _deadreckoning = dead_reckoning;

  _x = 0;
  _y = 0;
  _theta = 0;

  _left_encoder_counts_prev = 0;
  _right_encoder_counts_prev = 0;

  if(_deadreckoning){ // if using dead reckoning, initialize and calibrate IMU
    Wire.begin();
    _imu.init();
    _imu.enableDefault();

    //calibrate IMU
    int total = 0;
    for (int i = 0; i < 100; i++)
    {
      _imu.readGyro();
      total += _imu.g.z;
      delay(1);
    }
    _IMUavg_error = total / 100;  
  }
}

void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta) {
    // Constants
    float cL = (_diaL * PI) / (_nL * _gearRatio); // Distance per click for left wheel
    float cR = (_diaR * PI) / (_nR * _gearRatio); // Distance per click for right wheel
    
    // Calculate distance traveled by each wheel
    float dL = (left_encoder_counts - _left_encoder_counts_prev) * cL;
    float dR = (right_encoder_counts - _right_encoder_counts_prev) * cR;
    
    // Update previous encoder counts
    _left_encoder_counts_prev = left_encoder_counts;
    _right_encoder_counts_prev = right_encoder_counts;
    
    // Compute change in orientation
    float d_theta;
    if (_deadreckoning) {
        _imu.readGyro();
        d_theta = (_imu.g.z - _IMUavg_error) * 0.001; // Convert gyro reading to radians
    } else {
        d_theta = (dR - dL) / _w; // Differential drive odometry formula
    }

    // Update cumulative theta
    _theta += d_theta;

    // Compute displacement
    float d_center = (dL + dR) / 2.0;
    
    // Update x and y positions
    _x += d_center * cos(_theta);
    _y += d_center * sin(_theta);
    
    // Pass updated values back
    x = _x;
    y = _y;
    theta = _theta;
    
    // Print values on OLED and Serial Monitor
    printOLED.print_odom(_x, _y, _theta);
    
    
    
    
    Serial.print("x: "); Serial.println(_x);
    Serial.print("y: "); Serial.println(_y);
    Serial.print("theta: "); Serial.println(_theta);
}
