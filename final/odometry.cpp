#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4IMU.h>
#include "odometry.h"
using namespace Pololu3piPlus32U4;

#define PI 3.14159


Odometry::Odometry(float diaL, float diaR, float w, int nL, int nR, int gearRatio){
  _diaL = diaL;
  _diaR = diaR;
  _w = w;
  _nL = nL;
  _nR = nR;
  _gearRatio = gearRatio;

  _x = 0;
  _y = 0;
  _theta = 0;

  _left_encoder_counts_prev = 0;
  _right_encoder_counts_prev = 0;
}

// USE ODOMETRY FORMULAS TO CALCULATE ROBOT'S NEW POSITION AND ORIENTATION
void Odometry::update_odom(int left_encoder_counts, int right_encoder_counts, float &x, float &y, float &theta){
	
  // do odometry math to calculate robot's new position and orientation
  double deltaL = (double)((left_encoder_counts - _left_encoder_counts_prev)*PI*_diaL)/(_nL *_gearRatio);
  double deltaR = (double)((right_encoder_counts - _right_encoder_counts_prev)*PI*_diaR)/(_nR *_gearRatio); 
  Serial.print("dL="); Serial.print(deltaL,6);
  Serial.print(" dR="); Serial.println(deltaR,6);

  
  _theta += (deltaR-deltaL)/_w;

  // CALCULATE _x BASED ON THE FORMULA FROM THE LECTURES
  _x = ((float)((float)(deltaL+deltaR)/2) * cos(_theta)); //update x   // _theta passed to cos function in radians
  // CALCULATE _y BASED ON THE FORMULA FROM THE LECTURES #TODO: LOOK AT DIFFERENCE BETWEEN (+) and (-)
  _y = ((float)((float)(deltaL+deltaR)/2) * sin(_theta)); //update y   // _theta passed to sin function in radians

  // CALCULATE CUMULATIVE x, AND CUMULATIVE y. 
  //AKA UPDATE THE VALUE OF &x AND &y (THE PARAMETERS OF THE update_odom FUNCTIONS, WHICH ARE PASSED BY REFERENCE)
  //REMINDER: CUMULATIVE theta IS EQUAL TO _theta.
  x = _x;
  y = _y;
  theta = _theta;


  // PRINT THE x, y, theta VALUES ON SERIAL MONITOR
       //Odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta); // x, y, theta call by reference
      //  Serial.print(" x:  "); 
      //  Serial.println(x); 
      //  Serial.print(" y:  ");
      //  Serial.println(y);
      //  Serial.print(" theta:  ");
      //  Serial.println(theta);
      //  Serial.println("------------");

  // Save the current encoder values as the "previous" values, so you can use it in the next iteration
  _left_encoder_counts_prev = left_encoder_counts;
  _right_encoder_counts_prev = right_encoder_counts;
  delay(10);

}
