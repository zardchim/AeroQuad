#ifndef _Flight_Control_Child_Robot_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

#include "FlightControlVariable.h"
 
#define LEFT  MOTOR1
#define MIDDLE MOTOR2
#define RIGHT  MOTOR3

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  //motorCommand[FRONT_LEFT]  = throttle - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  //motorCommand[FRONT_RIGHT] = throttle - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[LEFT] = throttle -(YAW_DIRECTION*motorAxisCommandYaw);
  motorCommand[MIDDLE] = throttle;
  motorCommand[RIGHT] = throttle -(YAW_DIRECTION*motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

