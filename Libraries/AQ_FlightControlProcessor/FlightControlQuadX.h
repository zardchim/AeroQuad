#ifndef _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_
#define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

/*
       CW  0....Front....0 CCW
           ......***......    
           ......***......    
           ......***......    
      CCW  0....Back.....0  CW
*/

#include "FlightControlVariable.h"

#ifdef OLD_MOTOR_NUMBERING  
  #define FRONT_LEFT  MOTOR1
  #define REAR_RIGHT  MOTOR2
  #define FRONT_RIGHT MOTOR3
  #define REAR_LEFT   MOTOR4
#else
  #define FRONT_LEFT  MOTOR1
  #define FRONT_RIGHT MOTOR2
  #define REAR_RIGHT  MOTOR3
  #define REAR_LEFT   MOTOR4
#endif
#define LASTMOTOR   (MOTOR4+1)

int motorMaxCommand[4] = {0,0,0,0};
int motorMinCommand[4] = {0,0,0,0};
int motorConfiguratorCommand[4] = {0,0,0,0};

void applyMotorCommand() {
  motorCommand[FRONT_LEFT]  = throttle - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = throttle - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = throttle + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = throttle + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
}

#endif // #define _AQ_PROCESS_FLIGHT_CONTROL_X_MODE_H_

