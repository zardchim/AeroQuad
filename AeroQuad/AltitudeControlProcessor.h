// FlightControl.pde is responsible for combining sensor measurements and
// transmitter commands into motor commands for the defined flight configuration (X, +, etc.)
//////////////////////////////////////////////////////////////////////////////
/////////////////////////// calculateFlightError /////////////////////////////
//////////////////////////////////////////////////////////////////////////////

#ifndef _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
#define _AQ_ALTITUDE_CONTROL_PROCESSOR_H_


#if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder

#define INVALID_THROTTLE_CORRECTION -1000
#define ALTITUDE_BUMP_SPEED 0.01

int altitudeHoldThrottleCorrection;

/**
 * processAltitudeHold
 * 
 * This function is responsible to process the throttle correction 
 * to keep the current altitude if selected by the user 
 */
 
//test
float sonar_PID_time = 0;
float sonar_P = 100;
float sonar_I = 100;
float sonar_I_integratedError;
float sonar_D = -400;
float sonar_lastError = 0;

float sonar_zvel = 0;
float sonar_zvel_time = 0;
float sonar_last_range = 0;

float sonar_zaccel = 0;
float sonar_zaccel_time = 0;
float sonar_last_zvel = 0;

float sonar_height = 0;
float sonar_height_dt = 0;

/*
 float update_sonar_PID(float targetPosition, float currentPosition) {

  // AKA PID experiments
  const float delta_sonar_PIDTime = (currentTime - sonar_PID_time) / 1000000.0;
  sonar_PID_time = currentTime;  // AKA PID experiments
  
  float error = targetPosition - currentPosition;

  if (inFlight) {
    sonar_I_integratedError += error * delta_sonar_PIDTime;
  }
  else {	
    sonar_I_integratedError = 0.0;
  }
  sonar_I_integratedError = constrain(sonar_I_integratedError, -500, 500);
  float dTerm = sonar_D * (currentPosition - sonar_lastError) / (delta_sonar_PIDTime * 100); // dT fix from Honk
  sonar_lastError = currentPosition;

  return (sonar_P * error) + (sonar_I * sonar_I_integratedError) + dTerm;
}
*/

void calculate_sonar_zvel(float current_range) {

	sonar_zvel = (current_range - sonar_last_range)/ (currentTime - sonar_zvel_time);
	sonar_last_range = current_range;
	sonar_zvel_time = currentTime;

}

void calculate_sonar_zaccel() {

	sonar_zaccel = (sonar_zvel - sonar_last_zvel)/ (currentTime - sonar_zaccel_time);
	sonar_zaccel_time = currentTime;
	sonar_last_zvel = sonar_zvel;
}

void calculate_sonar_height() {

	float sonar_height_deltatime = (currentTime - sonar_height_dt);
	sonar_height_dt = currentTime;
	
	sonar_height = (sonar_zvel*sonar_height_deltatime)+(1/2)*(sonar_zaccel)*(sonar_height_deltatime)*(sonar_height_deltatime);

}
	
void processAltitudeHold()
{
  // ****************************** Altitude Adjust *************************
  // Thanks to Honk for his work with altitude hold
  // http://aeroquad.com/showthread.php?792-Problems-with-BMP085-I2C-barometer
  // Thanks to Sherbakov for his work in Z Axis dampening
  // http://aeroquad.com/showthread.php?359-Stable-flight-logic...&p=10325&viewfull=1#post10325

	//calculate_sonar_zvel(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
	//calculate_sonar_zaccel;	
	//calculate_sonar_height;
		
  if (altitudeHoldState == ON) {

	altitudeHoldThrottleCorrection = INVALID_THROTTLE_CORRECTION;
    // computer altitude error!
    #if defined AltitudeHoldRangeFinder
      if (isOnRangerRange(rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX])) {
        if (sonarAltitudeToHoldTarget == INVALID_RANGE) {
          sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
        }
		
		//test
		analogWrite(22,130);
		analogWrite(23,0);

		const float delta_sonar_PIDTime = (currentTime - sonar_PID_time) / 1000000.0;
		sonar_PID_time = currentTime;
		
		float sonar_error = sonarAltitudeToHoldTarget - rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
		
		if (inFlight) {
			sonar_I_integratedError += sonar_error * delta_sonar_PIDTime;
		}
		else {	
			sonar_I_integratedError = 0.0;
		}
		sonar_I_integratedError = constrain(sonar_I_integratedError, -500, 500);
		float sonar_dTerm = PID[SONAR_ALTITUDE_HOLD_PID_IDX].D * (rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] - sonar_lastError) / (delta_sonar_PIDTime * 100);
		sonar_lastError = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
		
		altitudeHoldThrottleCorrection = (sonar_error * PID[SONAR_ALTITUDE_HOLD_PID_IDX].P) + (sonar_I_integratedError * PID[SONAR_ALTITUDE_HOLD_PID_IDX].I) + sonar_dTerm;
		//test

		//altitudeHoldThrottleCorrection = update_sonar_PID(sonarAltitudeToHoldTarget,rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX]);
		//altitudeHoldThrottleCorrection = updatePID(sonarAltitudeToHoldTarget, rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX], &PID[SONAR_ALTITUDE_HOLD_PID_IDX]);
		altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      }
    #endif
    #if defined AltitudeHoldBaro
      if (altitudeHoldThrottleCorrection == INVALID_THROTTLE_CORRECTION) {
        altitudeHoldThrottleCorrection = updatePID(baroAltitudeToHoldTarget, getBaroAltitude(), &PID[BARO_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
      }
    #endif        
    if (altitudeHoldThrottleCorrection == INVALID_THROTTLE_CORRECTION) {
      throttle = receiverCommand[THROTTLE];
      return;
    }
    
    // ZDAMPENING COMPUTATIONS
    #if defined AltitudeHoldBaro || defined AltitudeHoldRangeFinder
      float zDampeningThrottleCorrection = -updatePID(0.0, estimatedZVelocity, &PID[ZDAMPENING_PID_IDX]);
      zDampeningThrottleCorrection = constrain(zDampeningThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
    #endif

    
    if (abs(altitudeHoldThrottle - receiverCommand[THROTTLE]) > altitudeHoldPanicStickMovement) {
      altitudeHoldState = ALTPANIC; // too rapid of stick movement so PANIC out of ALTHOLD
    } 
    else {
      
      if (receiverCommand[THROTTLE] > (altitudeHoldThrottle + altitudeHoldBump)) { // AKA changed to use holdThrottle + ALTBUMP - (was MAXCHECK) above 1900
        #if defined AltitudeHoldBaro
          baroAltitudeToHoldTarget += ALTITUDE_BUMP_SPEED;
        #endif
        #if defined AltitudeHoldRangeFinder
          float newalt = sonarAltitudeToHoldTarget + ALTITUDE_BUMP_SPEED;
          if (isOnRangerRange(newalt)) {
            sonarAltitudeToHoldTarget = newalt;
          }
        #endif
      }
      
      if (receiverCommand[THROTTLE] < (altitudeHoldThrottle - altitudeHoldBump)) { // AKA change to use holdThorrle - ALTBUMP - (was MINCHECK) below 1100
        #if defined AltitudeHoldBaro
          baroAltitudeToHoldTarget -= ALTITUDE_BUMP_SPEED;
        #endif
        #if defined AltitudeHoldRangeFinder
          float newalt = sonarAltitudeToHoldTarget - ALTITUDE_BUMP_SPEED;
          if (isOnRangerRange(newalt)) {
            sonarAltitudeToHoldTarget = newalt;
          }
        #endif
      }
    }
    throttle = altitudeHoldThrottle + altitudeHoldThrottleCorrection + zDampeningThrottleCorrection;

  /*
	#if defined AltitudeHoldRangeFinder
		sonarAltitudeToHoldTarget = rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX];
		
		analogWrite(22,130);
		analogWrite(23,0);
		
	    altitudeHoldThrottleCorrection = updatePID(sonarAltitudeToHoldTarget, rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX], &PID[SONAR_ALTITUDE_HOLD_PID_IDX]);
        altitudeHoldThrottleCorrection = constrain(altitudeHoldThrottleCorrection, minThrottleAdjust, maxThrottleAdjust);
  */
  }
  else {
    throttle = receiverCommand[THROTTLE];
	
	//test
	pinMode(22,0);
	pinMode(23,0);
	
  }
}

#endif

#endif // _AQ_ALTITUDE_CONTROL_PROCESSOR_H_
