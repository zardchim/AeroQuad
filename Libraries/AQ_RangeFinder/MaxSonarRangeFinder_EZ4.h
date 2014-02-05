#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_EZ4_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_EZ4_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(BOARD_aeroquad32)

#include "RangeFinder.h"

#define SPIKE_FILTER_MARGIN 300 // mm ; changes bigger than this need two samples to take effect

#define RANGER_COUNT ((sizeof(rangeFinders) / sizeof(struct rangeFinder)))

//The Arduino Wire library uses the 7-bit version of the address, so the code example uses 0x70 instead of the 8‑bit 0xE0
#define SensorAddress byte(0x70)
//The Sensor ranging command has a value of 0x51
#define RangeCommand byte(0x51)
//These are the two commands that need to be sent in sequence to change the sensor address

//FIR Filter Variable
float first_previous = 0;
float second_lastRange = 0;

// last reading used for 'spike' filter
short lastRange;

void takeRangeReading();
word requestRange();
float thirdorder_fir (float , float , float );

void inititalizeRangeFinders() {
	takeRangeReading();
}

void updateRangeFinders() {

  float range = (float) (requestRange()	*	(10)	);

  range = thirdorder_fir(range, lastRange, second_lastRange);
  rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] = (float)range / 1000.0;
  
  // Following will accept the sample if it's either withing "spike margin" of last raw reading or previous accepted reading
  // otherwise it's ignored as noise  
  if ((abs(range - lastRange) < SPIKE_FILTER_MARGIN)) {
    rangeFinderRange[ALTITUDE_RANGE_FINDER_INDEX] = (float)range / 1000.0;
  }
  second_lastRange = lastRange;
  lastRange = range;  
    
  takeRangeReading();
}

//Commands the sensor to take a range reading
void takeRangeReading(){
       Wire.beginTransmission(SensorAddress);             //Start addressing 
       Wire.write(RangeCommand);                             //send range command 
       Wire.endTransmission();                                  //Stop and do something else now
}    

//Returns the last range that the sensor determined in its last ranging cycle in centimeters. Returns 0 if there is no communication. 
word requestRange(){ 
    Wire.requestFrom(SensorAddress, byte(2));
            if(Wire.available() >= 2){                            //Sensor responded with the two bytes 
           byte HighByte = Wire.read();                        //Read the high byte back 
           byte LowByte = Wire.read();                        //Read the low byte back 
           word range = word(HighByte, LowByte);         //Make a 16-bit word out of the two bytes for the range 
           return range; 
        }
        else { 
        return word(0);                                        //Else nothing was received, return 0 
    }
}

float thirdorder_fir (float current_value, float first_previous, float second_previous) {
	float first_coefficient = 0.6;
	float second_coefficient = 0.3;
	float third_coefficient = 0.1;
	
	return (first_coefficient*current_value + second_coefficient*first_previous + third_coefficient*second_previous);
}
	


#endif 
#endif








