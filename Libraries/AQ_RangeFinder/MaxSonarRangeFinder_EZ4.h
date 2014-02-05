#ifndef _AEROQUAD_MAX_SONAR_RANGE_FINDER_EZ4_H_
#define _AEROQUAD_MAX_SONAR_RANGE_FINDER_EZ4_H_

#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(BOARD_aeroquad32)

#include "RangeFinder.h"

#define SPIKE_FILTER_MARGIN 500 // mm ; changes bigger than this need two samples to take effect

#define RANGER_COUNT ((sizeof(rangeFinders) / sizeof(struct rangeFinder)))

// last reading used for 'spike' filtter
short lastRange;

void inititalizeRangeFinders() {
	takeRangeReading();
}

void updateRangeFinders() {

  short range = (short)(	requestRange()	*	(10)	);

  // Following will accept the sample if it's either withing "spike margin" of last raw reading or previous accepted reading
  // otherwise it's ignored as noise
  
  if ((abs(range - lastRange[rangerToRead]) < SPIKE_FILTER_MARGIN)) {
    rangeFinderRange[rangeFinders[rangerToRead].target] = (float)range / 1000.0;
  }
  lastRange = range;

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

#endif 
#endif








