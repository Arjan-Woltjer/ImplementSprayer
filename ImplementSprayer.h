/*
  ImplementSprayer - a library for a sprayer
 Copyright (C) 2011-2015 J.A. Woltjer.
 All rights reserved.
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU Lesser General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU Lesser General Public License for more details.
 
 You should have received a copy of the GNU Lesser General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ImplementSprayer_h
#define ImplementSprayer_h

#include <Arduino.h>
#include <EEPROM.h>
#include "VehicleTractor.h"
#include "ConfigImplementSprayer.h"
#include "Language.h"

#ifdef GPS
#include "VehicleGps.h"
#endif

// software version of this library
#define SPRAYER_VERSION 0.1

class ImplementSprayer {
private:
  //-------------
  // data members
  //-------------
  
  // Default angle calibration set
  byte pwm_calibration_points[16];
  byte pwm_calibration_data[16];  //EEPROM 100
  
  int flow_calibration;  //EEPROM 120
  
  int gear_pulses;
  int rounds;
  int flow_pulses;
  unsigned long volume;
  byte pulses_temp;
  
  boolean gear_puls;
  boolean flow_puls;
   
  int actual_flow;
  int needed_flow;
  int calculated_flow;
  int setpoint_flow;

  int dose;
  int actual_dose;
  
  byte teeth;  //EEPROM 130
  byte pumps;  //EEPROM 140
  byte width;  //EEPROM 150
  
  float cc_per_omw; //EEPROM 160
  
  // Update timer
  unsigned long update_age;
  unsigned long update_age_flag;
  boolean update_flag;

  // Variables concerning adjust loop
  byte setpoint_pwm;
  boolean alarm;
  
  // PID
  int delta_flow;
  int delta_hist[20];
  long delta_sum;
  int delta_avg;
  int delta_delta;
  
  float P;
  float I;
  float D;
  
  byte KP;
  byte KI;
  byte KD;
 
  byte hist_count;
  byte hist_time;
  
  // Objects
#ifdef GPS
  VehicleGps * gps;
#else
  VehicleTractor * tractor;
#endif

  //REMOVE after testing
  int temp;

  //-------------------------------------------------------------
  // private member functions implemented in ImplementSprayer.cpp
  //-------------------------------------------------------------
  void calculateActualFlow();
  void calculateNeededFlow();
  void calculateCalculatedFlow();
  void calculateSetpointFlow(byte _mode);
  void calculateAlarm(byte _mode);
  void calculateSetpointPwm();

  void setDose(int _correction);
  void readDose();
  
  boolean readCalibrationData();
  void printCalibrationData();
  void writeCalibrationData();
  void wipeCalibrationData();

public:
  // -----------------------------------------------------------
  // public member functions implemented in ImplementSprayer.cpp
  // -----------------------------------------------------------

  // Constructor
#ifdef GPS
  ImplementSprayer(VehicleGps * _gps);
#else
  ImplementSprayer(VehicleTractor * _tractor);
#endif

  void update(byte _mode, int _buttons);
  void stop();
  int calibratePump();

  // ----------------------------------------------------------------
  // public inline member functions implemented in ImplementSprayer.h
  // ----------------------------------------------------------------
  inline void resetFlowPulses(){
    flow_pulses = 0;
  }
  
  inline void resetGearPulses(){
    gear_pulses = 0;
  }
  
  inline boolean resetCalibration(){
    return readCalibrationData();
  }
  
  inline void commitCalibration(){
    writeCalibrationData();
    printCalibrationData();
  }
  
  // -------
  // Getters
  // -------
  inline byte getTeeth(){
    return teeth;
  }
  
  inline byte getPumps(){
    return pumps;
  }
  
  inline byte getWidth(){
    return width;
  }
    
  inline byte getKP(){
    return KP;
  }
  
  inline byte getKI(){
    return KI;
  }

  inline byte getKD(){
    return KD;
  }
  
  inline int getFlowCalibration(){
    return 50000 / flow_calibration;
  }

  inline boolean getSwitch(){
    return digitalRead(IMPLEMENT_SWITCH);
  }
  
  inline int getDose(){
    return dose;
  }
  
  inline int getActualDose(){
#ifdef GPS
    return actual_flow / (width * gps->getSpeedMs());
#else
    if (tractor->getSpeedMs() != 0){
  	  return (needed_flow - delta_avg) / (width * tractor->getSpeedMs());
    }
    else {
      return 0;
    }
#endif
  }

  inline float getActualFlow(){
    return actual_flow;
  }
  
  inline byte getSetpoint(){
    return setpoint_pwm;
  }
  
  inline int getFlowPulses(){
    return flow_pulses;
  }
  
  inline int getGearPulses(){
    return gear_pulses;
  }
  
  inline boolean getAlarm(){
    return alarm;
  }
  
  inline boolean getFlag(){
    return update_flag;
  }
  
  inline unsigned long getVolume(){
    return (volume / 1000) * flow_calibration; // centiliters (liters is / 100000; ml of cc is / 100)
  }

  // -------
  // Setters
  // -------  
  inline void setTeeth(byte _i){
    teeth = _i;
  }
  
  inline void setPumps(byte _i){
    pumps = _i;
  }
  
  inline void setWidth(byte _i){
    width = _i;
  }
  
  inline void setKP(byte _i){
    KP  = _i;
  }

  inline void setKI(byte _i){
    KI  = _i;
  }

  inline void setKD(byte _i){
    KD  = _i;
  }
  
  inline void setFlowCalibration(int _impulses){
    flow_calibration = 50000 / _impulses;
  }

};
#endif
