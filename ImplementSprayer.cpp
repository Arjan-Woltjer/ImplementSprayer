/*
  ImplementSprayer - a libary for a Sprayer
 Copyright (C) 2011-2014 J.A. Woltjer.
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

#include "ImplementSprayer.h"

//------------
// Constructor
//------------
#ifdef GPS
ImplementSprayer::ImplementSprayer(VehicleGps * _gps){
  // TODO
}

#else
ImplementSprayer::ImplementSprayer(VehicleTractor * _tractor){
  Serial.println(S_DIVIDE);
  Serial.println("Initialising sprayer implement");
  Serial.println(S_DIVIDE);
  
  // Pin configuration  
  // Outputs
  pinMode(OUTPUT, OUTPUT);
  pinMode(ALARM, OUTPUT);
  
  digitalWrite(OUTPUT, LOW);
  digitalWrite(OUTPUT, LOW);

  // Inputs
  pinMode(GEAR_SENS_PIN, INPUT);
  pinMode(FLOW_SENS_PIN, INPUT);
  pinMode(IMPLEMENT_SWITCH, INPUT);
  
  digitalWrite(GEAR_SENS_PIN, LOW);
  digitalWrite(FLOW_SENS_PIN, LOW);
  digitalWrite(IMPLEMENT_SWITCH, HIGH);
  
  gear_puls = false;
  flow_puls = false;
  alarm = false;

  // Get calibration data from EEPROM otherwise use defaults
  if (!readCalibrationData()){
    // Default pwm calibration set
    pwm_calibration_data[0] = 1;  // 100 - 131
    pwm_calibration_data[1] = 47;
    pwm_calibration_data[2] = 65;
    pwm_calibration_data[3] = 79;
    pwm_calibration_data[4] = 90;
    pwm_calibration_data[5] = 129;
    pwm_calibration_data[6] = 150;
    pwm_calibration_data[7] = 170;
    pwm_calibration_data[8] = 187;
    pwm_calibration_data[9] = 192;
    pwm_calibration_data[10] = 196;
    pwm_calibration_data[11] = 203;
    pwm_calibration_data[12] = 220;
    pwm_calibration_data[13] = 251;
    pwm_calibration_data[14] = 255;
    pwm_calibration_data[15] = 255;

    // Default flow calibration set
    flow_calibration = 55;  // 140 - 141

    cc_per_omw = 11 * pumps; // 150 - 151, voor de hele pomp
    
    teeth = 20;              // 160
    pumps = 8;               // 161
    width = 30;              // 162, in decimeters
  
    KP = 100;                // 170
    KI = 18;                 // 171
    KD = 20;                 // 172
  
#ifdef DEBUG
    Serial.println("No calibration data found");
    Serial.println("Using defaults");
#endif
  }
  readDose();                // 180

  // Calibration points for pwm and flow
  pwm_calibration_points[0] = 45;
  pwm_calibration_points[1] = 50;
  pwm_calibration_points[2] = 55;
  pwm_calibration_points[3] = 60;
  pwm_calibration_points[4] = 65;
  pwm_calibration_points[5] = 70;
  pwm_calibration_points[6] = 75;
  pwm_calibration_points[7] = 80;
  pwm_calibration_points[8] = 85;
  pwm_calibration_points[9] = 90;
  pwm_calibration_points[10] = 95;
  pwm_calibration_points[11] = 100;
  pwm_calibration_points[12] = 105;
  pwm_calibration_points[13] = 110;
  pwm_calibration_points[14] = 115;
  pwm_calibration_points[15] = 120;
  
  // Reset timer and pulse counters and setpoints
  update_age = millis();
  gear_pulses = 0;
  rounds = 0;
  flow_pulses = 0;
  volume = 0;
  setpoint_pwm = 0;
  
  // PID
  for (int i = 0; i < 20; i++) delta_hist[i] = 0;
  delta_sum = 0;
  delta_avg = 0;
  delta_delta = 0;
  
  P = 0;
  I = 0;
  D = 0;
  
  update_age_flag = millis();
  update_flag = false;
    
  hist_count = 0;
  hist_time = 5;
  
  // Print calibration data
  printCalibrationData();

  tractor = _tractor;
}
#endif

// ----------------------------------
// Method for updating implement data
// ----------------------------------
void ImplementSprayer::update(byte _mode, int _buttons){
  if (gear_puls != digitalRead(GEAR_SENS_PIN)){
    gear_pulses++;
    gear_puls = !gear_puls;
  }
  
  if (flow_puls != digitalRead(FLOW_SENS_PIN)){
    flow_pulses++;
    flow_puls = !flow_puls;
  }
   
  if (millis() - update_age_flag >= 2000){
    update_flag = !update_flag;
    update_age_flag = millis();
  }
  
  // update offset, angle, steer and setpoint every second, reset counters
  if (millis() - update_age >= 1000){
    setDose(_buttons);
	
    if(_mode == 0 || _mode == 4){
      calculateActualFlow();
      calculateNeededFlow();
      calculateCalculatedFlow();
      calculateSetpointFlow(_mode);
      calculateAlarm(_mode);
      calculateSetpointPwm();
      
      analogWrite(OUTPUT, setpoint_pwm);
      digitalWrite(ALARM, alarm);
      temp = gear_pulses;
      volume += flow_pulses;
    }
    else if (_mode == 2){
      //I = 0;
      digitalWrite(OUTPUT, LOW);
      digitalWrite(ALARM, LOW);
    }

    // Reset timer and pulse counters
    update_age = millis();
    gear_pulses = 0;
    flow_pulses = 0;
  }
}

// ----------------------------------
// Method for calculating actual flow
// ----------------------------------
void ImplementSprayer::calculateActualFlow(){
  // In cc per 100 seconds
  actual_flow = flow_calibration * flow_pulses; //In cc per 100 seconden
}


// ----------------------------------
// Method for calculating needed flow
// ----------------------------------
void ImplementSprayer::calculateNeededFlow(){
  // In cc per 100 seconds
#ifdef GPS
  needed_flow = dose * width * gps->getSpeedMs(); //In cc per 100 seconden
#else
  needed_flow = dose * width * tractor->getSpeedMs(); //In cc per 100 seconden
#endif
}

// --------------------------------------
// Method for calculating calculated flow
// --------------------------------------
void ImplementSprayer::calculateCalculatedFlow(){  
  // In cc per 100 seconds
  calculated_flow = gear_pulses * cc_per_omw * 50.0f / teeth; // In cc per 100 seconden (100 * gear / teeth * 2)
}

//-------------------------
// Method for setpoint flow
//-------------------------
void ImplementSprayer::calculateSetpointFlow(byte _mode){
  byte _previous_hist_count = hist_count - 1;
  
  if (hist_count >= hist_time) {
    hist_count = 0;
    _previous_hist_count = hist_time - 1;
  }
  
  // Set delta sum, delta and average
  delta_flow = needed_flow - actual_flow;
  delta_sum = delta_sum - delta_hist[hist_count] + delta_flow;
  delta_avg = delta_sum / hist_time;
  delta_delta = delta_flow - delta_avg;
  
  // Update delta history
  delta_hist[hist_count] = delta_flow;
  
  P = float(needed_flow) * KP / 100.0f;
  
  // Only update I when mode equals automatic
  if (_mode == 0 || _mode == 4){
    I = I + (float(delta_avg) * KI / 100.0f);
    D = float(delta_flow) * KD / 100.0f;
  }
  
  if(I > 5000) I = 5000;
  else if(I < -5000) I = -5000;
  
  setpoint_flow =  P + I + D;
  hist_count ++;
}

void ImplementSprayer::calculateAlarm(byte _mode){
  if (_mode == 0 || _mode == 4){
    // delta flow is greater than 10 percent
    if (abs(long(needed_flow - calculated_flow) * 100 / needed_flow) > 25){
      alarm = true;
    }
    else {
      alarm = false;
    }
  }
}    

// ---------------------------------
// Method for calculating needed PWM
// ---------------------------------
void ImplementSprayer::calculateSetpointPwm(){
  int _read_raw = teeth * float(setpoint_flow) / (cc_per_omw * 20); // In pulsen per 2.5 seconden
  int i = 0;

  // Loop through calibrationdata
  while (_read_raw > pwm_calibration_data[i] && i < 15){
    i++;
  }

  if (i == 0){
    i++;
  }

  // Interpolate calibrationdata
  float a = _read_raw - pwm_calibration_data[i-1];
  float b = pwm_calibration_data[i] - pwm_calibration_data[i-1];
  float c = pwm_calibration_points[i] - pwm_calibration_points[i-1];
  float d = pwm_calibration_points[i-1];

  // Calculate actual implement offset
  setpoint_pwm = (((a * c) / b) + d);
}

// ------------------------------
// Method for adjusting implement
// ------------------------------
void ImplementSprayer::stop(){
  analogWrite(OUTPUT, 0);
  digitalWrite(ALARM, 0);
}

// ---------------------------------------------
// Method for setting dose and writing to EEPROM
// ----------------------------------------------
void ImplementSprayer::setDose(int _correction){
  if (_correction){
    dose += _correction;
    if (dose > 350 || dose < 50){
      dose = 100;
    }

    // Write each byte separately to the memory
    EEPROM.write(190, highByte(dose));
    EEPROM.write(191, lowByte(dose));
  }
}

// ---------------------------------------------
// Method for reading implement dose from EEPROM
// ---------------------------------------------
void ImplementSprayer::readDose(){
  if (EEPROM.read(190) < 255){
    // Read dose (2 bytes)
    dose = word(EEPROM.read(190), EEPROM.read(191));
    if (dose > 350 || dose < 50){
      dose = 300;
    }
  }
  else{
    dose = 100;
  }
}

// ----------------------------
// Methods for calibrating pump
// ----------------------------
int ImplementSprayer::calibratePump(){
  gear_pulses = 0;
  flow_pulses = 0;
  volume = 0;
  rounds = 0;

  for (int _i = 0; _i < 16; _i++){
    analogWrite(OUTPUT, pwm_calibration_points[_i]);
    
    update_age = millis();
    
    while(millis() - update_age <= 5000){
      if (millis() - update_age <= 2500) {
        gear_pulses = 0;
      }
      
      // Count flow pulses
      if (flow_puls != digitalRead(FLOW_SENS_PIN)){
        flow_pulses++;
        volume++;
        flow_puls = !flow_puls;
        digitalWrite(OUTPUT_LED, flow_puls); 
      }
      // Count gear pulses
      if (gear_puls != digitalRead(GEAR_SENS_PIN)){
        gear_pulses++;
        rounds++;
        gear_puls = !gear_puls;
      }
    }
    pwm_calibration_data[_i] = gear_pulses;
  }
  // Stop pump
  stop();
  
  // calculate cc_per_omw from: volume / rounds total
  cc_per_omw = float(volume * flow_calibration / 100) / float(rounds / (teeth * 2));
  
  return cc_per_omw;
}

// ----------------------------------------------
// Method for reading calibrationdata from EEPROM
// ----------------------------------------------
boolean ImplementSprayer::readCalibrationData(){
  // Read amount of startups and add 1
  EEPROM.write(0, EEPROM.read(0) + 1);
  
  // Read offset and angle calibration data
  if (EEPROM.read(100) != 255 || EEPROM.read(140) != 255 ||
    EEPROM.read(150) != 255 || EEPROM.read(160) != 255 ||
    EEPROM.read(161) != 255 || EEPROM.read(162) != 255 ||
    EEPROM.read(170) != 255 || EEPROM.read(171) != 255 ||
    EEPROM.read(172) != 255){

    // Read from eeprom highbyte, then lowbyte, and combine into words
    for(int i = 0; i < 32; i++){
      // 100 - 116
      pwm_calibration_data[i] = EEPROM.read(i+100);
    }
    
    flow_calibration = word(EEPROM.read(140), EEPROM.read(141));

    cc_per_omw = int(word(EEPROM.read(150), EEPROM.read(151))) / 100;

    teeth = EEPROM.read(160);
    pumps = EEPROM.read(161);
    width = EEPROM.read(162);
    
    KP = EEPROM.read(170);
    KI = EEPROM.read(171);
    KD = EEPROM.read(172);
  }
  else {
    return false;
  }
  return true;
}

//---------------------------------------------------
//Method for printing calibration data to serial port
//---------------------------------------------------
void ImplementSprayer::printCalibrationData(){
  // Printing calibration data to serial port
  Serial.println("-------------------------------");
  Serial.println("Implement using following data:");
  Serial.println("-------------------------------");
  Serial.println("Gear calibration data RPM->PWM");
  for (int i = 0; i < 16; i++){
    Serial.print(pwm_calibration_data[i]);
    Serial.print(", ");
    Serial.println(pwm_calibration_points[i]);
  }
  Serial.println("-------------------------------");

  Serial.println("Flow calibration");
  Serial.println(flow_calibration);
  Serial.println("-------------------------------");
  
  Serial.println("Teeth");
  Serial.println(teeth);
  Serial.println("-------------------------------");

  Serial.println("Pumps");
  Serial.println(pumps);
  Serial.println("-------------------------------");

  Serial.println("Width");
  Serial.println(width);
  Serial.println("-------------------------------");
  
  Serial.println("CC per round");
  Serial.println(cc_per_omw);
  Serial.println("-------------------------------");
  
  Serial.println("PID values KP/KI/KD");
  Serial.println(KP);
  Serial.println(KI);
  Serial.println(KD);
  Serial.println("-------------------------------");
}

// --------------------------------------------
// Method for writing calibrationdata to EEPROM
// --------------------------------------------
void ImplementSprayer::writeCalibrationData(){
  // Write each byte separately to the memory first the data then the points
  for(int i = 0; i < 32; i++){
    EEPROM.write(i + 100, pwm_calibration_data[i]);  // 100
  }
  
  EEPROM.write(140, highByte(flow_calibration));     // 140
  EEPROM.write(141, lowByte(flow_calibration));      // 141

  int _cc_per_omw = cc_per_omw * 100;
  EEPROM.write(150, highByte(_cc_per_omw));          // 150
  EEPROM.write(151, lowByte(_cc_per_omw));           // 151

  EEPROM.write(160, teeth);                          // 160
  EEPROM.write(161, pumps);                          // 161
  EEPROM.write(162, width);                          // 162
  
  EEPROM.write(170, KP);                             // 170
  EEPROM.write(171, KI);                             // 171
  EEPROM.write(172, KD);                             // 172

#ifdef DEBUG
  Serial.println("Calibration data written");
#endif  
}
