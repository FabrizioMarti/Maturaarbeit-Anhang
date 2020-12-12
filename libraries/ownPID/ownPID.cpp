/*
PID-Berechnung (X- und Y-Achse)
Erstellt von Fabrizio Marti 21.10.2020
*/

#include "Arduino.h"
#include "ownPID.h"

ownPID::ownPID (float Kp, float Ki, float Kd, float SetPoint){
  _Kp = Kp;                                 
  _Ki = Ki;                                  
  _Kd = Kd;  
  _SetPoint = SetPoint;                                
};

float ownPID::Output_fu(float kalAngle){
  t = millis();
  e = _SetPoint - kalAngle;  
  delta_t = t - t_old;
  delta_e = e - e_old;
  
  u_P = _Kp * e;                                          //proportionaler Fehler
  u_int = u_int + e * delta_t;
  u_I = _Ki * u_int;                                      //vergangener Fehler
  u_D = _Kd * delta_e / delta_t;                          //zukünftiger Fehler
  
  Output = u_P + u_I + u_D;

  e_old = e;
  t_old = t;

  return Output;                               //totaler Ausgang
};
