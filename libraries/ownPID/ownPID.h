/*
PID-Berechnung (X- und Y-Achse)
Erstellt von Fabrizio Marti 21.10.2020
*/
 
#ifndef ownPID_h
#define ownPID_h

#include "Arduino.h"

class ownPID {
  public:
    ownPID( float Kp, float Ki, float Kd, float SetPoint);
    float e {};                                  //X-Achsen-Fehler
    float e_old {};
    float delta_e {};

    float Output {};                            //totaler Ausgang                                                           
    float u_P {};                               //proportionaler Ausgang
    float u_I {};                               //vergangener Ausgang
    float u_D {};                               //zukünftiger Ausgang
    float u_int {};

    float delta_t {};
    float t {};
    float t_old {};
	
    float Output_fu (float kalAngle);    
    
        
  private:
    float _Kp {};                                  //P-Verstärkung
    float _Ki {};                                  //I-Verstärkung
    float _Kd {};                                  //D-Verstärkung
    int _SetPoint {};                              //Soll-Wert
};  

#endif
