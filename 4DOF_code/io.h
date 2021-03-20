#ifndef io_h
#define io_h

  #include "declare.h"

    void configPin(){
        pinMode(bJ1, INPUT_PULLUP);         
        pinMode(bJ2, INPUT_PULLUP);
        pinMode(13, OUTPUT);                // sets the digital pin 13 as outtput
      }
     
     
//  Attaches pin to servo
    void configServo(){
        servo_b.attach(base);                 
        servo_sL.attach(shoulder_Left);  
        servo_sR.attach(shoulder_Right);
        servo_w.attach(wirst);
        servo_g.attach(gripper);
      }


#endif
