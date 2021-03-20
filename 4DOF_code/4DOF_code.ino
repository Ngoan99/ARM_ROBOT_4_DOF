/*
  Design graphics and code by Thanh Ngoan, Can Tho university
  Plan ARM_ROBOT 4DOF 
  Last_version 28/1/2021 
 */
 
#include "declare.h"
#include "io.h"
#include "eepRom.h"
#include "function.h"

//---------------------------------------------------------------------------------------
void setup(){
  configPin();
  configServo();
  
  digitalWrite(13, HIGH);             // sets the LED on
  Serial.begin(115200);                 // baudrate have to be same on the IDE
  
  while(Serial.available()) {
    char ch = Serial.read();
  }
  
  digitalWrite(13, LOW);              // sets the LED off
  
  intro();

  center();                           // get the focus of joysticks
  read_EEP();                         // load coordinates in EEPROM memory
  
}

//---------------------------------------------------------------------------------------
void loop() {

  currentMillis = millis();                            // all is about timing
  currentMicros = micros();

  interface();

  Button();                                            // read the button  

  // Manualy mode
  if(!playmode){
    // Location update every 25 milliseconds
    if(currentMillis - previousMillis1 > 15){
      if(arrayStep < top){
        previousMillis1 = currentMillis; 
        manual(); 
      } 
      else if (arrayStep == top){      
        Serial.println("wanrning: Memory overflow"); 
        delay(200); 
        arrayStep++;    
      }
      else{
        Serial.println("Auto mode");
        arrayMax= arrayStep--;
        write_EEP();
        Boot();        
      }   
    }
  }

  // Auto play mode
  else if(playmode){
    // next step read from array
    if(Step){                                      
      digitalWrite(13, HIGH);
      // we not reach the end from stored data
      if (arrayStep < arrayMax){
        arrayStep += 1;                           // next array pos
        Read();                                   // from the arrays
        calculate();                              // find biggest travel distance and calculate the other 3 servos (the have to do smaler steps to be finished at same time!)
        Step = 0;
        digitalWrite(13, LOW);  
      }
      else                                        // array read finished > start over
      {
        arrayStep = 0; 
        calc_pause(1000);                         // delay between 2 cycles
      }
    }
    
    // do the servos!
    else
      // here we do a single micro step
      if (currentMicros - previousMicros > time){
        previousMicros = currentMicros;
        play_servo(); 
      }
  }
}
