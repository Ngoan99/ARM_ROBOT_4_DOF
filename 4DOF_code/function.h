#ifndef function_h
#define function_h

  #include "declare.h"
  #include "eepRom.h"

//-------------------------------------------------------------------
// Declare prototype
    void center(void);
    void readPot(void);
    void mapping(void);
    void record(void);
    void Read(void);
    void move_servo(void);
    void calculate(void);
    void calc_pause(int T);
    void intro(void);
    void instance(void);
    void data_out(void);
    void interface(void);
    void Button(void);
    void manual(void);
    void play_servo();
    void Boot(void);
//--------------------------------------------------------------------

//  Read analog inputs to get mid value
    void center(){
       SensVal[0] = analogRead(sensorPin0); // Base
       SensVal[1] = analogRead(sensorPin1); // Shoulder
       SensVal[2] = analogRead(sensorPin3); // Wirst
       SensVal[3] = analogRead(sensorPin2); // Gripper
    
       int SensVal_max = SensVal[0], SensVal_min = SensVal[0];
    
       for(int i =0; i<4; i++){
         SensVal_max = max(SensVal[i],SensVal_max);
         SensVal_min = min(SensVal[i],SensVal_min);
        }
        
        jCenter = (SensVal_max + SensVal_min)/2;
    }
    
    
    // read analog inputs and add some offsets (mechanical corrections)
    void readPot(){  
       SensVal[0] = analogRead(sensorPin0); // Base
       SensVal[1] = analogRead(sensorPin1); // Shoulder
       SensVal[2] = analogRead(sensorPin3); // Wirst 
       SensVal[3] = analogRead(sensorPin2); // Gripper 
    
      if(Default == true){
        
       ist[0] = 1437; // Base
       ist[1] = 1520; // Schoulder Left
       ist[4] = 780;  // Schoulder Right
       ist[2] = 1950; // Wirst
       ist[3] = 1450;// Gripper
       
       Default= false;
      }
    }
    
    
    // we need microsecond for the servos instead potentiometer values
    void mapping(){
      static int R =500,_del= 10;
      static float r1x, r1y, r2x, r2y, R1i, R2i;
      r1x = SensVal[0] - jCenter;
      r1y = SensVal[1] - jCenter;
      
      r2x = SensVal[2] - jCenter;
      r2y = SensVal[3] - jCenter;
    
      R1i =sqrt(r1x*r1x + r1y*r1y);
      R2i =sqrt(r2x*r2x + r2y*r2y);  
      
      // Mapping Joystick 1
      if(R1i > R){
        if(SensVal[0] > 770) ist[0] += _del;
          else if(SensVal[0] < 270) ist[0] -= _del;
          
        if(SensVal[1] > 770) {
          ist[1] += _del;
          ist[4] -= _del;
        }
          else if(SensVal[1] < 270){
            ist[1] -= _del;
            ist[4] += _del;
          }
      }
    
      // Mapping Joystick2
        if(R2i > R){
          if(SensVal[2] > 770) ist[2] += _del;
            else if(SensVal[2] < 270)ist[2] -= _del;
          if(SensVal[3] > 770) ist[3] -= _del;
            else if(SensVal[3] < 270) ist[3] += _del;
      }
    
      ist[0]=constrain(ist[0], 544,2500);
      ist[1]=constrain(ist[1], 600,1700);     //s_R
      ist[4]=constrain(ist[4], 600,1700);     //s_L
      ist[2]=constrain(ist[2], 1550,2150);
      ist[3]=constrain(ist[3], 1200,1750);
    
    }
    
    
    // write positions in servo array
    void record(){
        joint_b[arrayStep] = ist[0]; 
        joint_sL[arrayStep] = ist[1];
        joint_sR[arrayStep] = ist[4];
        joint_w[arrayStep] = ist[2];
        joint_g[arrayStep] = ist[3];
    }
    
    
    // read from the array
    void Read(){
        sol[0] = joint_b[arrayStep]; 
        sol[1] = joint_sL[arrayStep];
        sol[4] = joint_sR[arrayStep];
        sol[2] = joint_w[arrayStep];
        sol[3] = joint_g[arrayStep];
    }
    
    
    // send milissecond values to servos
    void move_servo(){       
      servo_b.writeMicroseconds(ist[0]); 
      servo_sL.writeMicroseconds(ist[1]); 
      servo_sR.writeMicroseconds(ist[4]);
      servo_w.writeMicroseconds(ist[2]); 
      servo_g.writeMicroseconds(ist[3]); 
    }
    
    
    // single steps calculating 
    void calculate(){
      // travel distance for each servo
      dif[0] = abs(ist[0]-sol[0]);
      dif[1] = abs(ist[1]-sol[1]);
      dif[2] = abs(ist[2]-sol[2]);
      dif[3] = abs(ist[3]-sol[3]);
      
      // biggest travel way from all 4 servos
      stepsMax = 0;
      for(int i =0; i <4; i++){
        stepsMax = max(stepsMax,dif[i]);
      }
      // stepsMax is the biggest distance a servo have to do beween momentary position and new pos read from the array
      if (stepsMax < 500) // del(ay) between a single step is bigger is move is smaler. just looks cool
        del = 2000;
      else
        del = 1000;
      
       // calculating single (micro) step for each servo
       // need that to do move all servos in a loop (stepsMax times done) with different values.
       // This makes all servos have done the traveling distance at same time
      if (sol[0] < ist[0]) dir[0] = 0-(float)dif[0]/stepsMax; else dir[0] = (float)dif[0]/stepsMax;
      if (sol[1] < ist[1]){
        dir[1] = 0-(float)dif[1]/stepsMax; 
        dir[4] = (float)dif[1]/stepsMax; 
        }
        else {
          dir[1] = (float)dif[1]/stepsMax;
          dir[4] = 0-(float)dif[1]/stepsMax; 
        }
      if (sol[2] < ist[2]) dir[2] = 0-(float)dif[2]/stepsMax; else dir[2] = (float)dif[2]/stepsMax;
      if (sol[3] < ist[3]) dir[3] = 0-(float)dif[3]/stepsMax; else dir[3] = (float)dif[3]/stepsMax;
    
      Serial.println(dir[0]);
      Serial.println(dir[1]);
      Serial.println(dir[2]);
      Serial.println(dir[3]);
      Serial.println("-----------------");
    }
    
    
    // read pot and map to usable delay time after a complete move is done
    void calc_pause(int T){     
      int countverz =0;
      while(countverz < T/1000) // verz = time getting from calc_pause();
      { // here we do loop and wait until next start over
        countverz += 1;
        digitalWrite(13, HIGH); delay(25);   
        digitalWrite(13, LOW); delay(975); 
      }
    }
    
    
    //----------------------------------------------------------------------------------------
    void intro(){
      Serial.println("-------------------------------------------------------------------");             
      Serial.println("|                                                                  |");  
      Serial.println("|                      WELLCOME ROBOTIC                            |");
      Serial.println("|             Mini robot ready for use control                     |");
      Serial.println("|                                                                  |");
      Serial.println("| 1 : introduction                                                 |");
      Serial.println("| 2 : view data in EEPROM                                          |");
      Serial.println("| 3 : view current position                                        |");
      Serial.println("|                                                                  |"); 
      Serial.println("| Guide User:                                                      |");
      Serial.println("| - Click Joystick left to record current position                 |");
      Serial.println("| - Double click Joystick left to change Auto mode                 |");
      Serial.println("| - Click Joystick right to save data into EEPROM                  |");
      Serial.println("-------------------------------------------------------------------"); 
      }
    
    
    void instance(){
      Serial.print("Joint_b: ");
      Serial.println(ist[0]);
      Serial.print("Joint_sR: ");
      Serial.println(ist[1]);
      Serial.print("Joint_sL: ");
      Serial.println(ist[4]);
      Serial.print("Joint_w: ");
      Serial.println(ist[2]);
      Serial.print("Joint_g: ");
      Serial.println(ist[3]);
      Serial.println("--------------------------\n\n");
    }
    
    
    // generiert listen mit Servo daten (milli sekunden)
    void data_out(){
      int i = 0;
      Serial.print("Joint_b: ");
      while(i < arrayMax)
      {
        digitalWrite(13, HIGH);
        i += 1;
        Serial.print(joint_b[i]); Serial.print(", ");
      }
      Serial.println("");
      
      i = 0;
      Serial.print("Joint_s: ");
      while(i < arrayMax)
      {
        digitalWrite(13, HIGH);
        i += 1;
        Serial.print(joint_sL[i]); Serial.print(", ");
      }
      Serial.println("");
      
      i = 0;
      Serial.print("Joint_w: ");
      while(i < arrayMax)
      {
        digitalWrite(13, HIGH);
        i += 1;
        Serial.print(joint_w[i]); Serial.print(", ");
      }
      Serial.println("");
      
      i = 0;
      Serial.print("Joint_g: ");
      while(i < arrayMax)
      {
        digitalWrite(13, HIGH);
        i += 1;
        Serial.print(joint_g[i]); Serial.print(", ");
      }
      Serial.println("");
    }
    
    
    void interface(){
      if(Serial.available()){
        String input = Serial.readStringUntil('\n');
        int data = input.toInt();
        debug = data;
        Serial.print("debug: ");
        Serial.println(debug);
      }
    
      if(debug== 1){
        intro();
        debug=0;
      }
    
      if(debug== 2){
        Serial.println("Location data saved in EEPROM: ");
        data_out();
        debug= 0;
      }
    
       if(debug== 3){
        if(currentMillis - previousMillis3 > 250){
          Serial.println("Curent position: ");
          instance();
          previousMillis3 = currentMillis;
          }
       }
    }
    
    
    //---------------------------------------------------------------------------------------
    // check buttons for single and doubleclick
    void Button(){
      if(digitalRead(bJ1) == false)
        {
          delay(20);
          if (digitalRead(bJ1) == true) // taster losgelassen
            {
                if (Taster == 0)
                {
                  Taster = 1;
                  previousMillis2 = currentMillis;
                  Serial.println("Status Record ");
                }
                else if ((Taster == 1) && (currentMillis - previousMillis2 < 500))
                  Taster = 2; 
             }
        }
        
      if ((Taster == 1) && (currentMillis - previousMillis2 > 500)) // write to array
         {
              arrayStep += 1;
              arrayMax = arrayStep;
              record();
              Taster = 0;
              playmode = false;
              Serial.print("Record Step: "); Serial.println(arrayStep);
              digitalWrite(13, HIGH);
              delay(100);
              digitalWrite(13, LOW);
        }
      else if (Taster == 2)
       Boot();   
    
      if (currentMillis - previousMillis2 > 2000) // button Status clear
          Taster = 0;

      // write into EEPROM
      if (digitalRead(bJ2) == false){
        delay(20);
        if (digitalRead(bJ1) == true){
          
          write_EEP();  
      
          Serial.println("Data recorded in EEPROM");
          data_out();
          digitalWrite(13, HIGH); delay(1000);   
          digitalWrite(13, LOW);
      }
     }  
    }
    
    
    void manual(){
      readPot();                                    // get the value from potentiometers
      mapping();                                    // map to milliseconds for servos
      move_servo();                                 // sets newservo position
    }
    
    
    
    void play_servo(){
      steps += 1;
      // sure we not reach the end from a move
      if (steps < stepsMax){
        if(steps == 20) time = del*4;                   // ramp up 
        else if(steps == 40) time = del*3;              // time is the delay in microsecns we wait in the mainloop until
        else if(steps == 80) time = del*2;              // a micro step will be done
        else if(steps == 100) time = del-1;             // cannot explain here is not del*1
        
        if(steps == stepsMax-200) time = del*2;         // stop ramp down (200 microsteps before end time will be increased
        else if(steps == stepsMax-80) time = del*3;
        else if(steps == stepsMax-40) time = del*4;
        else if(steps == stepsMax-20) time = del*5;
        
        ist[0] += dir[0]; // set new pos
        ist[1] += dir[1];
        ist[4] += dir[4];
        ist[2] += dir[2];
        ist[3] += dir[3];
    
    
        servo_b.writeMicroseconds(ist[0]); 
        servo_sL.writeMicroseconds(ist[1]); 
        servo_sR.writeMicroseconds(ist[4]);
        servo_w.writeMicroseconds(ist[2]); 
        servo_g.writeMicroseconds(ist[3]); 
      }
      else
      {
        Step = 1; // next step aus array lesen
        steps = 0; // servo zwischenschritte
      }
    }
    
    
    void Boot(){
      Taster= 0;
      arrayStep= 0;
      Step= 1;
      playmode= true;
      Serial.println("playmode ");
      data_out();
      digitalWrite(13, HIGH);  
      delay(500);   
      digitalWrite(13, LOW);   
    }


#endif
