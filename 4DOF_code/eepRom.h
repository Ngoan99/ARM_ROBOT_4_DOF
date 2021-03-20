#ifndef eppRom_h
#define eppRom_h 

  #include <EEPROM.h>
  #include "declare.h"

    void r_EEPROM(int address, int numbers[], int arraySize){
      int addressIndex = address;
      for (int i = 0; i < arraySize; i++){
        numbers[i] = (EEPROM.read(addressIndex) << 8) + EEPROM.read(addressIndex + 1);
        addressIndex += 2;
      }
    }
    
    void w_EEPROM(int address, int numbers[], int arraySize){
      int addressIndex = address;
      for (int i = 0; i < arraySize; i++) {
        EEPROM.write(addressIndex, numbers[i] >> 8);
        EEPROM.write(addressIndex + 1, numbers[i] & 0xFF);
        addressIndex += 2;
      }
    }
  
    void read_EEP(){
      
      if(EEPROM.read(200)== 0xff){
        arrayMax = 0;
        return;
        }
      else
        arrayMax = EEPROM.read(200);
      
      r_EEPROM(0, joint_b, arrayMax+1);    
      r_EEPROM(40, joint_sL, arrayMax+1);   
      r_EEPROM(80, joint_sR, arrayMax+1);   
      r_EEPROM(120, joint_w, arrayMax+1);   
      r_EEPROM(160, joint_g, arrayMax+1);   
          
    }
    
    void write_EEP(){
    
      EEPROM.write(200,arrayMax);
      w_EEPROM(0, joint_b, arrayMax+1);     
      w_EEPROM(40, joint_sL, arrayMax+1);   
      w_EEPROM(80, joint_sR, arrayMax+1);   
      w_EEPROM(120, joint_w, arrayMax+1);   
      w_EEPROM(160, joint_g, arrayMax+1);   
    }
  
#endif  
