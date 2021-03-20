#ifndef declare_h
#define declare_h

  #include <Servo.h> 

    
    int debug;             // debug for user

    Servo servo_b;
    Servo servo_sL;
    Servo servo_sR;
    Servo servo_w;
    Servo servo_g;
    
    int sensorPin0 = A0;    // Base
    int sensorPin1 = A1;    // Shoulder
    int sensorPin2 = A2;    // Gripper
    int sensorPin3 = A3;    // Wirst 
    
    #define bJ1 6             // Joystick button 1
    #define bJ2 7             // Joystick button 2

    #define base 12
    #define shoulder_Left 11
    #define shoulder_Right 10
    #define wirst 9
    #define gripper 8
    
    int arrayStep, arrayMax, stepsMax, steps, Taster, time = 1000, del = 1000, T;
  /*    
    arraystep = memory what pos in the array
    arrayMax = max steps we safed to array
    stepsMax = longest way a servo have to travel
    steps = single steps for a move between stored positions
    Taster: use to process button handle
    T = time to repeat between cycles
  */
  
    long previousMillis1 = 0;
    long previousMillis2 = 0;
    long previousMillis3 = 0;
    long previousMicros = 0;
    
    unsigned long currentMillis;
    unsigned long currentMicros;
    
    int SensVal[4];                        // sensor value
    int jCenter;
    
    int dif[5], sol[5];                    // difference between stored position and momentary position
    float ist[5], dir[5];
    int top = 20;                          // we should not write over the end from a array
    
    int joint_b[21];                       // array for servo(s)
    int joint_sL[21];
    int joint_sR[21];
    int joint_w[21];
    int joint_g[21];
    
    
    // status 
    boolean playmode = false, Step = false, Default = true;

  #endif
