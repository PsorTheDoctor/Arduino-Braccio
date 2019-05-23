#include <stdlib.h>
#include <Servo.h>
#include <Braccio.h>

Servo base, shoulder, elbow, wrist_ver, wrist_rot, gripper;
const int STEP_DELAY = 30, CLOSED_GRIPPER = 73, OPEN_GRIPPER = 10;

int random_servo, random_move, range = 15, reset_count = 0,
    m1 = 90, 
    m2 = 140, 
    m3 = 45, 
    m4 = 10, 
    m5 = 90, 
    m6 = CLOSED_GRIPPER;

void setup() {    
    Braccio.begin();
    Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
}

void loop() {
    if (reset_count < 10) {
      
        random_servo = rand() % 11;
        random_move = rand() % range;
    
        switch (random_servo) {
        
          case 1:
             m1 += random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
          case 2:
             m1 -= random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
             
          case 3:
             m2 += random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
          case 4:
             m2 -= random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
    
          case 5:
             m3 += random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
          case 6:
             m3 -= random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
             
          case 7:
             m4 += random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
          case 8:
             m4 -= random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
    
          case 9:
             m5 += random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
          case 10:
             m5 -= random_move;
             Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
             break;
    
          default:
              break;
        }

        if (rand() % 10 == 0) {
            m6 = OPEN_GRIPPER;
        } else if(rand() % 10 == 1){
            m6 = CLOSED_GRIPPER;
        }
    } else {
        // do the reset by setting initial position
        m1 = 90, 
        m2 = 140, 
        m3 = 45, 
        m4 = 10, 
        m5 = 90, 
        m6 = CLOSED_GRIPPER;
        
        Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
        delay(1000);
        reset_count = 0;
    }
    reset_count++;
}
