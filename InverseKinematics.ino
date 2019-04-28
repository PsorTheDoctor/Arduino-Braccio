/*
 * The program uses trial & error method to solve the problem of inverse kinematics.
 * It computes forward kinematics of a robot for random positition in each iteration of loop,
 * compares result and choose the value with the smallest distance from desired setup.
 * I know there are better ways to do it ...
 */

#include <stdlib.h>
#include <math.h>
#include <Servo.h>
#include <Braccio.h>

/////////////////////// initialize trial & error method ///////////////////////

int epoch = 0, num_of_epochs = 10000;
float error, smallest_error = 1000;
int best_setup [5];

void print_best_setup() {
    for (int i = 0; i < 5; i++) {
        Serial.print(best_setup[i]);
        Serial.print(" ");
    }
    Serial.println();
    Serial.println();
}

/////////////////////// initialize Braccio arm ///////////////////////

Servo base, shoulder, elbow, wrist_ver, wrist_rot, gripper;
const int STEP_DELAY = 0, CLOSED_GRIPPER = 73;
const int BASE_HEIGHT = 70, SEG1 = 120, SEG2 = 120, SEG3 = 60;
int m1, m2, m3, m4, m5, m6;

float desired_position_x = 0;
float desired_position_y = 370; // BASE_HEIGHT + SEG1 + SEG2 + SEG3;
float desired_position_z = 0;

void set_random_position() {
    m1 = rand() % 181;
    m2 = rand() % 151 + 15;
    m3 = rand() % 181;
    m4 = rand() % 181;
    m5 = rand() % 181;

    // Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
}

/////////////////////// initialize forward kinematics ///////////////////////

float result_mat[4][4];
float current_mat[4][4];

void multiply_matrices(float mat1[4][4], float mat2[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result_mat[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result_mat[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void copy_values_from_mat1_to_mat2(float mat1[4][4], float mat2[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mat2[i][j] = mat1[i][j];
        }
    }
}

void compute_position(int m1, int m2, int m3, int m4, int m5) {
  
    float mat_A01[4][4] = {
        { cos(90 - m1),  0, sin(90 - m1), 0           },
        { 0,             1, 0,            BASE_HEIGHT },
        { -sin(90 - m1), 0, cos(90 - m1), 0           },
        { 0,             0, 0,            1           }
    };
    
    float mat_A12[4][4] = {
        { cos(90 - m2), -sin(90 - m2), 0, 0    },
        { sin(90 - m2), cos(90 - m2),  0, SEG1 },
        { 0,            0,             1, 0    },
        { 0,            0,             0, 1    }
    };
    
    float mat_A23[4][4] = {
        { cos(90 - m3), -sin(90 - m3), 0, 0    },
        { sin(90 - m3), cos(90 - m3),  0, SEG2 },
        { 0,            0,             1, 0    },
        { 0,            0,             0, 1    }
    };
    
    float mat_A34[4][4] = {
        { cos(90 - m4), -sin(90 - m4), 0, 0    },
        { sin(90 - m4), cos(90 - m4),  0, SEG3 },
        { 0,            0,             1, 0    },
        { 0,            0,             0, 1    }
    };
    
    float mat_A45[4][4] = {
        { cos(90 - m1),  0, sin(90 - m1), 0 },
        { 0,             1, 0,            0 },
        { -sin(90 - m1), 0, cos(90 - m1), 0 },
        { 0,             0, 0,            1 }
    };
  
    multiply_matrices (mat_A01, mat_A12);
    copy_values_from_mat1_to_mat2 (result_mat, current_mat);
    
    multiply_matrices (current_mat, mat_A23);
    copy_values_from_mat1_to_mat2 (result_mat, current_mat);
    
    multiply_matrices (current_mat, mat_A34);
    copy_values_from_mat1_to_mat2 (result_mat, current_mat);
    
    multiply_matrices (current_mat, mat_A45);
}

//void print_result_mat(float mat[4][4]) {
//    for (int i = 0; i < 4; i++) {
//        for (int j = 0; j < 4; j++) {
//            Serial.print(mat[i][j]);
//            Serial.print("   ");
//        }
//        Serial.println();
//    }
//}

/////////////////////// main program ///////////////////////

void setup() {
    Serial.begin(9600);
    Braccio.begin();
}

void loop() {
    if (epoch < num_of_epochs) {
      
        set_random_position();
        compute_position(m1, m2, m3, m4, m5);
        
        // compute error
        error = desired_position_x - result_mat[0][3] 
              + desired_position_y - result_mat[1][3] 
              + desired_position_z - result_mat[2][3];

        // save best result
        if (abs(error) < smallest_error) {
            smallest_error = error;
            
            best_setup[0] = m1;
            best_setup[1] = m2;
            best_setup[2] = m3;
            best_setup[3] = m4;
            best_setup[4] = m5;

            Serial.println(error);
            print_best_setup();
        }
        epoch++;
    }
}
