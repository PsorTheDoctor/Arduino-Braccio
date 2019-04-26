#include <math.h>
#include <Servo.h>
#include <Braccio.h>

Servo base, shoulder, elbow, wrist_ver, wrist_rot, gripper;
const int CLOSED_GRIPPER = 73;
const int BASE_HEIGHT = 70, SEG1 = 120, SEG2 = 120, SEG3 = 60;
int m1 = 90, m2 = 90, m3 = 90, m4 = 90, m5 = 90;

/*
  The forward kinematics uses following assumptions:
  axis X -> right
  axis Y -> top
  axis Z -> front
 */

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

void print_result_mat(float mat[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            Serial.print(mat[i][j]);
            Serial.print("   ");
        }
        Serial.println();
    }
}

void setup() {
    Serial.begin(9600);
    Braccio.begin();
//
//    for (int i = 1; i < 5; i++) {
//        multiply_matrices(current_mat, mat[i], result_mat);
//    }
//    Serial.print(result_mat);

    multiply_matrices (mat_A01, mat_A12);
    copy_values_from_mat1_to_mat2 (result_mat, current_mat);
    
    multiply_matrices (current_mat, mat_A23);
    copy_values_from_mat1_to_mat2 (result_mat, current_mat);
    
    multiply_matrices (current_mat, mat_A34);
    copy_values_from_mat1_to_mat2 (result_mat, current_mat);
    
    multiply_matrices (current_mat, mat_A45);

    // print_result_mat(result_mat);
    Serial.print("x = "); Serial.println(result_mat[0][3]);
    Serial.print("y = "); Serial.println(result_mat[1][3]);
    Serial.print("z = "); Serial.println(result_mat[2][3]);
}

void loop() {
  
}
