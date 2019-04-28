/*
 * The program uses trial & error method to solve the problem of inverse kinematics.
 * It computes forward kinematics of a robot for random positition in each iteration of loop,
 * compares result and choose the value with the smallest distance from desired setup.
 *
 * It runs below algorithm for every single possible setup and saves results to a file.
 */

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <fstream>

using namespace std;

/////////////////////// initialize trial & error method ///////////////////////

const int NUM_OF_EPOCHS = 300;
float error, smallest_error = 1000;
int best_setup [5];

float compute_error_by_pythagoras_formula(float desired_x, float x,
                                         float desired_y, float y,
                                         float desired_z, float z) {
     error = sqrt(pow(desired_x - x, 2)
                + pow(desired_y - y, 2)
                + pow(desired_z - z, 2));
    return error;
}

float keep_best_result(int m1, int m2, int m3, int m4, int m5) {
    if (abs(error) < smallest_error) {
        smallest_error = error;

        best_setup[0] = m1;
        best_setup[1] = m2;
        best_setup[2] = m3;
        best_setup[3] = m4;
        best_setup[4] = m5;
    }
    return smallest_error;
}

/////////////////////// initialize Braccio arm model ///////////////////////

const int BASE_HEIGHT = 70, SEG1 = 120, SEG2 = 120, SEG3 = 60;
int m1, m2, m3, m4, m5, m6;

float desired_position_x = 0;
float desired_position_y = 370; // BASE_HEIGHT + SEG1 + SEG2 + SEG3;
float desired_position_z = 0;

const int RADIUS = SEG1 + SEG2 + SEG3;
// const int MAX_VALUE_X = RADIUS;
// const int MAX_VALUE_Y = BASE_HEIGHT + RADIUS;
// const int MAX_VALUE_Z = RADIUS;

bool is_point_belongs_to_range(int x, int y, int z, int radius) {
    float current_radius = sqrt(pow(x, 2) + pow(y - BASE_HEIGHT, 2) + pow(z, 2));

    if (current_radius <= radius) {
        return true;
    } else {
        return false;
    }
}

void set_random_position() {
    m1 = rand() % 181;
    m2 = rand() % 151 + 15;
    m3 = rand() % 181;
    m4 = rand() % 181;
    m5 = rand() % 181;
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

void compute_forward_kinematics(int m1, int m2, int m3, int m4, int m5) {

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

/////////////////////// main program ///////////////////////

int main() {
    fstream file;
    file.open("positions_dataset.txt", ios::out);

    for (int x = -RADIUS; x < RADIUS; x++) {
      for (int y = 0; y < BASE_HEIGHT + RADIUS; y++) {
        for (int z = -RADIUS; z < RADIUS; z++) {

            if (is_point_belongs_to_range(x, y, z, RADIUS)) {

              for (int epoch = 0; epoch < NUM_OF_EPOCHS; epoch++) {

                set_random_position();

                compute_forward_kinematics(m1, m2, m3, m4, m5);

                compute_error_by_pythagoras_formula(desired_position_x, result_mat[0][3],
                                                    desired_position_y, result_mat[1][3],
                                                    desired_position_z, result_mat[2][3]);

                keep_best_result(m1, m2, m3, m4, m5);
              }
              file << "x: " << x << " y: " << y << " z: " << z << endl;
              file << "error: " << error << endl;
              file << "m1: " << m1 << " m2: " << m2 << " m3: " << m3 << " m4: " << m4 << " m5: " << m5 << endl;
              file << endl;
            }
        }
      }
    }
    file.close();
    return 0;
}
