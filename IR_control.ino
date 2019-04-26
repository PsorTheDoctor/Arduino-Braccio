#include <IRremote.h>
#include <Braccio.h>
#include <Servo.h>

// constant infrared values from Phillips remote measured at 1 a.m. daylight
const long LEFT = 4139331627,     LEFT1 = 1694444169;
const long RIGHT = 4122554008,    RIGHT1 = 1677666550;
const long UP = 4259991806,       UP1 = 3252342964;
const long DOWN = 1740676561,     DOWN1 = 1039969935;
const long INFO = 1734096226,     INFO1 = 353497852;
const long OPTIONS = 963231140,   OPTIONS1 = 215094250;
const long BACK = 2486907987,     BACK1 = 436260253;
const long FORMAT = 3721635242,   FORMAT1 = 4285659560;
const long VOL_PLUS = 2734467172, VOL_PLUS1 = 411014570;
const long VOL_MIN = 2781446235,  VOL_MIN1 = 1943003649;
const long CH_PLUS = 2971556410,  CH_PLUS1 = 226393748;
const long CH_MIN = 3794690225,   CH_MIN1 = 1727524459;

/* Assigment of functions to buttons on Phillips remote
    base      -> menu (left, right)
    shoulder  -> menu (up, down)
    elbow     -> info, back
    wrist_ver -> options, format
    wrist_rot -> volume (+, -)
    gripper   -> channel (+, -)
*/

const int RECV_PIN = 7;
IRrecv irrecv(RECV_PIN);
decode_results results;

// initialize robot arm
Servo base, shoulder, elbow, wrist_ver, wrist_rot, gripper;
const int STEP_DELAY = 0, OPEN_GRIPPER = 10, CLOSED_GRIPPER = 73, MOVE = 20;
int m1 = 90, m2 = 90, m3 = 90, m4 = 90, m5 = 90, m6 = CLOSED_GRIPPER;

void setup() {
    Serial.begin(9600);
    Braccio.begin();
    irrecv.enableIRIn();
    irrecv.blink13(true);
}

void loop() {
    Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);

    if (irrecv.decode(&results)) {
        Serial.println(results.value);
        switch (results.value) {

        case LEFT: LEFT1:
            m1 -= MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        case RIGHT: RIGHT1:
            m1 += MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;

        case DOWN: DOWN1:
            m2 -= MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        case UP: UP1:
            m2 += MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;

        case BACK: BACK1:
            m3 -= MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        case INFO: INFO1:
            m3 += MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;

        case FORMAT: FORMAT1:
            m4 -= MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        case OPTIONS: OPTIONS1:
            m4 += MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;

        case VOL_MIN: VOL_MIN1:
            m5 -= MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        case VOL_PLUS: VOL_PLUS1:
            m5 += MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;

        case CH_MIN: CH_MIN1:
            m6 -= MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        case CH_PLUS: CH_PLUS1:
            m6 += MOVE;
            Braccio.ServoMovement(STEP_DELAY, m1, m2, m3, m4, m5, m6);
            break;
        default:
            break;
        }
        irrecv.resume();
    }
}
