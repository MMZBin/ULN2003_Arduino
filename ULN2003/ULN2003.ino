#include "ULN2003.h"

uint32_t startTime = 0;

ULN2003 motor(2, 3, 4, 5);

void setup() {
    pinMode(8, INPUT_PULLUP);
    pinMode(9, INPUT_PULLUP);

    //motor.setPhase(ULN2003::Phase::TWO);
    motor.setRPM(60);
    motor.setStartSpeed(10);
    motor.setAcceleration(1);
    //motor.setDelayCorrection(125);
}

void loop() {
    if (!digitalRead(8)) {
        if (motor.getState() == ULN2003::State::STOPPED) {
            startTime = millis();
        }
        motor.moveByRev(2);
    }
    if (!digitalRead(9)) {
        motor.stop();
    }

    if ((startTime != 0) && (motor.getState() == ULN2003::State::STOPPED)) {
        //Serial.println(millis() - startTime);
        startTime = 0;
    }

    motor.update();
}