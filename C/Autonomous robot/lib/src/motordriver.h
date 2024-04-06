#ifndef MOTORDRIVER_H
#define MOTORDRIVER_H

#include <Arduino.h>

class motordriver {
public:
    const int MAX_PWM = 255;

    // Pins for front-left motor
    const int FL = 15;
    const int FLDir = 13;

    // Pins for front-right motor
    const int FR = 14;
    const int FRDir = 12;

    // Pins for back-left motor
    const int BL = 18;
    const int BLDir = 20;

    // Pins for back-right motor
    const int BR = 19;
    const int BRDir = 21;

    // Constructor
    motordriver();

    // Destructor
    ~motordriver();

    // Function declarations
    void M1(int pwm);
    void M2(int pwm);
    void M3(int pwm);
    void M4(int pwm);
    void yMovement(int pwm);
    void xMovement(int pwm);
    void rotationalMovement(int pwm);
    void moveForward(int pwm);
    void moveBackward(int pwm);
    void moveRight(int pwm);
    void moveLeft(int pwm);
    void move45(int pwm);
    void move135(int pwm);
    void move315(int pwm);
    void move225(int pwm);
    void stopMotors();
};

#endif
