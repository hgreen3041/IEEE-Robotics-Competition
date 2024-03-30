#include <Arduino.h> 
#include <cmath>    


// Constructor
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
    motordriver() {


    //setup

    pinMode(FL, OUTPUT);
    pinMode(FLDir, OUTPUT);

    pinMode(FR, OUTPUT);
    pinMode(FRDir, OUTPUT);

    pinMode(BL, OUTPUT);
    pinMode(BLDir, OUTPUT);

    pinMode(BR, OUTPUT);
    pinMode(BRDir, OUTPUT);
    //end setup

    }

// Destructor
    ~motordriver() {
        // Clean up resources if needed
    }



    /*
    Handles all forward and backward movement.

    Args: 
    pwm(int)
     - Positive pwm for forward 
     - negative pwm for backward

    Returns: 
    None
    */

void yMovement(int pwm){
    if(pwm >= 0 ){
        analogWrite(FL, pwm);
        digitalWrite(FLDir, LOW);

        analogWrite(FR, pwm);
        digitalWrite(FRDir, LOW);

        analogWrite(BL, pwm);
        digitalWrite(BLDir, LOW);

        analogWrite(BR, pwm);
        digitalWrite(BRDir, LOW);
    }

    else{
        pwm = abs(pwm);
        analogWrite(FL, pwm);
        digitalWrite(FLDir, HIGH);

        analogWrite(FR, pwm);
        digitalWrite(FRDir, HIGH);

        analogWrite(BL, pwm);
        digitalWrite(BLDir, HIGH);

        analogWrite(BR, pwm);
        digitalWrite(BRDir, HIGH); 
    }
}

    /*
    Handles all x-movement (left and right).

    Args: 
    pwm(int)
        -   positive pwm for right. 
        -   negative pwm for left. 

    Returns: 
    None
    */

void xMovement(int pwm){
    if(pwm >= 0){
        analogWrite(FL, pwm);
        digitalWrite(FLDir, LOW);

        analogWrite(FR, pwm);
        digitalWrite(FRDir, HIGH);

        analogWrite(BL, pwm);
        digitalWrite(BLDir, HIGH);

        analogWrite(BR, pwm);
        digitalWrite(BRDir, LOW);  
    }

    else {
        pwm = abs(pwm);
        analogWrite(FL, pwm);
        digitalWrite(FLDir, HIGH);

        analogWrite(FR, pwm);
        digitalWrite(FRDir, LOW);

        analogWrite(BL, pwm);
        digitalWrite(BLDir, LOW);

        analogWrite(BR, pwm);
        digitalWrite(BRDir, HIGH);
    }
}

/*
Handles all rotational movement.
Args: 
    pwm(int)
        -   positive for counterclockwise. 
        -   negative for clockwise. 


Returns: 
None
*/

void rotationalMovement(int pwm){
    if(pwm >= 0){
        analogWrite(FL, pwm);
        digitalWrite(FLDir, HIGH);

        analogWrite(FR, pwm);
        digitalWrite(FRDir, LOW);

        analogWrite(BL, pwm);
        digitalWrite(BLDir, HIGH);

        analogWrite(BR, pwm);
        digitalWrite(BRDir, LOW);
    }

    else{
        pwm = abs(pwm);
        analogWrite(FL, pwm);
        digitalWrite(FLDir, LOW);

        analogWrite(FR, pwm);
        digitalWrite(FRDir, HIGH);

        analogWrite(BL, pwm);
        digitalWrite(BLDir, LOW);

        analogWrite(BR, pwm);
        digitalWrite(BRDir, HIGH);
    }
}

void moveForward(int pwm){
    /*
    Moves the robot forward at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
   analogWrite(FL, pwm);
   digitalWrite(FLDir, LOW);

   analogWrite(FR, pwm);
   digitalWrite(FRDir, LOW);

   analogWrite(BL, pwm);
   digitalWrite(BLDir, LOW);

   analogWrite(BR, pwm);
   digitalWrite(BRDir, LOW);
}




void moveBackward(int pwm){
    /*
    Moves the robot backward at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
    analogWrite(FL, pwm);
    digitalWrite(FLDir, HIGH);

    analogWrite(FR, pwm);
    digitalWrite(FRDir, HIGH);

    analogWrite(BL, pwm);
    digitalWrite(BLDir, HIGH);

    analogWrite(BR, pwm);
    digitalWrite(BRDir, HIGH);

}

void moveRight(int pwm){
    /*
    Moves the robot right at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
    analogWrite(FL, pwm);
    digitalWrite(FLDir, LOW);

    analogWrite(FR, pwm);
    digitalWrite(FRDir, HIGH);

    analogWrite(BL, pwm);
    digitalWrite(BLDir, HIGH);

    analogWrite(BR, pwm);
    digitalWrite(BRDir, LOW); 
}

void moveLeft(int pwm){
    /*
    Moves the robot left at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
    analogWrite(FL, pwm);
    digitalWrite(FLDir, HIGH);

    analogWrite(FR, pwm);
    digitalWrite(FRDir, LOW);

    analogWrite(BL, pwm);
    digitalWrite(BLDir, LOW);

    analogWrite(BR, pwm);
    digitalWrite(BRDir, HIGH); 
}

void move45(int pwm){
    /*
    Moves the robot +45 degrees with respect to the x-axis at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */

    analogWrite(FL, pwm);
    digitalWrite(FLDir, LOW);

    analogWrite(FR, 0);
    digitalWrite(FRDir, LOW);

    analogWrite(BL, 0);
    digitalWrite(BLDir, LOW);

    analogWrite(BR, pwm);
    digitalWrite(BRDir, LOW); 
}

void move135(int pwm){
    /*
    Moves the robot +135 degrees with respect to the x-axis at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
   
    analogWrite(FL, 0);
    digitalWrite(FLDir, LOW);

    analogWrite(FR, pwm);
    digitalWrite(FRDir, LOW);

    analogWrite(BL, pwm);
    digitalWrite(BLDir, LOW);

    analogWrite(BR, 0);
    digitalWrite(BRDir, LOW); 
}

void move315(int pwm){
    /*
    Moves the robot +315 degrees with respect to the x-axis at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
   
    analogWrite(FL, 0);
    digitalWrite(FLDir, LOW);

    analogWrite(FR, pwm);
    digitalWrite(FRDir, HIGH);

    analogWrite(BL, pwm);
    digitalWrite(BLDir, HIGH);

    analogWrite(BR, 0);
    digitalWrite(BRDir, LOW); 
}

void move225(int pwm){
    /*
    Moves the robot +225 degrees with respect to the x-axis at the given speed

    Args: 
    pwm(int): Speed 0 - 255

    Returns: 
    None
    */
   
    analogWrite(FL, pwm);
    digitalWrite(FLDir, HIGH);

    analogWrite(FR, 0);
    digitalWrite(FRDir, LOW);

    analogWrite(BL, 0);
    digitalWrite(BLDir, LOW);

    analogWrite(BR, pwm);
    digitalWrite(BRDir, HIGH); 
}

void stopMotors(){
    /*
    Shuts off all PWM signals

    Args: 
    None

    Returns: 
    None
    */

    analogWrite(FL, 0);
    analogWrite(FR, 0);
    analogWrite(BL, 0);
    analogWrite(BR, 0);

}


};