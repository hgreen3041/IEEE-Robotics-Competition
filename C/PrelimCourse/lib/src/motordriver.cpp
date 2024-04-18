#include <Arduino.h> 
#include <cmath>   
#include <motordriver.h> 


// Constructor
motordriver :: motordriver(){
        // Setup motor pins
    pinMode(FL, OUTPUT);
    pinMode(FLDir, OUTPUT);
    pinMode(FR, OUTPUT);
    pinMode(FRDir, OUTPUT);
    pinMode(BL, OUTPUT);
    pinMode(BLDir, OUTPUT);
    pinMode(BR, OUTPUT);
    pinMode(BRDir, OUTPUT);

}



// Destructor
    motordriver :: ~motordriver() {
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


void motordriver :: M1(int pwm){
    if(pwm >= 0 ){
    analogWrite(FL, pwm);
    digitalWrite(FLDir, LOW);
    }

    else{
        pwm = abs(pwm);
        analogWrite(FL, pwm);
        digitalWrite(FLDir, HIGH);


    }
}

void motordriver :: M2(int pwm){
    if(pwm >= 0 ){
    analogWrite(FR, pwm);
    digitalWrite(FRDir, LOW);
    }

    else{
        pwm = abs(pwm);
        analogWrite(FR, pwm);
        digitalWrite(FRDir, HIGH);


    }
}

void motordriver :: M3(int pwm){
    if(pwm >= 0 ){
    analogWrite(BL, pwm);
    digitalWrite(BLDir, LOW);
    }

    else{
        pwm = abs(pwm);
        analogWrite(BL, pwm);
        digitalWrite(BLDir, HIGH);


    }
}

void motordriver :: M4(int pwm){
    if(pwm >= 0 ){
    analogWrite(BR, pwm);
    digitalWrite(BRDir, LOW);
    }

    else{
        pwm = abs(pwm);
        analogWrite(BR, pwm);
        digitalWrite(BRDir, HIGH);


    }
}
void motordriver :: yMovement(int pwm){
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

void motordriver :: xMovement(int pwm){
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

void motordriver :: rotationalMovement(int pwm){
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

void motordriver :: moveForward(int pwm){
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




void motordriver :: moveBackward(int pwm){
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

void motordriver :: moveRight(int pwm){
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

void motordriver :: moveLeft(int pwm){
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

void motordriver :: move45(int pwm){
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

void motordriver :: move135(int pwm){
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

void motordriver :: move315(int pwm){
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

void motordriver :: move225(int pwm){
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

void motordriver :: stopMotors(){
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


