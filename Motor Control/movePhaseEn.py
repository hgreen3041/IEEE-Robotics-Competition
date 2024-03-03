import time
from machine import I2C, Pin,PWM

# Test script for simple omni-wheel movement using the DRV8835 Dual Motor Driver Carrier in Phase/enable mode
# Tested with LED to verify all PWM's are working


MAX_PWM = 65536 # Maximum value for PWM (This will basically be DC)
freq = 10 # PWM frequency

# Setup for front-left motor
FLSpeed = PWM(Pin(0), freq=freq) # check led
FLDir = Pin(1, Pin.OUT)

# setup for front-right motor
FRSpeed = PWM(Pin(28), freq=freq) # check led
FRDir = Pin(27, Pin.OUT)
# setup for back-left motor
BLSpeed = PWM(Pin(14), freq=freq) # check led
BLDir = Pin(15, Pin.OUT)

#setup for back-right motor
BRSpeed = PWM(Pin(22), freq=freq) # check led
BRDir = Pin(2, Pin.OUT)


def move_forward(speed):
    """
    Moves the robot forward at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(pwm)
    FLDir.value(0)

    FRSpeed.duty_u16(pwm)
    FRDir.value(0)

    BLSpeed.duty_u16(pwm)
    BLDir.value(0)

    BRSpeed.duty_u16(pwm)
    BRDir.value(0)


def move_backward(speed):
    """
    Moves the robot backward at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(pwm)
    FLDir.value(1)
    FRSpeed.duty_u16(pwm)
    FRDir.value(1)
    BLSpeed.duty_u16(pwm)
    BLDir.value(1)
    BRSpeed.duty_u16(pwm)
    BRDir.value(1)

def move_right(speed):
    """
    Moves the robot to the right at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(pwm)
    FLDir.value(0)
    FRSpeed.duty_u16(pwm)
    FRDir.value(1)
    BLSpeed.duty_u16(pwm)
    BLDir.value(1)
    BRSpeed.duty_u16(pwm)
    BRDir.value(0)

def move_left(speed):
    """
    Moves the robot to the left at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(pwm)
    FLDir.value(1)
    FRSpeed.duty_u16(pwm)
    FRDir.value(0)
    BLSpeed.duty_u16(pwm)
    BLDir.value(0)
    BRSpeed.duty_u16(pwm)
    BRDir.value(1)

def move_45(speed):
    """
    Moves the robot at 45 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(pwm)
    FLDir.value(0)
    FRSpeed.duty_u16(0)
    FRDir.value(0)
    BLSpeed.duty_u16(0)
    BLDir.value(0)
    BRSpeed.duty_u16(pwm)
    BRDir.value(0)

def move_135(speed):
    """
    Moves the robot at 135 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(0)
    FLDir.value(0)
    FRSpeed.duty_u16(pwm)
    FRDir.value(0)
    BLSpeed.duty_u16(pwm)
    BLDir.value(0)
    BRSpeed.duty_u16(0)
    BRDir.value(0)

def move_315(speed):  
    """
    Moves the robot at 315 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(0)
    FLDir.value(0)
    FRSpeed.duty_u16(pwm)
    FRDir.value(1)
    BLSpeed.duty_u16(pwm)
    BLDir.value(1)
    BRSpeed.duty_u16(0)
    BRDir.value(0)

def move_225(speed):  
    """
    Moves the robot at 225 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FLSpeed.duty_u16(pwm)
    FLDir.value(1)
    FRSpeed.duty_u16(0)
    FRDir.value(0)
    BLSpeed.duty_u16(0)
    BLDir.value(0)
    BRSpeed.duty_u16(pwm)
    BRDir.value(1)

def stop_motors():
    """
    Shuts off all PWM signals. 
    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    FLSpeed.duty_u16(0)
    FRSpeed.duty_u16(0)
    BLSpeed.duty_u16(0)
    BRSpeed.duty_u16(0)


# Test all
move_forward(100)
time.sleep(5)
move_backward(100)
time.sleep(5)
move_left(100)
time.sleep(5)
move_right(100)
time.sleep(5)
move_45(100)
time.sleep(5)
move_315(100)
time.sleep(5)
move_135(100)
time.sleep(5)
move_225(100)
time.sleep(5)
stop_motors()
    
# move_225(75)
# time.sleep(10)
# stop_motors()


