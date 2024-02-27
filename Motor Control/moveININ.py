import time
from machine import I2C, Pin,PWM


# Simple omniwheel movement using DRV8835 Dual Motor Driver Carrier in IN/IN mode. 
# Forward, backward, right, left, and all 45-degree multiples implemented


# This is not a good option. Uses too many GPIO'S. Phase/Enable mode is better for our application
#TODO: Fix interfering PWM pins.


MAX_PWM = 65536 # Maximum value for PWM (This will basically be DC)
freq = 10 # PWM frequency



# Create PWM objects for controlling motor speed
FL1 = PWM(Pin(0))
print(FL1)
FL2 = PWM(Pin(1))
print(FL2)
FR1 = PWM(Pin(2))
print(FR1)
FR2 = PWM(Pin(3))
print(FR2)
BL1 = PWM(Pin(14))
print(BL1)
BL2 = PWM(Pin(15))
print(BL2)
BR1 = PWM(Pin(17))
print(BR1)
BR2 = PWM(Pin(16))
print(BR2)
FL1.freq(50)



def move_forward(speed):
    """
    Moves the robot forward at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    # print(pwm)
    # print(FL1)
    FL1.duty_u16(pwm)
    # FL2.duty_u16(0)
    # FR1.duty_u16(pwm)
    # FR2.duty_u16(0)
    # BL1.duty_u16(pwm) 
    # BL2.duty_u16(0)
    # BR1.duty_u16(pwm)
    # BR2.duty_u16(0)

def move_backward(speed):
    """
    Moves the robot backward at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(0)
    FL2.duty_u16(pwm)
    FR1.duty_u16(0)
    FR2.duty_u16(pwm)
    BL1.duty_u16(0)
    BL2.duty_u16(pwm)
    BR1.duty_u16(0)
    BR2.duty_u16(pwm)

def move_right(speed):
    """
    Moves the robot to the right at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(0)
    FL2.duty_u16(pwm)
    FR1.duty_u16(pwm)
    FR2.duty_u16(0)
    BL1.duty_u16(pwm)
    BL2.duty_u16(0)
    BR1.duty_u16(0)
    BR2.duty_u16(pwm)

def move_left(speed):
    """
    Moves the robot to the left at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(pwm)
    FL2.duty_u16(0)
    FR1.duty_u16(0)
    FR2.duty_u16(pwm)
    BL1.duty_u16(0)
    BL2.duty_u16(pwm)
    BR1.duty_u16(pwm)
    BR2.duty_u16(0)
    
def move_45(speed):
    """
    Moves the robot at 45 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(0)
    FL2.duty_u16(0)
    FR1.duty_u16(pwm)
    FR2.duty_u16(0)
    BL1.duty_u16(pwm)
    BL2.duty_u16(0)
    BR1.duty_u16(0)
    BR2.duty_u16(0)

def move_135(speed):
    """
    Moves the robot at 135 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(pwm)
    FL2.duty_u16(0)
    FR1.duty_u16(0)
    FR2.duty_u16(0)
    BL1.duty_u16(0)
    BL2.duty_u16(0)
    BR1.duty_u16(pwm)
    BR2.duty_u16(0) 

def move_315(speed):  
    """
    Moves the robot at 315 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(0)
    FL2.duty_u16(pwm)
    FR1.duty_u16(0)
    FR2.duty_u16(0)
    BL1.duty_u16(0)
    BL2.duty_u16(0)
    BR1.duty_u16(0)
    BR2.duty_u16(pwm) 

def move_225(speed):  
    """
    Moves the robot at 225 degrees measured from +x-axis

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(0)
    FL2.duty_u16(0)
    FR1.duty_u16(0)
    FR2.duty_u16(pwm)
    BL1.duty_u16(0)
    BL2.duty_u16(pwm)
    BR1.duty_u16(0)
    BR2.duty_u16(0) 

def stop_motors():
    """
    Shuts off all PWM signals. Should come to a coast stop (probably not desired)
    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    FL1.duty_u16(0)
    FL2.duty_u16(0)
    FR1.duty_u16(0)
    FR2.duty_u16(0)
    BL1.duty_u16(0)
    BL2.duty_u16(0)
    BR1.duty_u16(0)
    BR2.duty_u16(0)
  


# Example usage

move_forward(100)  # Move forward at 50% speed
# time.sleep(10)
# stop_motors()      # Stop all motors


