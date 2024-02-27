import time
from machine import I2C, Pin,PWM


# Simple omniwheel movement
# Forward, backward, right, left, and all 45-degree multiples implemented
#TODO: implement braking/test with actual robot

MAX_PWM = 65536 # Maximum value for PWM (This will basically be DC)
freq = 50 # PWM frequency

# Define motor pins
frontLeft1= Pin(0, Pin.OUT)
frontLeft2 = Pin(1, Pin.OUT)
frontRight1 = Pin(2, Pin.OUT)
frontRight2 = Pin(3, Pin.OUT)
backLeft1 = Pin(14, Pin.OUT)
backLeft2 = Pin(15, Pin.OUT)
backRight1 = Pin(17, Pin.OUT)
backRight2 = Pin(16, Pin.OUT)

# Create PWM objects for controlling motor speed
FL1 = PWM(frontLeft1, freq=freq, duty_u16 = 0)
FL2 = PWM(frontLeft2, freq=freq, duty_u16 = 0)
FR1 = PWM(frontRight1, freq=freq, duty_u16 = 0)
FR2 = PWM(frontRight2, freq=freq, duty_u16 = 0)
BL1 = PWM(backLeft1, freq=freq, duty_u16 = 0)
BL2 = PWM(backLeft2, freq=freq, duty_u16 = 0)
BR1 = PWM(backRight1, freq=freq, duty_u16 = 0)
BR2 = PWM(backRight2, freq=freq, duty_u16 = 0)



def move_forward(speed):
    """
    Moves the robot forward at the given speed

    Args: 
    speed (int) from 0-100 

    Returns: 
    none
    """
    pwm = int((speed/100)*MAX_PWM)
    FL1.duty_u16(pwm)
    FL2.duty_u16(0)
    FR1.duty_u16(pwm)
    FR2.duty_u16(0)
    BL1.duty_u16(pwm)
    BL2.duty_u16(0)
    BR1.duty_u16(pwm)
    BR2.duty_u16(0)

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
move_forward(500)  # Move forward at 50% speed
time.sleep(2)      # Move for 2 seconds
stop_motors()      # Stop all motors


