import pigpio, time

#=======================================================================//
# KEY VARIABLES
#=======================================================================//
# import os
# os.system("sudo pigpiod")
#===================================#
# ROBOT STATE
#===================================#
GPIO   = pigpio.pi()  # GPIO ACCESS THE LOCAL'S GPIO PINS
INPUT  = pigpio.INPUT
OUTPUT = pigpio.OUTPUT

LOW    = 0
HIGH   = 1
#===================================#
# ROBOT STATE
#===================================#
STOPPED         = 0
MOVING_FORWARD  = 1
MOVING_BACKWARD = 2
ROTATING        = 3

robot_state = STOPPED   # STORES THE ROBOT STATE
#=======================================================================//
# PIN DEFINITIONS
#=======================================================================//
#===================================#
# MOTOR PINS
#===================================#
# A IS FOR  LEFT MOTOR, B IS FOR RIGHT MOTOR
AIN2   = 20
AIN1   = 21
BIN2   = 19
BIN1   = 26
PWMA   = 16
PWMB   = 13
m_pins = [AIN2,AIN1,BIN2,BIN1,PWMA,PWMB]

#===================================#
# IR PINS
#===================================#
L_IR2   = 5 
L_IR1   = 3
R_IR1   = 4
R_IR2   = 18
ir_pins = [L_IR2,L_IR1,R_IR1,R_IR2]

CW  = 0
CCW = 1
#===================================#
# BUTTON/BUZZER PINS
#===================================#
BUTTON = 8  # WILL READ BUTTON WHEN SET AS INPUT
BUZZER = 8  # WILL CONTROL WHEN SET AS AN setup

#===================================#
# RGB SEARCHLIGHT
#===================================#
LED_R = 22
LED_G = 27
LED_B = 24
led_pins   = [LED_R,LED_G,LED_B]

LED_MAX_DUTY = 255
LED_MIN_DUTY = 0

#===================================#
# ULTRASONIC PINS
#===================================#
ULTRA_TX = 9  # J3
ULTRA_RX = 11 # J2

#===================================#
# CAMERA AND ULTRASONIC SERVOS
#===================================#
CAM_HORZ_SERVO   = 2
CAM_VERT_SERVO   = 25
ULTRA_HORZ_SERVO = 10
servo_pins = [CAM_HORZ_SERVO,CAM_VERT_SERVO,ULTRA_HORZ_SERVO]

# MIN AND MAX ANGLES OF SERVOS
SERVO_MIN_ANGLE = 0
SERVO_MAX_ANGLE = 180

# MIN, MID, AND MAX PULSE WIDTH FOR SERVOS 
SERVO_MIN_PULSE_WIDTH_MS = 500 # 500 ?
SERVO_MID_PULSE_WIDTH_MS = 1500
SERVO_MAX_PULSE_WIDTH_MS = 2400 # 2400 ?

#===================================#
# IR REMOTE
#===================================#
IR_REMOTE = 2

#=======================================================================//
# MAIN FUNCTIONS
#=======================================================================//
#===================================#
# INIT FUNCTIONS
#===================================#
def init():
    #===================================#
    # SETUP MOTORS
    #===================================#
    # SETUP ALL MOTOR ASSOCIATED PIN AS AN OUTPUT
    # GPIO.set_mode(m_pins,OUTPUT)   
    # SETUP THE PWM FREQUENCY AND DUTY CYCLE         
    GPIO.set_PWM_frequency(PWMA,1000)       
    GPIO.set_PWM_frequency(PWMB,1000)
    GPIO.set_PWM_dutycycle(PWMA,0)
    GPIO.set_PWM_dutycycle(PWMB,0)

    #===================================#
    # SETUP LEDS
    #===================================#
    # SETUP ALL LED PINS AS AN OUTPUT
    # GPIO.set_mode(led_pins,OUTPUT)
    # SETUP THE PWM FREQUENCY AND DUTY CYCLE
    GPIO.set_PWM_frequency(LED_R,1000)
    GPIO.set_PWM_frequency(LED_G,1000)
    GPIO.set_PWM_frequency(LED_B,1000)
    GPIO.set_PWM_dutycycle(LED_R,0)
    GPIO.set_PWM_dutycycle(LED_G,0)
    GPIO.set_PWM_dutycycle(LED_B,0)

    #===================================#
    # SETUP BUZZER
    #===================================#
    # SETUP THE PWM FREQUENCY AND DUTY CYCLE
    GPIO.set_PWM_frequency(BUZZER,100)
    GPIO.set_PWM_dutycycle(BUZZER,255)

    #===================================#
    # SETUP SERVOS
    #===================================#
    # BEGIN SERVOS AT MID PULSE WIDTH TO BE SET AT 90 DEGREES
    # LIBRARY SETS PINS AS OUTPUT AND BEGINS PULSE FREQUENCY TO 50Hz
    GPIO.set_servo_pulsewidth(CAM_HORZ_SERVO,   SERVO_MID_PULSE_WIDTH_MS)
    GPIO.set_servo_pulsewidth(CAM_VERT_SERVO,   SERVO_MID_PULSE_WIDTH_MS)
    GPIO.set_servo_pulsewidth(ULTRA_HORZ_SERVO, SERVO_MID_PULSE_WIDTH_MS)

    #===================================#
    # SETUP THE ULTRASONIC SENSOR
    #===================================#
    #GPIO.set_mode(ULTRA_TX,OUTPUT)
    #GPIO.set_mode(ULTRA_RX,INPUT)

    #===================================#
    # SETUP THE IR SENSORS
    #===================================#
    GPIO.set_mode(L_IR2,INPUT)
    GPIO.set_mode(L_IR1,INPUT)
    GPIO.set_mode(R_IR1,INPUT)
    GPIO.set_mode(R_IR2,INPUT)
    
#===================================#
# MOVE FORWARD FUNCTIONS
#===================================#
def moveForward(dutyCycle):
    global robot_state, GPIO
    # set the robot's state
    robot_state = MOVING_FORWARD 
    #=============================#
    # SET THE DIRECTION
    #=============================#
    # LEFT MOTOR SHOULD BE CCW
    GPIO.write(AIN1,LOW)
    GPIO.write(AIN2,HIGH)

    # RIGHT MOTOR SHOULD BE CW
    GPIO.write(BIN1,LOW)
    GPIO.write(BIN2,HIGH)

    #=============================#
    # SET THE SPEED
    #=============================#
    GPIO.set_PWM_dutycycle(PWMA,dutyCycle)
    GPIO.set_PWM_dutycycle(PWMB,dutyCycle)

#===================================#
# MOVE BACKWARD FUNCTIONS
#===================================#
def moveBackward(dutyCycle):
    global robot_state, GPIO
    # set the robot's state
    robot_state = MOVING_BACKWARD
    #=============================#
    # SET THE DIRECTION
    #=============================#
    # LEFT MOTOR SHOULD BE CW
    GPIO.write(AIN1,HIGH)
    GPIO.write(AIN2,LOW)

    # RIGHT MOTOR SHOULD BE CCW
    GPIO.write(BIN1,HIGH)
    GPIO.write(BIN2,LOW)

    #=============================#
    # SET THE SPEED
    #=============================#
    GPIO.set_PWM_dutycycle(PWMA,dutyCycle)
    GPIO.set_PWM_dutycycle(PWMB,dutyCycle)

#===================================#
# ROTATE FUNCTIONS
# dir = 0 -> CW, 1 -> CCW
#===================================#
def rotate(dir,dutyCycle):
    global robot_state, GPIO
    # set the robot's state
    robot_state = ROTATING
    #=============================#
    # SET THE DIRECTION
    #=============================#
    if(dir == 0):
        # LEFT MOTOR SHOULD BE 
        GPIO.write(AIN1,LOW)
        GPIO.write(AIN2,HIGH)

        # RIGHT MOTOR SHOULD BE 
        GPIO.write(BIN1,HIGH)
        GPIO.write(BIN2,LOW)
        
    else:
        # LEFT MOTOR SHOULD BE
        GPIO.write(AIN1,HIGH)
        GPIO.write(AIN2,LOW)

        # RIGHT MOTOR SHOULD BE
        GPIO.write(BIN1,LOW)
        GPIO.write(BIN2,HIGH)

    #=============================#
    # SET THE SPEED
    #=============================#
    GPIO.set_PWM_dutycycle(PWMA,dutyCycle)
    GPIO.set_PWM_dutycycle(PWMB,dutyCycle)

#===================================#
# STOP FUNCTIONS
#===================================#
def stop():
    global robot_state, GPIO
    robot_state = STOPPED
    # LEFT MOTOR SHOULD BE CCW
    GPIO.write(AIN1,LOW)
    GPIO.write(AIN2,LOW)

    # RIGHT MOTOR SHOULD BE CW
    GPIO.write(BIN1,LOW)
    GPIO.write(BIN2,LOW)

    #=============================#
    # SET THE SPEED TO 0,
    # NOT NEEDED TO STOP MOTOR, BUT
    # JUST SO THAT THE MOTOR'S DON'T 
    # START SPINING IN CASE THE USER 
    # CHANGES MOTOR'S DIRECTION.
    #=============================#
    GPIO.set_PWM_dutycycle(PWMA,0)
    GPIO.set_PWM_dutycycle(PWMB,0)

#===================================#
# GET DISTANCE READING
#===================================#
def ping(angle):
    timeout = 2
    setUltraServo(angle)
    
    # LET SENSOR SETTLE
    GPIO.write(ULTRA_TX,LOW)
    time.sleep(2)
    
    GPIO.write(ULTRA_TX,HIGH)
    time.sleep(0.00001)
    GPIO.write(ULTRA_TX,LOW)

    # While the echo is still low, keep getting
    # the timestamp
    while GPIO.read(ULTRA_RX)==LOW:
        pulse_start = time.time()

    # As soon as the echo is heard back, ULTRA_RX goes HIGH
    # So, while it's high, keep getting the current time
    while GPIO.read(ULTRA_RX) == HIGH:
        pulse_end = time.time()

    # Calculate pulse duration
    pulse_duration = pulse_end - pulse_start

    # speed = Distance/time, speed of sound at sea level = 343m/s
    # and since the distance measured is the distance to and from wall
    # we need to divide by 2, therefore, 34300cm/s = (Distance/2)/Time
    distance = pulse_duration*17150
    # Round up value to 2 decimal places
    distance = round(distance,2)
    return (distance)

#===================================#
# SET RGB SEARCHLIGHT [GOOD]
#===================================#
def setLED(R_DUTY,G_DUTY,B_DUTY):
    global GPIO

    # Make sure values are good
    if R_DUTY>LED_MAX_DUTY:
        R_DUTY = LED_MAX_DUTY
    if G_DUTY>LED_MAX_DUTY:
        G_DUTY = LED_MAX_DUTY
    if B_DUTY>LED_MAX_DUTY:
        B_DUTY = LED_MAX_DUTY

    if R_DUTY<LED_MIN_DUTY:
        R_DUTY = LED_MIN_DUTY
    if G_DUTY<LED_MIN_DUTY:
        G_DUTY = LED_MIN_DUTY
    if B_DUTY<LED_MIN_DUTY:
        B_DUTY = LED_MIN_DUTY

    # Change duty Cycle
    GPIO.set_PWM_dutycycle(LED_R,R_DUTY)
    GPIO.set_PWM_dutycycle(LED_G,G_DUTY)
    GPIO.set_PWM_dutycycle(LED_B,B_DUTY)

#===================================#
# SET ULRASONIC SERVO ANGLE [GOOD]
#===================================#
def setUltraServo(angle):
    global ULTRA_HORZ_SERVO
    setServoAngle(ULTRA_HORZ_SERVO,angle)

#===================================#
# SET CAMERA SERVO ANGLES [GOOD]
#===================================#
def setCameraHorzServo(angle):
    global CAM_HORZ_SERVO
    setServoAngle(CAM_HORZ_SERVO,angle)

def setCameraVertServo(angle):
    global CAM_VERT_SERVO
    setServoAngle(CAM_VERT_SERVO,angle)

def setCameraServos(horz_angle,vert_angle):
    setCameraHorzServo(horz_angle)
    setCameraVertServo(vert_angle)
    
#===================================#
# READ BUTTON
#===================================#
def readButton():
    # The Buzzer and Button share the same pin. Thus, there pin
    # will be setup as an input or output right before its use
    GPIO.set_mode(BUTTON,INPUT)
    return GPIO.read(BUTTON)

#===================================#
# READ IR [GOOD]
#===================================#
def readFarLeftIR():
    return GPIO.read(L_IR1)

def readLeftIR():
    return GPIO.read(L_IR2)

def readRightIR():
    return GPIO.read(R_IR1)

def readFarRightIR():
    return GPIO.read(R_IR2)

#===================================#
# SET BUZZER [GOOD]
#===================================#
def setBuzzer(duty,frequency):
    global BUZZER
    if duty>255:
        duty = 255
    if duty<0:
        duty = 0

    # The Buzzer and Button share the same pin. Thus, there pin
    # will be setup as an input or output right before its use
    GPIO.set_mode(BUZZER,OUTPUT)
    GPIO.set_PWM_frequency(BUZZER,frequency)
    GPIO.set_PWM_dutycycle(BUZZER,duty)

#=======================================================================//
# HELPER FUNCTIONS 
#=======================================================================//
#===================================#
# SET SERVO'S ANGLE [GOOD]
#===================================#
def setServoAngle(servo_pin,angle):
    if angle>SERVO_MAX_ANGLE:
        angle = SERVO_MAX_ANGLE
    if angle<SERVO_MIN_ANGLE:
        angle = SERVO_MIN_ANGLE

    # Calculate the necessary pulseWidth
    pulseWidth = SERVO_MIN_PULSE_WIDTH_MS + (angle)*(SERVO_MAX_PULSE_WIDTH_MS-SERVO_MIN_PULSE_WIDTH_MS)/(SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)
    # Set the duty cycle
    GPIO.set_servo_pulsewidth(servo_pin,pulseWidth)
    
#===================================#
# SET MOTORS SPEED FUNCTIONS (FUTURE
# IF NEEDED)
#===================================#
def setLeftSpeed(speed):
    pass

def setRightSpeed(speed):
    pass