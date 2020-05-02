import RPi.GPIO as GPIO
from gpiozero import DistanceSensor, LineSensor
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

STOPPED         = 0
MOVING_FORWARD  = 1
MOVING_BACKWARD = 2
ROTATING        = 3

robot_state = STOPPED
#=======================================================================//
# PIN DEFINITIONS
#=======================================================================//
#===================================#
# MOTOR PINS
#===================================#
# A IS FOR  LEFT MOTOR, B IS FOR RIGHT MOTOR
AIN2 = 20
AIN1 = 21
BIN2 = 19
BIN1 = 26
PWMA = 16
PWMB = 13

L_MOTOR_PWM = None # Will store the pwm object for the left motor
R_MOTOR_PWM = None # Will store the pwm object for the right motor
#===================================#
# IR PINS
#===================================#
L_IR2 = 5 
L_IR1 = 3
R_IR1 = 4
R_IR2 = 18

# Objects to hold lineSensor objects
far_left_ir  = None
left_ir      = None
right_ir     = None
far_right_ir = None
#===================================#
# BUTTON/BUZZER PINS
#===================================#
BUTTON_PIN = 8  # WILL READ BUTTON WHEN SET AS INPUT
BUZZER_PIN = 8  # WILL CONTROL WHEN SET AS AN setup

BUZZER = None   # Variable to hold buzzers PWM object
#===================================#
# RGB SEARCHLIGHT
#===================================#
LED_R_PIN = 22
LED_G_PIN = 27
LED_B_PIN = 24

# Objects to hold led's pwm objects
LED_R = None
LED_G = None
LED_B = None

LED_MAX_VAL = 255
LED_MIN_VAL = 0
#===================================#
# ULTRASONIC PINS
#===================================#
ULTRA_TX = 0
ULTRA_RX = 1
ultraSensor = None
#===================================#
# CAMERA SERVOS
#===================================#
CAM_HORZ_SERVO_PIN = 2
CAM_VERT_SERVO_PIN = 25

# Variables to hold PWM objects
CAM_HORZ_SERVO = None
CAM_VERT_SERVO = None

# Angle Servo can reach
SERVO_MIN_ANGLE = -90
SERVO_MAX_ANGLE = 90

# Duty cycles corresponding to Servo min 
# and max angles 
SERVO_MIN_DUTY = 5
SERVO_MID_DUTY = 7.5
SERVO_MAX_DUTY = 10

# Update Rate 50 Hz = Period of 20ms
SERVO_UPDATE_RATE_HZ = 50

#===================================#
# ULTRASONIC SERVOS
#===================================#
ULTRA_HORZ_SERVO_PIN = 10

# Variables to hold PWM objects
ULTRA_HORZ_SERVO = None
ULTRA_VERT_SERVO = None
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
    global L_MOTOR_PWM, R_MOTOR_PWM
    global CAM_HORZ_SERVO, CAM_VERT_SERVO, ULTRA_HORZ_SERVO
    global LED_R,LED_G,LED_B,BUZZER,
    global far_left_ir, left_ir, right_ir, far_right_ir, ultraSensor
    
    # SET UP SIMPLE PINS
    m_pins     = [AIN2,AIN1,BIN2,BIN1,PWMA,PWMB]
    #ir_pins    = [L_IR2,L_IR1,R_IR1,R_IR2]
    servo_pins = [CAM_HORZ_SERVO,CAM_VERT_SERVO,ULTRA_HORZ_SERVO]
    led_pins   = [LED_R,LED_G,LED_B] 

    GPIO.setup(m_pins,GPIO.OUT)
    #GPIO.setup(ir_pins,GPIO.IN)
    GPIO.setup(servo_pins,GPIO.OUT)
    GPIO.setup(led_pins,GPIO.OUT)

    # The Buzzer and Button share the same pin. Thus, there pin
    # will be setup as an input or output right before its use
    #out_pins = [ULTRA_TX]
    #in_pins  = [ULTRA_RX]
    #GPIO.setup(out_pins,GPIO.OUT)
    #GPIO.setup(in_pins,GPIO.IN)
    ultraSensor = DistanceSensor(echo=ULTRA_RX,trigger=ULTRA_TX)
    
    far_left_ir  = LineSensor(L_IR2)
    left_ir      = LineSensor(L_IR1)
    right_ir     = LineSensor(R_IR1)
    far_right_ir = LineSensor(R_IR2)

    # SET UP PINS WITH PWM REQUIREMENTS
    # GPIO.PWM([pin], [frequency])
    L_MOTOR_PWM = GPIO.PWM(PWMA,1000)
    R_MOTOR_PWM = GPIO.PWM(PWMB,1000)

    CAM_HORZ_SERVO   = GPIO.PWM(CAM_HORZ_SERVO_PIN,SERVO_UPDATE_RATE_HZ)  # 50 Hz
    CAM_VERT_SERVO   = GPIO.PWM(CAM_VERT_SERVO_PIN,SERVO_UPDATE_RATE_HZ)
    ULTRA_HORZ_SERVO = GPIO.PWM(ULTRA_HORZ_SERVO_PIN,SERVO_UPDATE_RATE_HZ)

    LED_R = GPIO.PWM(LED_R_PIN,1000)
    LED_G = GPIO.PWM(LED_G_PIN,1000)
    LED_B = GPIO.PWM(LED_B_PIN,1000)

    BUZZER = GPIO.PWM(BUZZER_PIN,100)

    # pwm.start([duty cycle])
    # Start both motors at 0 duty cycle, 
    L_MOTOR_PWM.start(0)
    R_MOTOR_PWM.start(0)

    # Start servo's at 0 degrees (MIDDLE) 
    CAM_HORZ_SERVO.start(SERVO_MID_DUTY)
    CAM_VERT_SERVO.start(SERVO_MID_DUTY)
    ULTRA_HORZ_SERVO.start(SERVO_MID_DUTY) 

    # Start LED's off
    LED_R.start(LED_MIN_VAL)
    LED_G.start(LED_MIN_VAL)
    LED_B.start(LED_MIN_VAL)

    # START BUZZER OFF
    BUZZER.start(0)

    # Setup ultrasonic sensor
    GPIO.setup(ULTRA_TX,GPIO.OUT)
    GPIO.setup(ULTRA_RX,GPIO.IN)
    # Let ultrasonic sensor settle
    GPIO.output(ULTRA_TX,GPIO.LOW)
    time.sleep(2)

#===================================#
# MOVE FORWARD FUNCTIONS
#===================================#
def moveForward(dutyCycle):
    global robot_state, L_MOTOR_PWM, R_MOTOR_PWM
    # set the robot's state
    robot_state = MOVING_FORWARD 
    #=============================#
    # SET THE DIRECTION
    #=============================#
    # LEFT MOTOR SHOULD BE CCW
    GPIO.output(AIN1,GPIO.LOW)
    GPIO.output(AIN2,GPIO.HIGH)

    # RIGHT MOTOR SHOULD BE CW
    GPIO.output(BIN1,GPIO.HIGH)
    GPIO.output(BIN2,GPIO.LOW)

    #=============================#
    # SET THE SPEED
    # pwm.ChangeDutyCycle([duty cycle])
    #=============================#
    L_MOTOR_PWM.ChangeDutyCycle(dutyCycle)
    R_MOTOR_PWM.ChangeDutyCycle(dutyCycle)


#===================================#
# MOVE BACKWARD FUNCTIONS
#===================================#
def moveBackward(dutyCycle):
    global robot_state, L_MOTOR_PWM, R_MOTOR_PWM
    # set the robot's state
    robot_state = MOVING_BACKWARD
    #=============================#
    # SET THE DIRECTION
    #=============================#
    # LEFT MOTOR SHOULD BE CW
    GPIO.output(AIN1,GPIO.HIGH)
    GPIO.output(AIN2,GPIO.LOW)

    # RIGHT MOTOR SHOULD BE CCW
    GPIO.output(BIN1,GPIO.LOW)
    GPIO.output(BIN2,GPIO.HIGH)

    #=============================#
    # SET THE SPEED
    #=============================#
    L_MOTOR_PWM.ChangeDutyCycle(dutyCycle)
    R_MOTOR_PWM.ChangeDutyCycle(dutyCycle)


#===================================#
# ROTATE FUNCTIONS
# dir = 0 -> CW, 1 -> CCW
#===================================#
def rotate(dutyCycle,dir):
    global robot_state, L_MOTOR_PWM, R_MOTOR_PWM
    # set the robot's state
    robot_state = ROTATING
    #=============================#
    # SET THE DIRECTION
    #=============================#
    if(dir == 0):
        # LEFT MOTOR SHOULD BE CCW
        GPIO.output(AIN1,GPIO.LOW)
        GPIO.output(AIN2,GPIO.HIGH)

        # RIGHT MOTOR SHOULD BE CW
        GPIO.output(BIN1,GPIO.LOW)
        GPIO.output(BIN2,GPIO.HIGH)
    else:
        # LEFT MOTOR SHOULD BE CCW
        GPIO.output(AIN1,GPIO.LOW)
        GPIO.output(AIN2,GPIO.HIGH)

        # RIGHT MOTOR SHOULD BE CW
        GPIO.output(BIN1,GPIO.LOW)
        GPIO.output(BIN2,GPIO.HIGH)

    #=============================#
    # SET THE SPEED
    #=============================#
    L_MOTOR_PWM.ChangeDutyCycle(dutyCycle)
    R_MOTOR_PWM.ChangeDutyCycle(dutyCycle)


#===================================#
# STOP FUNCTIONS
#===================================#
def stop():
    global robot_state, L_MOTOR_PWM, R_MOTOR_PWM
    robot_state = STOPPED
    # LEFT MOTOR SHOULD BE CCW
    GPIO.output(AIN1,GPIO.LOW)
    GPIO.output(AIN2,GPIO.LOW)

    # RIGHT MOTOR SHOULD BE CW
    GPIO.output(BIN1,GPIO.LOW)
    GPIO.output(BIN2,GPIO.LOW)

    #=============================#
    # SET THE SPEED TO 0,
    # NOT NEEDED TO STOP MOTOR, BUT
    # JUST SO THAT THE MOTOR'S DON'T 
    # START SPINING IN CASE THE USER 
    # CHANGES MOTOR'S DIRECTION.
    #=============================#
    L_MOTOR_PWM.ChangeDutyCycle(0)
    R_MOTOR_PWM.ChangeDutyCycle(0)
    pass


#===================================#
# GET DISTANCE READING
#===================================#
def ping(angle):
    global ultraSensor
    # GPIO.output(ULTRA_TX,GPIO.HIGH)
    # time.sleep(0.00001)
    # GPIO.output(ULTRA_TX,GPIO.LOW)

    # # While the echo is still low, keep getting
    # # the timestamp
    # while GPIO.input(ULTRA_RX)==GPIO.LOW:
    #     pulse_start = time.time()

    # # As soon as the echo is heard back, ULTRA_RX goes HIGH
    # # So, while it's high, keep getting the current time
    # while GPIO.input(ULTRA_RX) == GPIO.HIGH:
    #     pulse_end = time.time()

    # # Calculate pulse duration
    # pulse_duration = pulse_end - pulse_start

    # # speed = Distance/time, speed of sound at sea level = 343m/s
    # # and since the distance measured is the distance to and from wall
    # # we need to divide by 2, therefore, 34300cm/s = (Distance/2)/Time
    # distance = pulse_duration*17150
    # # Round up value to 2 decimal places
    # distance = round(distance,2)    
    return (ultraSensor.distance * 100)

#===================================#
# SET RGB SEARCHLIGHT
#===================================#
def setLED(R,G,B):
    global LED_R, LED_G, LED_B

    # Make sure values are good
    if R>LED_MAX_VAL:
        R = LED_MAX_VAL
    if G>LED_MAX_VAL:
        G = LED_MAX_VAL
    if B>LED_MAX_VAL:
        B = LED_MAX_VAL

    if R<LED_MIN_VAL:
        R = LED_MIN_VAL
    if G<LED_MIN_VAL:
        G = LED_MIN_VAL
    if B<LED_MIN_VAL:
        B = LED_MIN_VAL

    # Calculate Duty Cycle
    r_duty = (R/LED_MAX_VAL) * 100
    g_duty = (G/LED_MAX_VAL) * 100
    b_duty = (B/LED_MAX_VAL) * 100

    # Change duty Cycle
    LED_R.ChangeDutyCycle(r_duty)
    LED_G.ChangeDutyCycle(g_duty)
    LED_B.ChangeDutyCycle(b_duty)


#===================================#
# SET ULRASONIC SERVO ANGLE
#===================================#
def setUltraServo(angle):
    global ULTRA_HORZ_SERVO
    setServoAngle(ULTRA_HORZ_SERVO,angle)
    pass

#===================================#
# SET CAMERA SERVO ANGLES
#===================================#
def setCameraHorzServos(angle):
    global CAM_HORZ_SERVO
    setServoAngle(CAM_HORZ_SERVO,angle)

def setCameraVertServos(angle):
    global CAM_VERT_SERVO
    setServoAngle(CAM_VERT_SERVO,angle)

def setCameraServos(horz_angle,vert_angle):
    setCameraHorzServos(horz_angle)
    setCameraVertServos(vert_angle)

#===================================#
# READ BUTTON
#===================================#
def readButton():
    # The Buzzer and Button share the same pin. Thus, there pin
    # will be setup as an input or output right before its use
    GPIO.setup(BUTTON_PIN,GPIO.IN)
    return GPIO.input(BUTTON_PIN)


#===================================#
# SET BUZZER
#===================================#
def setBuzzer(duty,frequency):
    global BUZZER
    if duty>100:
        duty = 100
    if duty<0:
        duty = 0

    # The Buzzer and Button share the same pin. Thus, there pin
    # will be setup as an input or output right before its use
    GPIO.setup(BUZZER_PIN,GPIO.OUT)
    BUZZER.ChangeFrequency(frequency)
    BUZZER.ChangeDutyCycle(duty)
    pass


#=======================================================================//
# HELPER FUNCTIONS
#=======================================================================//
# Angles limited to -90 to 90
def setServoAngle(servo_pwm_obj,angle):
    # Angles limited to -90 to 90
    if angle>SERVO_MAX_ANGLE:
        angle = 90
    if angle<SERVO_MIN_ANGLE:
        angle = -90

    # Calculate the necessary duty cycle
    dutyCycle = SERVO_MID_DUTY + (angle-SERVO_MIN_ANGLE)*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/(SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)
    # Set the duty cycle
    servo_pwm_obj.ChangeDutyCycle(dutyCycle)
    

#===================================#
# SET MOTORS SPEED FUNCTIONS (FUTURE
# IF NEEDED)
#===================================#
def setLeftSpeed(speed):
    pass

def setRightSpeed(speed):
    pass