import cv2
import robot_control as rc
import time

#===================================#
# FOR OPENCV FACE DETECTION
#===================================#
# Load the cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') # Load Cascade 
cap = cv2.VideoCapture(0)                                                   # Camera Capture for 
#===================================#
# VARIABLES
#===================================#
START_WAITING_TIME = 5  # THE AMOUNT OF SECONDS TO WAIT BEFORE STARTING AFTER THE START BUTTON IS PRESSED

TAPE_DETECTED    = 1
NO_TAPE_DETECTED = 0

FORWARD_SPEED    = 120
NO_TAPE_OFFSET   = 10

ROTATE_SPEED        = 100
ROTATE_SPEED_OFFSET = 10

RIGHT = rc.CW
LEFT  = rc.CCW 

NOTHING_DETECTED    = 0
INTERSECTION_RETURN = 1
NO_MORE_LINE_RETURN = 2

FORWARD_FOR_TURN_TIMEOUT = 1 # seconds

# THE FOLLOWING DEFINITIONS WILL HELP US KEEP TRACK OF THE DIFFERENT MODES WE HAVE IN THE LINE FOLLOWING
DEFAULT_LINE_FOLLOWING      = 0     # THIS MODE IS FOR WHEN WE JUST WANT TO FOLLOW THE LINE
NO_DETECTION_LINE_FOLLOWING = 1     # THIS MODE IS FOR WHEN WE WANT TO FOLLOW THE LINE BUT WITHOUT ANY OF THE INTERSECTION DETECTION 
                                    # THIS IS PARTICULARILY USEFUL FOR OUR TURN FUNCTION

# THE FOLLOWING DEFINITIONS WILL HELP KEEP TRACK OF THE CODES RETURNED FROM THE FUNCTION
# THAT TRIES TO DETECTS FACES.
NO_FACE_DETECTED      = 0
FACE_DETECTED_LEFT    = 1
FACE_DETECTED_FORWARD = 2
FACE_DETECTED_RIGHT   = 3
LOOKING_TIMEOUT       = 3   # Seconds

# THE FOLLOWING WILL HELP IN KEEPING THE CURRENT DIRECTION OF THE ROBOT
NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3
# ITS INITIAL DIRECTION WILL ALWAYS BE ASSUMED AS NORTH! NO MATTER WHAT, IN OUR SETUP!!!
CUR_DIR = NORTH 

# THE FOLLOWING WILL HELP IN KEEPING THE STATE OF THE STATE MACHINE
IDLE             = 0
FOLLOW_LINE      = 1
CHANGE_DIRECTION = 2
# THE INITIAL STATE WILL BE IDLE
STATE = IDLE

FAR_LEFT_IR_VAL  = 0
LEFT_IR_VAL      = 0
RIGHT_IR_VAL     = 0
FAR_RIGHT_IR_VAL = 0

#===================================#
# HELPER FUNCTIONS TO READ IR
#===================================#
def readIR():
    global FAR_LEFT_IR_VAL, LEFT_IR_VAL, RIGHT_IR_VAL, FAR_RIGHT_IR_VAL
    FAR_LEFT_IR_VAL  = rc.readFarLeftIR()
    LEFT_IR_VAL      = rc.readLeftIR()
    RIGHT_IR_VAL     = rc.readRightIR()
    FAR_RIGHT_IR_VAL = rc.readFarRightIR()

#==========================================#
# HELPER FUNCTIONS TO FOLLOW LINE
#==========================================#
def followLine(MODE):
    readIR()
    
    # IN THIS MODE, WE WILL DETECT IF WE RUN OUT OF TAPE TO FOLLOW, MISS THE LINE, OR GET TO AN INTERSECTION
    if (MODE==DEFAULT_LINE_FOLLOWING):
        # IF BOTH OF THE MIDDLE SENSORS DETECT NO TAPE, WE MUST HAVE REACHED THE END OF A LINE
        # OR ESCAPED THE LINE
        if(LEFT_IR_VAL == NO_TAPE_DETECTED) and (RIGHT_IR_VAL == NO_TAPE_DETECTED):
            rc.stop()                       # THUS, STOP THE MOTOR
            return NO_MORE_LINE_RETURN      # AND RETURN WITH THE CODE FOR NO MORE LINE 

        # IF TAPE IS DETECTED ON THE FAR LEFT OR FAR RIGHT IR SENSORS THEN WE'VE REACHED AN INTERSECTION
        elif (FAR_LEFT_IR_VAL == TAPE_DETECTED) or (FAR_RIGHT_IR_VAL == TAPE_DETECTED):
            rc.stop()                     # THUS, STOP THE ROBOT
            return INTERSECTION_RETURN    # AND RETURN WITH THE CODE FOR INTERSECTION

        # HOWEVER, IF WE'RE NOT AT AN INTERSECTION YET AND THERE'S A LINE STILL, KEEP TRYING TO FOLLOW THE LINE
        else:
            # IF BOTH MIDDLE SENSORS ARE DETECTING THE TAPE, KEEP MOVING FORWARD
            # WITH BOTH MOTORS GOING AT THE SAME SPEED
            if(LEFT_IR_VAL == TAPE_DETECTED) and (RIGHT_IR_VAL == TAPE_DETECTED):
                rc.moveForward(FORWARD_SPEED)

            # ELSE IF ONLY THE LEFT IR IS NOT DETECTING A LINE
            elif LEFT_IR_VAL == NO_TAPE_DETECTED:
                # SPEED UP THE LEFT SIDE, AND SLOW DOWN THE RIGHT SIDE
                rc.moveLeftForward(FORWARD_SPEED+NO_TAPE_OFFSET)
                rc.moveRightForward(FORWARD_SPEED-NO_TAPE_OFFSET)

            # ELSE IF ONLY THE RIGHT IR IS NOT DETECTING A LINE
            elif RIGHT_IR_VAL == NO_TAPE_DETECTED:
                # SPEED UP THE RIGHt SIDE, AND SLOW DOWN THE LEFT SIDE
                rc.moveLeftForward(FORWARD_SPEED-NO_TAPE_OFFSET)
                rc.moveRightForward(FORWARD_SPEED+NO_TAPE_OFFSET)
    

    # IN THIS MODE, WE WILL ONLY FOLLOW THE LINE IF WE DETECT THE TAPE, OTHERWISE THE MOTORS WILL JUST MOVE FORWARD
    elif(MODE==NO_DETECTION_LINE_FOLLOWING):
        # IF NO TAPE IS DETECTED BY THE MIDDLE SENSORS JUST MOVE FORWARD
        if(LEFT_IR_VAL == NO_TAPE_DETECTED) and (RIGHT_IR_VAL == NO_TAPE_DETECTED):
            rc.moveForward(FORWARD_SPEED)   # KEEP MOVING FORWARD WITH BOTH MOTORS GOING AT THE SAME SPEED

        # ELSE IF BOTH MIDDLE SENSORS ARE DETECTING THE TAPE JUST MOVE FORWARD
        elif(LEFT_IR_VAL == TAPE_DETECTED) and (RIGHT_IR_VAL == TAPE_DETECTED):
            rc.moveForward(FORWARD_SPEED)   # KEEP MOVING FORWARD WITH BOTH MOTORS GOING AT THE SAME SPEED

        # ELSE IF ONLY THE LEFT IR IS NOT DETECTING A LINE
        elif LEFT_IR_VAL == NO_TAPE_DETECTED:
            # SPEED UP THE LEFT SIDE, AND SLOW DOWN THE RIGHT SIDE
            rc.moveLeftForward(FORWARD_SPEED+NO_TAPE_OFFSET)
            rc.moveRightForward(FORWARD_SPEED-NO_TAPE_OFFSET)

        # ELSE IF ONLY THE RIGHT IR IS NOT DETECTING A LINE
        elif RIGHT_IR_VAL == NO_TAPE_DETECTED:
            # SPEED UP THE RIGHt SIDE, AND SLOW DOWN THE LEFT SIDE
            rc.moveLeftForward(FORWARD_SPEED-NO_TAPE_OFFSET)
            rc.moveRightForward(FORWARD_SPEED+NO_TAPE_OFFSET)

    return NOTHING_DETECTED

#==========================================#
# HELPER FUNCTIONS TO TURN AT INTERSECTIONS
#==========================================#
def turn(dir):
    start = time.time()
    end   = start
    # WE FIRST NEED TO MOVE THE ROBOT FORWARD FOR A SET AMOUNT OF TIME WITH LINE FOLLOWING BUT NO DETECTION
    while((end-start)<FORWARD_FOR_TURN_TIMEOUT):
        followLine(NO_DETECTION_LINE_FOLLOWING)
        end = time.time()
    
    # STOP ROBOT
    rc.stop()
    
    # ONCE WE'VE ADVANCED FORWARD ENOUGH, LET'S ROTATE THE ROBOT TILL WE HIT ONE OF OUR FAR OUT SENSORS
    # THEN SLOWLY ROTATE IT, TILL BOTH INSIDE IR EMMITERS DETECT THE TAPE
    if (dir == rc.CW):
        # ROTATE TILL FAR RIGHT IR SENSOR DETECTS LINE SINCE WE'RE TURNING CW
        while(rc.readFarRightIR() == NO_TAPE_DETECTED):
            rc.rotate(rc.CW,ROTATE_SPEED)
        
        # ROTATE SLOWER TILL BOTH OF INSIDE SENSORS DETECT THE LINE
        while((rc.readLeftIR() == NO_TAPE_DETECTED) or (rc.readRightIR() == NO_TAPE_DETECTED)):
            rc.rotate(rc.CW,ROTATE_SPEED-ROTATE_SPEED_OFFSET)

        # THEN STOP THE ROBOT
        rc.stop()

    elif (dir == rc.CCW):
        # ROTATE TILL FAR LEFT IR SENSOR DETECTS LINE SINCE WE'RE TURNING CCW
        while(rc.readFarLeftIR() == NO_TAPE_DETECTED):
            rc.rotate(rc.CCW,ROTATE_SPEED)
        
        # ROTATE SLOWER TILL BOTH OF INSIDE SENSORS DETECT THE LINE
        while((rc.readLeftIR() == NO_TAPE_DETECTED) or (rc.readRightIR() == NO_TAPE_DETECTED)):
            rc.rotate(rc.CCW,ROTATE_SPEED-ROTATE_SPEED_OFFSET)

        # THEN STOP THE ROBOT
        rc.stop()
    
#====================================================#
# HELPER FUNCTIONS TO SEE IF CAMERA CAN DETECT A FACE
#====================================================#
def detectFace(angle):
    rc.setCameraHorzServo(angle)                       # SET THE CAMERA TO LOOK AT THE LEFT SIDE
    time.sleep(0.01)                                   # LET'S GIVE IT 10MS TO GET THERE, JUST IN CASE IT'S NOT THERE
    
    start_time = time.time()
    # TRY TO SEE IF WE CAN DETECT A FACE FOR A CERTAIN TIME LIMIT
    while ((time.time()-start_time)<LOOKING_TIMEOUT):
        _,frame = cap.read()                                # READ FRAME
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)       # CONVERT FRAME TO GRAY SCALE
        faces = face_cascade.detectMultiScale(gray,1.1,4)   # DETECT FACES

        # IF A FACE IS DETECTED, RETURN CODE 1
        if len(faces) != 0:
            return 1    

    # IF NO FACE WAS DETECTED, RETURN CODE 0
    return 0    

def lookForFace():
    # LET'S SEE IF WE CAN DETECT A FACE IN ANY OF THE DIRECTIONS
    # FIRST LET'S LOOK FORWARD
    ret = detectFace(90)
    if ret == 1:
        return FACE_DETECTED_FORWARD

    # THEN THE LEFT SIDE
    ret = detectFace(180)
    if ret == 1:
        return FACE_DETECTED_LEFT

    # THEN THE RIGHT SIDE
    ret = detectFace(0)
    if ret == 1:
        return FACE_DETECTED_RIGHT

    # IF NO FACE WAS DETECTED JUST RETURN WITH THE NO_FACE_DETECTED CODE
    return NO_FACE_DETECTED


def main():
    global CUR_DIR, STATE
    
    # INITIALIZE VARIABLES
    CUR_DIR = NORTH
    STATE   = 0

    # INITIALIZE ROBOT
    rc.init()

    while(True):
        #===================================#
        # IDLE STATE: DO NOTHING UNTIL START
        #             BUTTON IS PRESSED
        #===================================#
        if(STATE == IDLE):
            rc.stop()
            # IT'LL WAIT FOR THE START BUTTON TO BE PRESSED
            # readButton() RETURNS HIGH WHEN NOT PRESSED
            while(rc.readButton()):
                pass
            
            # ONCE PRESSED, WE'LL CHANGE TO THE LINE FOLLOWING STATE
            STATE = FOLLOW_LINE
            # THEN WAIT FOR A LITTLE FOR THE PERSON THAT STARTS IT TO BACK UP
            time.sleep(START_WAITING_TIME)
        
        #===================================#
        # LINE FOLLOWING STATE
        #===================================#
        if(STATE == FOLLOW_LINE):
            ret = followLine(DEFAULT_LINE_FOLLOWING)

            # IF WE DETECTED AN INTERSECTION
            if ret == INTERSECTION_RETURN:
                # CHECK IF THE INTERSECTION HAS A RIGHT TURN
                if (FAR_RIGHT_IR_VAL == TAPE_DETECTED):
                    turn(RIGHT)
                    CUR_DIR = (CUR_DIR+1)%4
                else:
                    rc.moveForward(50)
                    time.sleep(0.01)
                    rc.stop()

                    
            
            # IF NO LINE WAS DETECTED, WE MUST HAVE LEFT LINE
            elif ret == NO_MORE_LINE_RETURN:
                pass

        #===================================#
        # DIRECTION CHANGING STATE
        #===================================#
        if(STATE == CHANGE_DIRECTION):
            pass
    



if __name__ == "__main__":
    main()