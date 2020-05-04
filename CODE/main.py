#===================================#
# IMPORT THE NECESSARY LIBRARIES
#===================================#
import cv2
import robot_control as rc
import time

#===================================#
# FOR OPENCV FACE DETECTION
#===================================#
# Load the cascade
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml') # LOAD CASCADE CLASSIFIER
cap          = cv2.VideoCapture(0)                                          # CAMERA CAPTURE FOR THE USB CAM
DEBUG        = 1                                                            # SET THIS TO ONE WHEN WE WANT TO DISPLAY THE CAMERA WHEN IT DETECTS FACES
#===================================#
# VARIABLES
#===================================#
START_WAITING_TIME = 5  # THE AMOUNT OF SECONDS TO WAIT BEFORE STARTING AFTER THE START BUTTON IS PRESSED

# THE FOLLOWING WILL HELP KEEP TRACK OF THE RETURNED VALUE FROM THE IR SENSORS
TAPE_DETECTED    = 0    # VALUE OF IR WHEN TAPE IS BELOW
NO_TAPE_DETECTED = 1    # VALUE OF IR WHEN NO TAPE IS BELOW

# THE FOLLOWING WILL HELP KEEP TRA CK OF THE SPEED WHEN THE ROBOT IS MOVING FORWARD
FORWARD_SPEED    = 60  # THIS IS THE SPEED OF THE WHEELS WHEN THE ROBOT MOVES FORWARD
NO_TAPE_OFFSET   = 20   # WHAT TO OFFSET THE WHEELS SPEED WITH WHEN STARTS TO LEAVE THE TAPE

# THE FOLLOWING WILL HELP KEEP TRACK OF THE SPEED WHEN THE ROBOT IS ROTATING
FORWARD_FOR_TURN_TIMEOUT        = 0.85  # THE AMOUNT OF SECONDS TO MOVE FORWARD RIGHT BEFORE ROTATING [s]
FORWARD_LINE_DETECTION_TIMEOUT  = 0.01  # THE AMOUNT OF SECONDS TO MOVE FORWARD TO SEE IF THE LINE CONTINUES FORWARD AFTER REACHING AN INTERSECTION [s]


ROTATE_SPEED_L                  = 70   # THIS IS THE SPEED OF THE WHEELS WHEN ROBOT ROTATES
ROTATE_SPEED_R                  = 90   # THIS IS THE SPEED OF THE WHEELS WHEN ROBOT ROTATES
ROTATE_RIGHT_OFFSET             = ROTATE_SPEED_R - ROTATE_SPEED_L

ROTATE_SPEED_OFFSET             = 30    # WHAT TO OFFSET THE WHEELS SPEED WITH WHEN WE DETECT A LINE ON THE FAR OUT IR SENSORS

# THE FOLLOWING WILL HELP KEEP TRACK OF THE BIAS AND TURNING DIRECTIONS
NONE    = 0
FORWARD = 1
RIGHT   = 2
LEFT    = 3
BACK    = 4

# THE FOLLOWING WILL HELP KEEP TRACK OF WHAT THE LINE FOLLOWING FUNCTION RETURNS
NOTHING_DETECTED      = 0
INTERSECTION_DETECTED = 1
NO_MORE_LINE_DETECTED = 2
FORWARD_LINE_DETECTED = 3

# THE FOLLOWING DEFINITIONS WILL HELP US KEEP TRACK OF THE DIFFERENT MODES WE HAVE IN THE LINE FOLLOWING
DEFAULT_LINE_FOLLOWING      = 0     # THIS MODE IS FOR WHEN WE JUST WANT TO FOLLOW THE LINE
NO_DETECTION_LINE_FOLLOWING = 1     # THIS MODE IS FOR WHEN WE WANT TO FOLLOW THE LINE BUT WITHOUT ANY OF THE INTERSECTION DETECTION 
                                    # THIS IS PARTICULARILY USEFUL FOR OUR TURN FUNCTION

# THE FOLLOWING DEFINITIONS WILL HELP KEEP TRACK OF THE CODES RETURNED FROM THE FACE DETECTION FUNCTION.
LOOKING_TIMEOUT       = 2   # THE AMOUNT OF SECONDS TO LOOK AT A DIRECTION FOR FACE DETECTION [s]

NO_FACE_DETECTED      = 0
FACE_DETECTED_LEFT    = 1
FACE_DETECTED_FORWARD = 2
FACE_DETECTED_RIGHT   = 3

# THE FOLLOWING WILL HELP IN KEEPING THE CURRENT DIRECTION OF THE ROBOT
NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3

# ITS INITIAL DIRECTION WILL ALWAYS BE ASSUMED AS NORTH! NO MATTER WHAT; IN MY SETUP!!!
CUR_DIR = NORTH 

# THE FOLLOWING WILL HELP KEEP TRACK OF THE STATE MACHINE STATE
IDLE             = 0
FOLLOW_LINE      = 1
CHANGE_DIRECTION = 2
STATE = IDLE            # THE INITIAL STATE WILL BE IDLE

# THE FOLLOWING WILL STORE THE RECENT VALUES THE IR SENSORS
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
            return NO_MORE_LINE_DETECTED    # AND RETURN WITH THE CODE FOR NO MORE LINE 

        # IF TAPE IS DETECTED ON THE FAR LEFT OR FAR RIGHT IR SENSORS THEN WE'VE REACHED AN INTERSECTION
        elif (FAR_LEFT_IR_VAL == TAPE_DETECTED) or (FAR_RIGHT_IR_VAL == TAPE_DETECTED):
            rc.stop()                       # THUS, STOP THE ROBOT
            return INTERSECTION_DETECTED    # AND RETURN WITH THE CODE FOR INTERSECTION

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
        rc.rotate(rc.CW,ROTATE_SPEED_L, ROTATE_RIGHT_OFFSET)
        while(rc.readFarRightIR() == NO_TAPE_DETECTED):
            pass
            
        rc.rotate(rc.CW,ROTATE_SPEED_L-ROTATE_SPEED_OFFSET, ROTATE_RIGHT_OFFSET)
        # ROTATE SLOWER TILL BOTH OF INSIDE SENSORS DETECT THE LINE
        while((rc.readLeftIR() == NO_TAPE_DETECTED) or (rc.readRightIR() == NO_TAPE_DETECTED)):
            pass

        # THEN STOP THE ROBOT
        rc.stop()

    elif (dir == rc.CCW):
        # ROTATE TILL FAR LEFT IR SENSOR DETECTS LINE SINCE WE'RE TURNING CCW
        rc.rotate(rc.CCW,ROTATE_SPEED_L,ROTATE_RIGHT_OFFSET)
        while(rc.readFarLeftIR() == NO_TAPE_DETECTED):
            pass
        
        # ROTATE SLOWER TILL BOTH OF INSIDE SENSORS DETECT THE LINE
        rc.rotate(rc.CCW,ROTATE_SPEED_L-ROTATE_SPEED_OFFSET,ROTATE_RIGHT_OFFSET)
        while((rc.readLeftIR() == NO_TAPE_DETECTED) or (rc.readRightIR() == NO_TAPE_DETECTED)):
            pass
        # THEN STOP THE ROBOT
        rc.stop()
    
#====================================================#
# HELPER FUNCTIONS TO SEE IF CAMERA CAN DETECT A FACE
#====================================================#
def detectFace(angle):
    rc.setCameraVertServo(100)
    rc.setCameraHorzServo(angle)                       # SET THE CAMERA TO LOOK AT THE LEFT SIDE
    time.sleep(0.01)                                   # LET'S GIVE IT 10MS TO GET THERE, JUST IN CASE IT'S NOT THERE
    
    start_time = time.time()
    # TRY TO SEE IF WE CAN DETECT A FACE FOR A CERTAIN TIME LIMIT
    while ((time.time()-start_time)<LOOKING_TIMEOUT):
        _,frame = cap.read()                                                # READ FRAME
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)                       # CONVERT FRAME TO GRAY SCALE
        faces = face_cascade.detectMultiScale(gray,1.3,5,minSize=(190,190)) # DETECT FACES OF MIN SIZE 190x190 PIXELS

        # IF DEBUG IS SET TO TRUE
        if DEBUG:
            # ADD RECTANGLE WHERE THE FACES ARE ON THE IMAGE
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            # THEN SHOW FRAME
            cv2.imshow('FRAME', frame)

        # IF A FACE IS DETECTED, RETURN CODE 1
        if len(faces) != 0:
            return 1    

    # IF DEBUG IS SET TO TRUE
    if DEBUG:
        # DESTROY ALL OPENED WINDOWS FROM OPENCV DEBUGGING WE'VE CREATED
        cv2.destroyAllWindows()
        
    # IF NO FACE WAS DETECTED, RETURN CODE 0
    return 0    

def lookForFace():
    # LET'S SEE IF WE CAN DETECT A FACE IN ANY OF THE DIRECTIONS
    # FIRST LET'S LOOK FORWARD
    ret = detectFace(90)
    if ret == 1:
        print("FACE_DETECTED_FORWARD :)")
        rc.setCameraHorzServo(90)                       # SET THE CAMERA TO LOOK AT THE CENTER
        return FACE_DETECTED_FORWARD

    # THEN THE LEFT SIDE
    ret = detectFace(160)
    if ret == 1:
        print("FACE_DETECTED_LEFT :)")
        rc.setCameraHorzServo(90)                       # SET THE CAMERA TO LOOK AT THE CENTER
        return FACE_DETECTED_LEFT

    # THEN THE RIGHT SIDE
    ret = detectFace(20)
    if ret == 1:
        print("FACE_DETECTED_RIGHT :)")
        rc.setCameraHorzServo(90)                       # SET THE CAMERA TO LOOK AT THE CENTER
        return FACE_DETECTED_RIGHT

    rc.setCameraHorzServo(90)                       # SET THE CAMERA TO LOOK AT THE CENTER
    print("NO_FACE_DETECTED :(")
    # IF NO FACE WAS DETECTED JUST RETURN WITH THE NO_FACE_DETECTED CODE
    return NO_FACE_DETECTED

#====================================================#
# DETECT FORWARD LINE
#====================================================#
def detectForwardLine():
    while((rc.readFarLeftIR() == TAPE_DETECTED) or (rc.readFarRightIR() == TAPE_DETECTED)):
        # SO MOVE WITH SLOW SPEED
        rc.moveForward(50)
    # FOR A TINY AMOUNT OF TIME
    #time.sleep(FORWARD_LINE_DETECTION_TIMEOUT)
    # STOP ROBOT
    rc.stop()

    # READ MID SENSORS. IF EVEN JUST ONE OF THEM RETURN TRUE
    # THEN IT MEANS WE HAVE DETECTED A LINE FORWARD
    if((rc.readLeftIR()==TAPE_DETECTED) or (rc.readRightIR()==TAPE_DETECTED)):
        return FORWARD_LINE_DETECTED

    # IF NO LINE IS READ THEN JUST RETURN FALSE
    return NOTHING_DETECTED

#====================================================#
# HELPER FUNCTIONS TO SEE IF CAMERA CAN DETECT A FACE
#====================================================#
def main():
    global CUR_DIR, STATE
    
    # INITIALIZE VARIABLES
    TURN_DIR = NONE                                 # STORES THE TURNING DIRECTION
    BIAS     = RIGHT                                # STORES THE BIAS DIRECTION WE WANT TO TURN IN
    CUR_DIR  = NORTH                                # STORES THE CURRENT FACING DIRECTION OF THE ROBOT, ALWAYS INITIATES TO NORTH
    STATE    = 0                                    # STORES THE STATE OF THE STATE MACHINE
    LINE_FOLLOWING_MODE = DEFAULT_LINE_FOLLOWING    # STORES THE LINE FOLLOWING MODE WE WANT TO USE

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
            ret = followLine(LINE_FOLLOWING_MODE)

            # IF WE DETECTED AN INTERSECTION
            if ret == INTERSECTION_DETECTED:
                # FIRST THING TO DO IS CHECK IF THE CAMERA SEES A FACE IN ANY OF THE DIRECTIONS
                ret = lookForFace()
                # IF THERE'S ONE DETECTED FOWARD
                if ret == FACE_DETECTED_FORWARD:
                    # LET'S BIAS NO TURN TO OCCUR
                    BIAS = FORWARD
                
                # IF THERE'S ONE DETECTED LEFT
                elif ret == FACE_DETECTED_LEFT:
                    # LET'S BIAS THE ROBOT TO TURN LEFT
                    BIAS = LEFT

                # IF THERE'S ONE DETECTED RIGHT
                elif ret == FACE_DETECTED_RIGHT:
                    # LET'S BIAS THE ROBOT TO TURN RIGH
                    BIAS = RIGHT
                
                # OTHERWISE, WE WILL ALWAYS BIAS TO GO RIGHT
                else:
                    BIAS = RIGHT


                if BIAS == FORWARD:
                    # CHECK IF THERE'S A LINE FORWARD 
                    ret = detectForwardLine()
                    # IF THERE IS A LINE, JUST CONTINUE FOLLOWING THE LINE
                    # SO DON'T CHANGE STATE, !!! THERE IS NO NEED TO DO THE FOLLOWING, 
                    # BECAUSE WE ARE ALREADY IN THE LINE FOLLOWING STATE. I AM JUST 
                    # DOING IT FOR CODE DECIPHERING CLARITY !!!
                    if ret == FORWARD_LINE_DETECTED:
                        STATE = FOLLOW_LINE

                    # IF NOT,
                    else:
                        # CHECK IF THE INTERSECTION HAS A RIGHT TURN
                        # IF IT DOES
                        if (FAR_RIGHT_IR_VAL == TAPE_DETECTED):
                            TURN_DIR = RIGHT            # SET THE TURNING DIRECTION TO RIGHT
                            STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION
                        
                        # OTHERWISE, IF THERE'S NOTHING, THEN IT MEANS THAT THERE
                        # WAS ONLY A LEFT TURN AT THIS INTERSECTION SO
                        # LET'S TURN LEFT
                        elif ret == NOTHING_DETECTED:
                            TURN_DIR = LEFT             # THUS, SET THE TURNING DIRECTION TO LEFT
                            STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION


                if BIAS == RIGHT:
                    # CHECK IF THE INTERSECTION HAS A RIGHT TURN
                    # IF IT DOES
                    if (FAR_RIGHT_IR_VAL == TAPE_DETECTED):
                        TURN_DIR = RIGHT            # SET THE TURNING DIRECTION TO RIGHT
                        STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION
                        
                    # IF NOT,
                    else:
                        # CHECK IF THERE'S A LINE FORWARD 
                        ret = detectForwardLine()
                        # IF THERE IS A LINE, JUST CONTINUE FOLLOWING THE LINE
                        # SO DON'T CHANGE STATE, !!! THERE IS NO NEED TO DO THE FOLLOWING, 
                        # BECAUSE WE ARE ALREADY IN THE LINE FOLLOWING STATE. I AM JUST 
                        # DOING IT FOR CODE DECIPHERING CLARITY !!!
                        if ret == FORWARD_LINE_DETECTED:
                            STATE = FOLLOW_LINE

                        # OTHERWISE, IF THERE'S NOTHING, THEN IT MEANS THAT THERE
                        # WAS ONLY A LEFT TURN AT THIS INTERSECTION SO
                        # LET'S TURN LEFT
                        elif ret == NOTHING_DETECTED:
                            TURN_DIR = LEFT             # THUS, SET THE TURNING DIRECTION TO LEFT
                            STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION

                elif BIAS == LEFT:
                    # CHECK IF THE INTERSECTION HAS A LEFT TURN
                    # IF IT DOES
                    if (FAR_LEFT_IR_VAL == TAPE_DETECTED):
                        TURN_DIR = LEFT             # SET THE TURNING DIRECTION TO LEFT
                        STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION
                        
                    # IF NOT,
                    else:
                        # CHECK IF THERE'S A LINE FORWARD 
                        ret = detectForwardLine()
                        # IF THERE IS A LINE, JUST CONTINUE FOLLOWING THE LINE
                        # SO DON'T CHANGE STATE, !!! THERE IS NO NEED TO DO THE FOLLOWING, 
                        # BECAUSE WE ARE ALREADY IN THE LINE FOLLOWING STATE. I AM JUST 
                        # DOING IT FOR CODE DECIPHERING CLARITY !!!
                        if ret == FORWARD_LINE_DETECTED:
                            STATE = FOLLOW_LINE

                        # OTHERWISE, IF THERE'S NOTHING, THEN IT MEANS THAT THERE
                        # WAS ONLY A RIGHT TURN AT THIS INTERSECTION SO
                        # LET'S TURN RIGHT
                        elif ret == NOTHING_DETECTED:
                            TURN_DIR = RIGHT            # THUS, SET THE TURNING DIRECTION TO LEFT
                            STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION


            # IF NO LINE WAS DETECTED, WE MUST HAVE LEFT THE LINE
            # WITCH POTENTIALLY MEANS THAT WE'RE AT A DEAD END, OR
            # THE LINE FOLLOWING MESSED UP AND LEFT THE LINE!!
            # WE'LL ASSUME THAT THERE WE REACHED A DEAD END FOR NOW
            elif ret == NO_MORE_LINE_DETECTED:
                TURN_DIR = BACK             # IF SO, SET THE TURNING DIRECTION TO BACK
                STATE = CHANGE_DIRECTION    # THEN CHANGE STATE TO CHANGE_DIRECTION
                pass

        #===================================#
        # DIRECTION CHANGING STATE
        #===================================#
        if(STATE == CHANGE_DIRECTION):
            if(TURN_DIR == RIGHT):
                turn(rc.CW)
                CUR_DIR = (CUR_DIR+1)%4

            elif(TURN_DIR == LEFT):
                turn(rc.CCW)
                CUR_DIR = (CUR_DIR-1)%4

            elif(TURN_DIR == BACK):
                turn(RIGHT)
                CUR_DIR = (CUR_DIR+2)%4

            # ONCE, WE'RE DONE CHANGING DIRECTION
            # GO BACK TO LINE FOLLOWING
            STATE = FOLLOW_LINE
    
if __name__ == "__main__":
    main()