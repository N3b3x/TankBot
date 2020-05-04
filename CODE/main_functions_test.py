import main as mn
import time

# ENABLE WHEN A SPECIFIC PART IS TO BE TESTED
TEST_IR                         = 1
TEST_DEFAULT_FOLLOW_LINE        = 0
TEST_NO_DETECTION_FOLLOW_LINE   = 0
TEST_FORWARD_LINE_DETECTION     = 0
TEST_TURN                       = 0
TEST_FACE_DETECTION             = 0
TEST_MAIN                       = 0

# TEST TIMES FOR THE ONES THAT REQUIRE IT
IR_TEST_TIME                        = 5     # TEST TIME FOR ALL WHILE LOOPS [s]
FOLLOW_LINE_NO_DETECTION_TEST_TIME  = 5     # TEST TIME FOR NO_DETECTION_LINE_FOLLOWING

#===================================#
# TEST READ IR
#===================================#
if TEST_IR:
    start = time.time()
    while((time.time()-start)<IR_TEST_TIME):
        mn.readIR()                                                                         # READ THE IR SENSORS
        print(mn.FAR_LEFT_IR_VAL, mn.LEFT_IR_VAL, mn.RIGHT_IR_VAL, mn.FAR_RIGHT_IR_VAL)     # PRINT THE VALUES


#===================================#
# TEST DEFAULT LINE FOLLOWING
#===================================#
if TEST_DEFAULT_FOLLOW_LINE:
    ret = mn.followLine(mn.DEFAULT_LINE_FOLLOWING)
    while ret == mn.NOTHING_DETECTED:
        ret = mn.followLine(mn.DEFAULT_LINE_FOLLOWING)

if TEST_NO_DETECTION_FOLLOW_LINE:
    start = time.time()
    while((time.time()-start)<FOLLOW_LINE_NO_DETECTION_TEST_TIME):
        mn.followLine(mn.NO_DETECTION_LINE_FOLLOWING)

#===================================#
# TEST FORWARD LINE DETECTION
#===================================#
if TEST_FORWARD_LINE_DETECTION:
    ret = mn.followLine(mn.DEFAULT_LINE_FOLLOWING)
    # WHILE NOTHING IS DETECTED, KEEP GOING FORWARD
    while ret == mn.NOTHING_DETECTED:
        ret = mn.followLine(mn.DEFAULT_LINE_FOLLOWING)

    # IF AN INTERSECTION IS DETECTED
    if ret == mn.INTERSECTION_DETECTED:
        ret2 = mn.detectForwardLine()
        if ret2 == mn.NOTHING_DETECTED:
            print("NO FORWARD LINE WAS DETECTED")
        elif ret2 == mn.FORWARD_LINE_DETECTED:
            print("FORWARD LINE WAS DETECTED")

#===================================#
# TEST TURNING
#===================================#
if TEST_TURN:
    ret = mn.followLine(mn.DEFAULT_LINE_FOLLOWING)
    # WHILE NOTHING IS DETECTED, KEEP GOING FORWARD
    while ret == mn.NOTHING_DETECTED:
        ret = mn.followLine(mn.DEFAULT_LINE_FOLLOWING)

    # IF AN INTERSECTION IS DETECTED
    if ret == mn.INTERSECTION_DETECTED:
        # IF THERE'S TAPE ON THE RIGHT SIDE
        if (mn.FAR_RIGHT_IR_VAL == mn.TAPE_DETECTED):
            # TURN RIGHT
            mn.turn(mn.rc.CW)
        # IF THERE'S TAPE ON THE LEFT SIDE
        elif (mn.FAR_LEFT_IR_VAL == mn.TAPE_DETECTED):
            # TURN LEFT
            mn.turn(mn.rc.CCW)

    if ret == mn.NO_MORE_LINE_DETECTED:
        # ATTEMPT TO DO A 360 BY JUST ROTATING CW. THIS WILL ALLOW US TO DO
        # A 360 BECAUSE TURN, JUST TURNS THE ROBOT TILL IT DETECTS A LINE OON
        # IT'S SENSORS
        mn.turn(mn.rc.CW)

#===================================#
# TEST FACE DETECTION
#===================================#
if TEST_FACE_DETECTION:
    ret = mn.lookForFace()
    if ret == mn.FACE_DETECTED_FORWARD:
        print("A FACE WAS DETECTED RIGHT IN FRONT OF THE ROBOT: TRY TO GO FORWARD")
    elif ret == mn.FACE_DETECTED_RIGHT:
        print("A FACE WAS DETECTED ON THE RIGHT SIDE OF THE ROBOT: TRY TO GO RIGHT")
    elif ret == mn.FACE_DETECTED_LEFT:
        print("A FACE WAS DETECTED ON THE LEFT SIDE OF THE ROBOT: TRY TO GO LEFT")


#===================================#
# TEST MAIN
#===================================#
if TEST_MAIN:
    mn.main()
