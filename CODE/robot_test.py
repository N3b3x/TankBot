import robot_control as rc
import time

rc.init()

#===================================#
# TEST ULTRASONIC SERVO
#===================================#
for angle in range(181):
    rc.setUltraServo(angle)
    time.sleep(0.01)
    
for a in range(90):
    angle = 180-a
    rc.setUltraServo(angle)
    time.sleep(0.01)
    
#===================================#
# TEST HORZ CAMERA SERVO
#===================================#
for angle in range(181):
    rc.setCameraHorzServo(angle)
    time.sleep(0.01)
    
for a in range(90):
    angle = 180-a
    rc.setCameraHorzServo(angle)
    time.sleep(0.01)
    
#===================================#
# TEST VERT CAMERA SERVO
#===================================#
for angle in range(181):
    rc.setCameraVertServo(angle)
    time.sleep(0.01)
    
for a in range(90):
    angle = 180-a
    rc.setCameraVertServo(angle)
    time.sleep(0.01)

#===================================#
# SET CAMERA SERVO ANGLES
#===================================#
for DUTY in range(256):
    rc.setLED(DUTY,DUTY,DUTY)
    time.sleep(0.01)
    
for duty in range(256):
    DUTY = 255-duty
    rc.setLED(DUTY,DUTY,DUTY)
    time.sleep(0.01)