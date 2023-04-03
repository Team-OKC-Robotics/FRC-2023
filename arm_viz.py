import pygame, time
from math import sin, cos, radians
import process_wpilib_logs

BASE_ARM_LENGTH = 200
WIDTH = 800
HEIGHT = 800
HEIGHT_OFFSET = 0
WIDTH_OFFSET = 150
WHEEL_LENGTH = 100

# state variables
arm_length_ = 0
arm_tilt_ = 0
intake_tilt_ = 0

pygame.init()

# R, G, B
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
LIGHT_BLUE = (100, 100, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# create our hardware surface object
screen = pygame.display.set_mode((WIDTH, HEIGHT))

# set the title for the window
pygame.display.set_caption("ArmViz v 0.2")

# go ahead and clear window
screen.fill(WHITE)

# make ourselves a font for displaying text
font = pygame.font.Font(None, 48)

# make ourselves a clock as well
clock = pygame.time.Clock()

print(len(process_wpilib_logs.liftEncLog[1]))

index = 0
frame = 0
advance = False

# search for where the setpoint changes (where things actually start to happen)
for idx, setpoint in enumerate(process_wpilib_logs.liftSetpointLog[1]):
    if setpoint != 0:
        index = idx
        break

# for idx, val in enumerate(process_wpilib_logs.joystickTurnLog[1]):
#     if val != 0:
#         index = idx
#         break

# main loop
while True:
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            raise SystemExit
        elif event.type == pygame.KEYDOWN:
            pass

    length_setpoint_ = BASE_ARM_LENGTH + process_wpilib_logs.extendSetpointLog[1][index]
    arm_length_ = BASE_ARM_LENGTH + process_wpilib_logs.extendEncLog[1][index]

    # draw the setpoint
    setpoint_x = sin(radians(process_wpilib_logs.liftSetpointLog[1][index])) * length_setpoint_  +  WIDTH//2 + WIDTH_OFFSET
    setpoint_y = cos(radians(process_wpilib_logs.liftSetpointLog[1][index])) * length_setpoint_  +  HEIGHT//2 + HEIGHT_OFFSET
    pygame.draw.line(screen, LIGHT_BLUE, (WIDTH//2 + WIDTH_OFFSET, HEIGHT//2 + HEIGHT_OFFSET), (setpoint_x, setpoint_y), 20)

    # draw the current arm tilt from the middle of the window with a length of BASE_ARM_LENGTH, and a width of 10
    x = sin(radians(process_wpilib_logs.liftEncLog[1][index])) * arm_length_  +  WIDTH//2 + WIDTH_OFFSET
    y = cos(radians(process_wpilib_logs.liftEncLog[1][index])) * arm_length_  +  HEIGHT//2 + HEIGHT_OFFSET
    pygame.draw.line(screen, BLUE, (WIDTH//2 + WIDTH_OFFSET, HEIGHT//2 - HEIGHT_OFFSET), (x, y), 15)

    # draw the non-arm swerve drive stuff (for the heck of it, otherwise watching this is too boring)
    #   |\
    # y | \
    #   |  \
    #   |   \
    #   ====== x
    steer_x = sin(radians(process_wpilib_logs.steerEncLog[1][index])) * WHEEL_LENGTH +  WIDTH//2 - WIDTH_OFFSET
    steer_y = cos(radians(process_wpilib_logs.steerEncLog[1][index])) * WHEEL_LENGTH +  HEIGHT//2 - HEIGHT_OFFSET
    pygame.draw.line(screen, GREEN, (WIDTH//2 - WIDTH_OFFSET, HEIGHT//2 - HEIGHT_OFFSET), (steer_x, steer_y), 15)
    pygame.draw.line(screen, GREEN, (steer_x, steer_y), (WIDTH//2 - WIDTH_OFFSET, HEIGHT//2 - HEIGHT_OFFSET), 15)
    
    # draw the joystick values as well
    # turn is simply a horizontal line back and forth
    pygame.draw.line(screen, GREEN, (WIDTH//2 + WIDTH_OFFSET, HEIGHT//4 - HEIGHT_OFFSET*2), (WIDTH//2 + WIDTH_OFFSET + process_wpilib_logs.joystickTurnLog[1][index] * 25, HEIGHT//4 - HEIGHT_OFFSET*2), 10)

    # and then we'll make the drive+strafe a vector. actually, I might not need to do anything fancy here
    pygame.draw.line(screen, RED, (WIDTH//2 - WIDTH_OFFSET, HEIGHT//4 - HEIGHT_OFFSET*2), (WIDTH//2 - WIDTH_OFFSET + process_wpilib_logs.joystickStrafeLog[1][index] * 100, HEIGHT//4 - HEIGHT_OFFSET*2 + process_wpilib_logs.joystickDriveLog[1][index] * 40), 15)

    pygame.display.flip()

    # calculate time difference
    if index > 1:
        delta_t = process_wpilib_logs.driveOutputLog[0][index] - process_wpilib_logs.driveOutputLog[0][index - 1]
        time.sleep(delta_t)
    index += 1