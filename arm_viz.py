import pygame
from math import sin, cos, radians
import process_wpilib_logs

BASE_ARM_LENGTH = 200
WIDTH = 800
HEIGHT = 800

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

# search for where the setpoint changes (where things actually start to happen)
for idx, setpoint in enumerate(process_wpilib_logs.liftSetpointLog[1]):
    if setpoint != 0:
        index = idx
        break

# main loop
while True:
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            raise SystemExit

    #TODO

    length_setpoint_ = BASE_ARM_LENGTH + process_wpilib_logs.extendSetpointLog[1][index]
    arm_length_ = BASE_ARM_LENGTH + process_wpilib_logs.extendEncLog[1][index]

    # draw the setpoint
    setpoint_x = sin(radians(process_wpilib_logs.liftSetpointLog[1][index])) * length_setpoint_  +  WIDTH//2
    setpoint_y = cos(radians(process_wpilib_logs.liftSetpointLog[1][index])) * length_setpoint_  +  HEIGHT//2
    pygame.draw.line(screen, LIGHT_BLUE, (WIDTH//2, HEIGHT//2), (setpoint_x, setpoint_y), 20)

    # draw the current arm tilt from the middle of the window with a length of BASE_ARM_LENGTH, and a width of 10
    x = sin(radians(process_wpilib_logs.liftEncLog[1][index])) * arm_length_  +  WIDTH//2
    y = cos(radians(process_wpilib_logs.liftEncLog[1][index])) * arm_length_  +  HEIGHT//2
    pygame.draw.line(screen, BLUE, (WIDTH//2, HEIGHT//2), (x, y), 15)
    
    # advance the time once every 2 frames
    frame += 1
    if frame % 1 == 0:
        index += 1
    pygame.display.flip()
    clock.tick(60)

    # if process_wpilib_logs.liftEncLog[0][index] > 200:
        # raise SystemExit