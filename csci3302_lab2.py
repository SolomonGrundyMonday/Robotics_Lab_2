"""csci3302_lab2 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot, Motor, DistanceSensor
import os

# Ground Sensor Measurements under this threshold are black
# measurements above this threshold can be considered white.
# TODO: Fill this in with a reasonable threshold that separates "line detected" from "no line detected"
GROUND_SENSOR_THRESHOLD = 550

# These are your pose vas that you will update by solving the odometry equations
pose_x = 0
pose_y = 0
pose_theta = 0
time_three_sensors = 0

# Index into ground_sensors and ground_sensor_readings for each of the 3 onboard sensors.
LEFT_IDX = 2
CENTER_IDX = 1
RIGHT_IDX = 0

# create the Robot instance.
robot = Robot()

# ePuck Constants
EPUCK_AXLE_DIAMETER = 0.053 # ePuck's wheels are 53mm apart.
EPUCK_MAX_WHEEL_SPEED = 0.1256 # TODO: To be filled in with ePuck wheel speed in m/s
MAX_SPEED = 6.28

# get the time step of the current world.
SIM_TIMESTEP = int(robot.getBasicTimeStep())

# Initialize Motors
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)

# Initialize and Enable the Ground Sensors
gsr = [0, 0, 0]
ground_sensors = [robot.getDevice('gs0'), robot.getDevice('gs1'), robot.getDevice('gs2')]
for gs in ground_sensors:
    gs.enable(SIM_TIMESTEP)

# Allow sensors to properly initialize
for i in range(10): robot.step(SIM_TIMESTEP)

vL = 0 # TODO: Initialize variable for left speed
vR = 0 # TODO: Initialize variable for right speed

# Main Control Loop:
while robot.step(SIM_TIMESTEP) != -1:

    # Read ground sensor values
    for i, gs in enumerate(ground_sensors):
        gsr[i] = gs.getValue()

    #vL = MAX_SPEED
    #vR = MAX_SPEED

    print(gsr) # TODO: Uncomment to see the ground sensor values!

    # Hints:
    #
    # 1) Setting vL=MAX_SPEED and vR=-MAX_SPEED lets the robot turn
    # right on the spot. vL=MAX_SPEED and vR=0.5*MAX_SPEED lets the
    # robot drive a right curve.
    #
    # 2) If your robot "overshoots", turn slower.
    #
    # 3) Only set the wheel speeds once so that you can use the speed
    # that you calculated in your odometry calculation.
    #
    # 4) Disable all console output to simulate the robot superfast
    # and test the robustness of your approach.
    #

    # TODO: Insert Line Following Code Here

    # Center ground sensor
    if((gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD) or time_three_sensors > 0.1):
         vL = MAX_SPEED*0.5
         vR = MAX_SPEED*0.5

    # Left ground sensor
    elif(gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[2] > GROUND_SENSOR_THRESHOLD and gsr[1] > GROUND_SENSOR_THRESHOLD):
         vL = MAX_SPEED*0.1
         vR = MAX_SPEED*0.5
    # Right ground sensor
    elif(gsr[2] < GROUND_SENSOR_THRESHOLD and gsr[0] > GROUND_SENSOR_THRESHOLD and gsr[1] > GROUND_SENSOR_THRESHOLD):
         vL = MAX_SPEED*0.5
         vR = MAX_SPEED*0.1
    else:
        vL = -MAX_SPEED*0.5
        vR = MAX_SPEED*0.5


    # TODO: Call update_odometry Here

    # Hints:
    #
    # 1) Divide vL/vR by MAX_SPEED to normalize, then multiply with
    # the robot's maximum speed in meters per second.
    #
    # 2) SIM_TIMESTEP tells you the elapsed time per step. You need
    # to divide by 1000.0 to convert it to seconds
    #
    # 3) Do simple sanity checks. In the beginning, only one value
    # changes. Once you do a right turn, this value should be constant.
    #
    # 4) Focus on getting things generally right first, then worry
    # about calculating odometry in the world coordinate system of the
    # Webots simulator first (x points down, y points right)

    time = SIM_TIMESTEP/1000.0
    pose_x += (math.cos(pose_theta)*(vL+vR)/2)*time
    pose_y += (math.sin(pose_theta)*(vL+vR)/2)*time
    pose_theta += ((vR-vL)/0.053)*time

    if(pose_theta >= 360):
        pose_theta -= 360


    # TODO: Insert Loop Closure Code Here

    # Hints:
    #
    # 1) Set a flag whenever you encounter the line
    #
    # 2) Use the pose when you encounter the line last
    # for best results

    if(gsr[0] < GROUND_SENSOR_THRESHOLD and gsr[1] < GROUND_SENSOR_THRESHOLD and gsr[2] < GROUND_SENSOR_THRESHOLD):
        time_three_sensors += SIM_TIMESTEP
    else:
        time_three_sensors = 0

    if(time_three_sensors > 0.1):
        pose_x = 0
        pose_y = 0
        pose_theta = 0

    

    print("Current pose: [%5f, %5f, %5f]" % (pose_x, pose_y, pose_theta))
    leftMotor.setVelocity(vL)
    rightMotor.setVelocity(vR)
