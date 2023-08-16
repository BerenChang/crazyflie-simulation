#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2022 Bitcraze

# @file crazyflie_controllers_py.py
# Controls the crazyflie motors in webots in Python

"""crazyflie_controller_py controller."""


from controller import Robot
from controller import Motor
from controller import InertialUnit
from controller import GPS
from controller import Gyro
from controller import Keyboard
from controller import Camera
from controller import DistanceSensor

from math import cos, sin, degrees, radians

import numpy as np

import sys
# Change this path to your crazyflie-firmware folder
sys.path.append('../../../../crazyflie-firmware/build')
import cffirmware

robot = Robot()

timestep = int(robot.getBasicTimeStep())

## Initialize motors
m1_motor = robot.getDevice("m1_motor")
m1_motor.setPosition(float('inf'))
m1_motor.setVelocity(-1)
m2_motor = robot.getDevice("m2_motor")
m2_motor.setPosition(float('inf'))
m2_motor.setVelocity(1)
m3_motor = robot.getDevice("m3_motor")
m3_motor.setPosition(float('inf'))
m3_motor.setVelocity(-1)
m4_motor = robot.getDevice("m4_motor")
m4_motor.setPosition(float('inf'))
m4_motor.setVelocity(1)

## Initialize Sensors
imu = robot.getDevice("inertial_unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
camera = robot.getDevice("camera")
camera.enable(timestep)
range_front = robot.getDevice("range_front")
range_front.enable(timestep)
range_left = robot.getDevice("range_left")
range_left.enable(timestep)
range_back = robot.getDevice("range_back")
range_back.enable(timestep)
range_right = robot.getDevice("range_right")
range_right.enable(timestep)

## Crazyflie parameters
armLength = 0.031  # 0.046
thrustToTorque = 0.6  # 0.005964552

## Get keyboard
keyboard = Keyboard()
keyboard.enable(timestep)

## Initialize variables
pastXGlobal = 0.0
pastYGlobal = 0.0
pastZGlobal = 0.0

past_time = robot.getTime()

ctrl = cffirmware.controllerMellinger_t()
cffirmware.controllerMellingerInit(ctrl)
ctrl.massThrust = 1.0  #  132000
ctrl.mass = 0.027

#  XY Position PID
ctrl.kp_xy = 0.3  # 0.4       # P
ctrl.kd_xy = 0.1  # 0.2       # D
ctrl.ki_xy = 0.0  # 0.05      # I
ctrl.i_range_xy = 0.0 #  2.0

#  Z Position done
ctrl.kp_z = 0.0  #  1.25       # P
ctrl.kd_z = 0.0  #  0.4        # D
ctrl.ki_z = 0.0  #  0.5       # I
ctrl.i_range_z  = 0.0  #  0.4

#  Attitude
ctrl.kR_xy = -0.01  #  70000  # 70000, # P
ctrl.kw_xy = -0.01  #  20000  # 20000, # D
ctrl.ki_m_xy = 0.0 # I
ctrl.i_range_m_xy = 0.0  #  1.0

#  Yaw done
ctrl.kR_z = 0.2 # 60000  # 60000, # P
ctrl.kw_z = -0.1  # 12000  # 12000, # D
ctrl.ki_m_z = 0.0  # 500 # 500, # I
ctrl.i_range_m_z  = 0.0  # 1500 # 1500,

# roll and pitch angular velocity
ctrl.kd_omega_rp = 0.0  # 200 # 200 # D

print('Take off!')

# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time

    ## Get measurements
    qx = imu.getQuaternion()[0]
    qy = imu.getQuaternion()[1]
    qz = imu.getQuaternion()[2]
    qw = imu.getQuaternion()[3]
    # roll = imu.getRollPitchYaw()[0]
    # pitch = imu.getRollPitchYaw()[1]
    # yaw = imu.getRollPitchYaw()[2]
    roll_rate = gyro.getValues()[0]
    pitch_rate = gyro.getValues()[1]
    yaw_rate = gyro.getValues()[2]
    xGlobal = gps.getValues()[0]
    vxGlobal = (xGlobal - pastXGlobal)/dt
    yGlobal = gps.getValues()[1]
    vyGlobal = (yGlobal - pastYGlobal)/dt
    zGlobal = gps.getValues()[2]
    vzGlobal = (zGlobal - pastZGlobal)/dt

    ## Put measurement in state estimate
    # TODO replace these with a EKF python binding
    state = cffirmware.state_t()
    state.attitudeQuaternion.x = qx
    state.attitudeQuaternion.y = qy
    state.attitudeQuaternion.z = qz
    state.attitudeQuaternion.w = qw
    # state.attitude.roll = degrees(roll)
    # state.attitude.pitch = -degrees(pitch)
    # state.attitude.yaw = degrees(yaw)
    state.position.x = xGlobal
    state.position.y = yGlobal
    state.position.z = zGlobal
    state.velocity.x = vxGlobal
    state.velocity.y = vyGlobal
    state.velocity.z = vzGlobal
    
    # state.attitude.pitch = -state.attitude.pitch

    # Put gyro in sensor data
    sensors = cffirmware.sensorData_t()
    sensors.gyro.x = degrees(roll_rate)
    sensors.gyro.y = degrees(pitch_rate)
    sensors.gyro.z = degrees(yaw_rate)

    # keyboard input
    forwardDesired = 0.0
    sidewaysDesired = 0.0
    yawDesired = 0.0
    height_diff_desired = 0.0

    key = keyboard.getKey()
    while key>0:
        if key == Keyboard.UP:
            forwardDesired = 0.5
        elif key == Keyboard.DOWN:
            forwardDesired = -0.5
        elif key == Keyboard.RIGHT:
            sidewaysDesired = -0.5
        elif key == Keyboard.LEFT:
            sidewaysDesired = 0.5
        elif key == ord('Q'):
            yawDesired = 8.0
        elif key == ord('E'):
            yawDesired = -8.0

        key = keyboard.getKey()

    ## Example how to get sensor data
    # range_front_value = range_front.getValue();
    # cameraData = camera.getImage()

    ## Fill in Setpoints
    setpoint = cffirmware.setpoint_t()
    
    setpoint.mode.x = cffirmware.modeAbs
    setpoint.mode.y = cffirmware.modeAbs
    setpoint.mode.z = cffirmware.modeAbs
    setpoint.mode.yaw = cffirmware.modeAbs
    
    setpoint.position.z = 0.3
    setpoint.position.x = forwardDesired
    # setpoint.position.y = 0.0
    
    setpoint.attitudeRate.yaw = degrees(yawDesired)
    setpoint.attitude.yaw = -180.0
    
    setpoint.velocity.x = forwardDesired
    setpoint.velocity.y = sidewaysDesired
    setpoint.velocity_body = True

    ## Firmware PID bindings
    control = cffirmware.control_t()
    tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
    # cffirmware.controllerPid(control, setpoint,sensors,state,tick)
    
    
    cffirmware.controllerMellinger(ctrl, control, setpoint, sensors, state, tick)
    # print("M_x component", eR_x, ew_x, err_d_roll)
    # print("control signal", control.thrust, M_x, M_y, M_z)
    
    print("control signal", control.thrust, control.roll, control.pitch, control.yaw)
        
    ##
    cmd_thrust = control.thrust
    cmd_roll = radians(control.roll)
    cmd_pitch = radians(control.pitch)
    cmd_yaw = -radians(control.yaw)
    
    ## get thrust f1-f4 from thrust roll pitch yaw
    thrust1 = 0.25*cmd_thrust + cmd_pitch / (2*armLength) - cmd_yaw / (4*thrustToTorque)
    thrust2 = 0.25*cmd_thrust - cmd_roll / (2*armLength) + cmd_yaw / (4*thrustToTorque)
    thrust3 = 0.25*cmd_thrust - cmd_pitch / (2*armLength) - cmd_yaw / (4*thrustToTorque)
    thrust4 = 0.25*cmd_thrust + cmd_roll / (2*armLength) + cmd_yaw / (4*thrustToTorque)

    motor1 = (abs(thrust1 / 4e-05))**0.5
    motor2 = (abs(thrust2 / 4e-05))**0.5
    motor3 = (abs(thrust3 / 4e-05))**0.5
    motor4 = (abs(thrust4 / 4e-05))**0.5
    
    # print("motor input", motor1, motor2, motor3, motor4)
    
    m1_motor.setVelocity(-motor1)
    m2_motor.setVelocity(motor2)
    m3_motor.setVelocity(-motor3)
    m4_motor.setVelocity(motor4)
    print(degrees(imu.getRollPitchYaw()[2]))
    print(sensors.gyro.z)
    
    # cmd_roll = 0.0
    # cmd_pitch = 0.0
    # cmd_yaw = 0.0   
    

    ## Motor mixing
    # motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
    # motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
    # motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
    # motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw
    
    # scaling = 1000 ##Todo, remove necessity of this scaling (SI units in firmware) 1000
    # m1_motor.setVelocity(-motorPower_m1/scaling)
    # m2_motor.setVelocity(motorPower_m2/scaling)
    # m3_motor.setVelocity(-motorPower_m3/scaling)
    # m4_motor.setVelocity(motorPower_m4/scaling)

    past_time = robot.getTime()
    pastXGlobal = xGlobal
    pastYGlobal = yGlobal
    pastZGlobal = zGlobal

    pass
