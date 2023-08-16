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
pastXGlobal = 0
pastYGlobal = 0
pastZGlobal = 0

past_time = robot.getTime()

ctrl = cffirmware.controllerMellinger_t()
cffirmware.controllerMellingerInit(ctrl)
ctrl.massThrust = 1.0  #  132000
ctrl.mass = 0.027

#  XY Position PID
ctrl.kp_xy = 0.02  # 0.4       # P
ctrl.kd_xy = 0.01  # 0.2       # D
ctrl.ki_xy = 0.0  # 0.05      # I
ctrl.i_range_xy = 0.0 #  2.0

#  Z Position done
ctrl.kp_z = 0.3  #  1.25       # P
ctrl.kd_z = 0.2  #  0.4        # D
ctrl.ki_z = 0.0  #  0.5       # I
ctrl.i_range_z  = 0.0  #  0.4

#  Attitude
ctrl.kR_xy = 0.01  #  70000  # 70000, # P
ctrl.kw_xy = 0.01  #  20000  # 20000, # D
ctrl.ki_m_xy = 0.0 # I
ctrl.i_range_m_xy = 0.0  #  1.0

#  Yaw done
ctrl.kR_z = 0.05 # 60000  # 60000, # P
ctrl.kw_z = 0.05  # 12000  # 12000, # D
ctrl.ki_m_z = 0.0  # 500 # 500, # I
ctrl.i_range_m_z  = 0.0  # 1500 # 1500,

# roll and pitch angular velocity
ctrl.kd_omega_rp = 0.0  # 200 # 200 # D

prev_omega_roll = 0.0
prev_omega_pitch = 0.0
prev_setpoint_omega_roll = 0.0
prev_setpoint_omega_pitch = 0.0
GRAVITY_MAGNITUDE = 9.81

desired_x = 0

print('Take off!')

# Main loop:
while robot.step(timestep) != -1:

    dt = robot.getTime() - past_time

    ## Get measurements
    
    # roll = imu.getRollPitchYaw()[0]
    # pitch = imu.getRollPitchYaw()[1]
    # yaw = imu.getRollPitchYaw()[2]
    qx = imu.getQuaternion()[0]
    qy = imu.getQuaternion()[1]
    qz = imu.getQuaternion()[2]
    qw = imu.getQuaternion()[3]
    
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
    
    # state.attitude.roll = degrees(roll)
    # state.attitude.pitch = -degrees(pitch)
    # state.attitude.yaw = degrees(yaw)
    state.attitudeQuaternion.x = qx
    state.attitudeQuaternion.y = qy
    state.attitudeQuaternion.z = qz
    state.attitudeQuaternion.w = qw
    
    state.position.x = xGlobal
    state.position.y = yGlobal
    state.position.z = zGlobal
    state.velocity.x = vxGlobal
    state.velocity.y = vyGlobal
    state.velocity.z = vzGlobal

    # Put gyro in sensor data
    sensors = cffirmware.sensorData_t()
    sensors.gyro.x = degrees(roll_rate)
    sensors.gyro.y = degrees(pitch_rate)
    sensors.gyro.z = degrees(yaw_rate)

    # keyboard input
    forwardDesired = 0
    sidewaysDesired = 0
    yawDesired = 0
    height_diff_desired = 0

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
            yawDesired = 8
        elif key == ord('E'):
            yawDesired = -8

        key = keyboard.getKey()

    ## Example how to get sensor data
    # range_front_value = range_front.getValue();
    # cameraData = camera.getImage()

    ## Fill in Setpoints
    setpoint = cffirmware.setpoint_t()
    
    # setpoint.mode.x = cffirmware.modeAbs
    # setpoint.mode.y = cffirmware.modeAbs
    # setpoint.mode.z = cffirmware.modeAbs
    # setpoint.mode.yaw = cffirmware.modeAbs
    
    setpoint.position.z = 1.0
    # desired_x += 0.01*dt
    setpoint.position.x = desired_x
    print(desired_x)
    # setpoint.position.y = 0.0
    
    setpoint.attitudeRate.yaw = degrees(yawDesired)
    setpoint.attitude.yaw = 0.0
    
    setpoint.velocity.x = forwardDesired
    setpoint.velocity.y = sidewaysDesired
    setpoint.velocity_body = True

    ## Firmware PID bindings
    control = cffirmware.control_t()
    tick = 100 #this value makes sure that the position controller and attitude controller are always always initiated
    # cffirmware.controllerPid(control, setpoint,sensors,state,tick)
    
    
    # cffirmware.controllerMellinger(ctrl, control, setpoint, sensors, state, tick)
    ############################################ cffirmware in python
    setpointPos = np.array([setpoint.position.x, setpoint.position.y, setpoint.position.z])
    setpointVel = np.array([setpoint.velocity.x, setpoint.velocity.y, setpoint.velocity.z])
    statePos = np.array([state.position.x, state.position.y, state.position.z])
    stateVel = np.array([state.velocity.x, state.velocity.y, state.velocity.z])
    
    r_error = setpointPos - statePos
    v_error = setpointVel - stateVel
    
    # print(r_error)
    
    # integral error
    i_error_x = 0.0
    i_error_y = 0.0
    i_error_z = 0.0
    
    target_thrust = np.array([0.0, 0.0, 0.0])
    target_thrust[0] = ctrl.mass * setpoint.acceleration.x + ctrl.kp_xy * r_error[0] + ctrl.kd_xy * v_error[0] # + ctrl.ki_xy * i_error_x
    target_thrust[1] = ctrl.mass * setpoint.acceleration.y + ctrl.kp_xy * r_error[1] + ctrl.kd_xy * v_error[1] # + ctrl.ki_xy * i_error_y
    target_thrust[2] = ctrl.mass * (setpoint.acceleration.z + GRAVITY_MAGNITUDE) + ctrl.kp_z * r_error[2] + ctrl.kd_z * v_error[2] # + ctrl.ki_z * i_error_z
    
    desiredYaw = setpoint.attitude.yaw
    
    # R = np.matrix([[1 - 2*qy*qy - 2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
    #                [2*qx*qy + 2*qz*qw, 1 - 2*qx*qx - 2*qz*qz, 2*qy*qz - 2*qx*qw],
    #                [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy]])
    
    z_axis = np.array([2*qx*qz + 2*qy*qw, 2*qy*qz - 2*qx*qw, 1 - 2*qx*qx - 2*qy*qy])
    
    # R = np.matrix([[cos(pitch)*cos(yaw), sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw), cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw)],
    #                [cos(pitch)*sin(yaw), sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw), cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw)],
    #                [-sin(pitch), sin(roll) * cos(pitch), cos(roll)*cos(pitch)]])
    # R_transpose = R.T
    # Rdes_transpose = np.matrix([x_axis_desired, y_axis_desired, z_axis_desired])
    # Rdes = Rdes_transpose.T
    # eRM = np.subtract(np.matmul(Rdes_transpose, R), np.matmul(R_transpose, Rdes))
    # z_axis = np.array([cos(roll) * sin(pitch) * cos(yaw) + sin(roll) * sin(yaw), cos(roll) * sin(pitch) * sin(yaw) - sin(roll) * cos(yaw), cos(roll) * cos(pitch)])
    
    current_thrust = np.dot(target_thrust, z_axis)
    z_axis_desired = target_thrust / np.linalg.norm(target_thrust)
    
    x_c_des = np.array([cos(radians(desiredYaw)), sin(radians(desiredYaw)), 0])

    y_axis_desired = np.cross(z_axis_desired, x_c_des)
    y_axis_desired = y_axis_desired / np.linalg.norm(y_axis_desired)
    x_axis_desired = np.cross(y_axis_desired, z_axis_desired)
    
    eR_x = (-1 + 2*qx**2 + 2*qy**2)*y_axis_desired[2] + z_axis_desired[1] - 2*(qx*y_axis_desired[0]*qz + qy*y_axis_desired[1]*qz - qx*qy*z_axis_desired[0] + qx**2*z_axis_desired[1] + qz**2*z_axis_desired[1] - qy*qz*z_axis_desired[2]) + 2*qw*(-(qy*y_axis_desired[0]) - qz*z_axis_desired[0] + qx*(y_axis_desired[1] + z_axis_desired[2]))
    eR_y = x_axis_desired[2] - z_axis_desired[0] - 2*(qx**2*x_axis_desired[2] + qy*(x_axis_desired[2]*qy - x_axis_desired[1]*qz) - (qy**2 + qz**2)*z_axis_desired[0] + qx*(-(x_axis_desired[0]*qz) + qy*z_axis_desired[1] + qz*z_axis_desired[2]) + qw*(qx*x_axis_desired[1] + qz*z_axis_desired[1] - qy*(x_axis_desired[0] + z_axis_desired[2])));
    eR_y = -eR[1]
    eR_z = y_axis_desired[1] - 2*(qy*(qx*x_axis_desired[0] + qy*y_axis_desired[0] - qx*y_axis_desired[1]) + qw*(qx*x_axis_desired[2] + qy*y_axis_desired[2])) + 2*(-(x_axis_desired[2]*qy) + qw*(x_axis_desired[0] + y_axis_desired[1]) + qx*y_axis_desired[2])*qz - 2*y_axis_desired[0]*qz**2 + x_axis_desired[1]*(-1 + 2*qx**2 + 2*qz**2);
    
    err_d_roll = 0.0
    err_d_pitch = 0.0
    
    stateAttitudeRateRoll = radians(sensors.gyro.x)
    stateAttitudeRatePitch = -radians(sensors.gyro.y)
    stateAttitudeRateYaw = radians(sensors.gyro.z)
    
    ew_x = radians(setpoint.attitudeRate.roll) - stateAttitudeRateRoll
    ew_y = -radians(setpoint.attitudeRate.pitch) - stateAttitudeRatePitch
    ew_z = radians(setpoint.attitudeRate.yaw) - stateAttitudeRateYaw

    # err_d_roll = ((radians(setpoint.attitudeRate.roll) - prev_setpoint_omega_roll) - (stateAttitudeRateRoll - prev_omega_roll)) / dt
    # err_d_pitch = (-(radians(setpoint.attitudeRate.pitch) - prev_setpoint_omega_pitch) - (stateAttitudeRatePitch - prev_omega_pitch)) / dt
   
    prev_omega_roll = stateAttitudeRateRoll
    prev_omega_pitch = stateAttitudeRatePitch
    prev_setpoint_omega_roll = radians(setpoint.attitudeRate.roll)
    prev_setpoint_omega_pitch = radians(setpoint.attitudeRate.pitch)
    
    i_error_m_x = 0
    i_error_m_y = 0
    i_error_m_z = 0
        
    M_x = -ctrl.kR_xy * eR_x + ctrl.kw_xy * ew_x # + ctrl.ki_m_xy * ctrl.i_error_m_x + ctrl.kd_omega_rp * err_d_roll
    M_y = -ctrl.kR_xy * eR_y + ctrl.kw_xy * ew_y # + ctrl.ki_m_xy * ctrl.i_error_m_y + ctrl.kd_omega_rp * err_d_pitch
    M_z = -ctrl.kR_z  * eR_z + ctrl.kw_z  * ew_z # + ctrl.ki_m_z  * ctrl.i_error_m_z
    
    print(" ")
    control.thrust = ctrl.massThrust * current_thrust
    
    ####################################################### cffirmware in python

    # print("M_x component", eR_x, ew_x, err_d_roll)
    print("control signal", control.thrust, M_x, M_y, M_z)
    
    ##
    cmd_thrust = control.thrust
    cmd_roll = radians(M_x)
    cmd_pitch = radians(M_y)
    cmd_yaw = -radians(M_z)
    
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
    
    

    ## Motor mixing
    # motorPower_m1 =  cmd_thrust - cmd_roll + cmd_pitch + cmd_yaw
    # motorPower_m2 =  cmd_thrust - cmd_roll - cmd_pitch - cmd_yaw
    # motorPower_m3 =  cmd_thrust + cmd_roll - cmd_pitch + cmd_yaw
    # motorPower_m4 =  cmd_thrust + cmd_roll + cmd_pitch - cmd_yaw
    
    # scaling = 1 ##Todo, remove necessity of this scaling (SI units in firmware) 1000
    # m1_motor.setVelocity(-motorPower_m1/scaling)
    # m2_motor.setVelocity(motorPower_m2/scaling)
    # m3_motor.setVelocity(-motorPower_m3/scaling)
    # m4_motor.setVelocity(motorPower_m4/scaling)

    past_time = robot.getTime()
    pastXGlobal = xGlobal
    pastYGlobal = yGlobal
    pastZGlobal = zGlobal

    pass
