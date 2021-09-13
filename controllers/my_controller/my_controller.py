"""my_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math


def get_angle(ir_value, p):
    return (ir_value - 15)*p

def run_robot(robot):
    time_step = 16
    max_speed = 1.6
    basespeed = 6.28
    dark = 20
    bright = 50
    # 累计误差 I
    iHistory = [0, 0, 0]
    # D
    lastError = 0
    Kp = 0.025
    Ki = 0.012
    Kd = 0.01
    
    # Motor
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # enable sensor
    irL4 = robot.getDevice('irL4')
    irL3 = robot.getDevice('irL3')
    irL2 = robot.getDevice('irL2')
    irL1 = robot.getDevice('irL1')
    irR1 = robot.getDevice('irR1')
    irR2 = robot.getDevice('irR2')
    irR3 = robot.getDevice('irR3')
    irR4 = robot.getDevice('irR4')
    # 设置扫描频率
    irL4.enable(time_step)
    irL3.enable(time_step)
    irL2.enable(time_step)
    irL1.enable(time_step)
    irR1.enable(time_step)
    irR2.enable(time_step)
    irR3.enable(time_step)
    irR4.enable(time_step)
    
    # setp simulation
    while robot.step(time_step) != -1:
    
        # read ir sensor
        irL4_value = irL4.getValue() * math.pow(1-0.0165, 0)
        irL3_value = irL3.getValue() * math.pow(1-0.0165, 1)
        irL2_value = irL2.getValue() * math.pow(1-0.0165, 2)
        irL1_value = irL1.getValue() * math.pow(1-0.0165, 3)
        irR1_value = irR1.getValue() * math.pow(1-0.0165, 5)
        irR2_value = irR2.getValue() * math.pow(1-0.0165, 6)
        irR3_value = irR3.getValue() * math.pow(1-0.0165, 7)
        irR4_value = irR4.getValue() * math.pow(1-0.0165, 8)
        
        # 有多少个传感器压在线上
        ir_online_amount = 0    
        angle = 0
        if (dark < irL4_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irL4_value, -4)
        if (dark < irL3_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irL3_value, -3)
        if (dark < irL2_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irL2_value, -2)
        if (dark < irL1_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irL1_value, -1)
        if (dark < irR1_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irR1_value, 1)
        if (dark < irR2_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irR2_value, 2)
        if (dark < irR3_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irR3_value, 3)
        if (dark < irR4_value < bright):
            ir_online_amount = ir_online_amount+1
            angle = angle + get_angle(irR4_value, 4)
            
        # 计算方向值
        if ir_online_amount == 0:
            angle = 0
        else:
            angle = angle/ir_online_amount
        
        
            
        print("L4: {:0.2f} L3: {:0.2f} L2: {:0.2f} L1: {:0.2f} | R1: {:0.2f} R2: {:0.2f} R3: {:0.2f} R4: {:0.2f} amount: {} angle: {:0.2f}"
        .format(irL4_value, irL3_value, irL2_value, irL1_value,
        irR1_value, irR2_value, irR3_value, irR4_value,
        ir_online_amount, angle))
        
        error = angle
        # P
        P = error
        # I
        I = 0
        for index in range(len(iHistory)-1):
            iHistory[index] = iHistory[index + 1]
            I = I + iHistory[index]
        iHistory[len(iHistory) - 1] = error
        D = I + error
        # D
        d = error - lastError
        lastError = error
        
        motorspeed = P*Kp + I*Ki + D*Kd
        basespeed = 6.28 - abs(motorspeed) * 0.5


        left_speed = min(basespeed + motorspeed, 6.28)
        right_speed = min(basespeed - motorspeed, 6.28)

        print("P: {:0.2f} I: {:0.2f} D: {:0.2f} S: {:0.2f}".format(
        P, I, D, motorspeed))
        
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
if __name__ == '__main__':
    # create the Robot instance.
    my_robot = Robot()
    run_robot(my_robot)
    
