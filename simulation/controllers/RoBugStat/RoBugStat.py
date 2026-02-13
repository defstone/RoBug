"""RoBug controller."""

from math import degrees, pi, sin
from time import sleep
import math
import asyncio
from robug_utils import v3
from robug_constants_stat import constants as c
from robug_robot import robug
from controller import Robot


async def loop_timer(loop_ms):
    await asyncio.sleep(loop_ms*0.001)   
 
async def walk():

    while robot.step(fLoop) != -1:
    
        timer = asyncio.create_task(loop_timer(fLoop))
        
        for ID in range(4):
            j = 2*ID
            k = 2*ID+1 
            delta = (r.lLeg[ID].delta-pi/2) * c._SERVO_SGN[j]
            gamma = (r.lLeg[ID].gamma-pi/2) * c._SERVO_SGN[k]        
            lServo[j].setPosition(delta)
            lServo[k].setPosition(gamma)
           
        # increment loop counter of all legs       
        r.inc_loop_counters()
        
        # pre-caluculate new joint angles for next iteration
        r.calculate_joint_angles(bAbs=False)
        
        # wait for timer completion
        await timer


async def main():
    # dispatch async. walk task
    task1 = asyncio.create_task(walk())
    await task1


if __name__ == "__main__":

    # RoBug PHYSICAL MODEL instance
    robot = Robot()
    # get loop time from world info
    fLoop = int(robot.getBasicTimeStep())
    
    # RoBug BEHAVIORAL controller
    r = robug()
    r.set_loop_counter_resume()
    r.instant_update()
    
    # build virtual servo map
    lServo = []
    lServo.append(robot.getDevice('leg_femur_FL_link_joint'))
    lServo.append(robot.getDevice('leg_tibia_FL_link_joint'))
    lServo.append(robot.getDevice('leg_femur_RL_link_joint'))
    lServo.append(robot.getDevice('leg_tibia_RL_link_joint'))
    lServo.append(robot.getDevice('leg_femur_FR_link_joint'))
    lServo.append(robot.getDevice('leg_tibia_FR_link_joint'))
    lServo.append(robot.getDevice('leg_femur_RR_link_joint'))
    lServo.append(robot.getDevice('leg_tibia_RR_link_joint'))

    # start async. walk task
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Controller stopped.")
    

