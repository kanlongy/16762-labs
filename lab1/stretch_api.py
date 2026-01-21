import time
import numpy as np
import stretch_body.robot

robot = stretch_body.robot.Robot()
robot.startup()

#1 stow
robot.stow()
robot.push_command()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()
robot.end_of_arm.wait_until_at_setpoint()
robot.head.wait_until_at_setpoint()
#time.sleep(3.0)

#2 arm & lift extend
robot.arm.move_to(0.5)
robot.lift.move_to(1.1)
robot.push_command()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()

#3 wrist motors one at a time
robot.end_of_arm.move_to('wrist_yaw',0.5)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_pitch', -0.5)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('wrist_roll', 1)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

#4 open and close gripper
robot.end_of_arm.move_to('stretch_gripper',50)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

robot.end_of_arm.move_to('stretch_gripper',-50)
robot.push_command()
robot.end_of_arm.wait_until_at_setpoint()

#5 move both head motors
robot.head.move_by('head_pan', np.radians(45)) 
robot.head.move_by('head_tilt', np.radians(45))
robot.push_command()
robot.head.wait_until_at_setpoint()

#6 reset back to stow
robot.stow()
robot.push_command()
robot.arm.wait_until_at_setpoint()
robot.lift.wait_until_at_setpoint()
robot.end_of_arm.wait_until_at_setpoint()
robot.head.wait_until_at_setpoint()

#7 base movement
robot.base.translate_by(0.5)
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.base.rotate_by(np.radians(180))
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.base.translate_by(0.5)
robot.push_command()
robot.base.wait_until_at_setpoint()

robot.stop()