import robomaster
import time
from robomaster import robot
from robomaster import led


class grip_object:
    def _init_(self, ep_robot):
        self.gripper_ctrl = ep_robot.gripper
        self.robotic_arm_ctrl = ep_robot.robotic_arm
        self.led_ctrl = ep_robot.led
        self.rm_define = robomaster.armor

    def pick_object(self):
        # change color
        self.led_ctrl.set_led(comp='all', r=255, g=0, b=0, effect="on", freq=5)
        
        # open Gripper
        self.gripper_ctrl.open()
        time.sleep(3)
    
        # arm up 60(Y)
        self.robotic_arm_ctrl.move(0, 60).wait_for_completed()
    
        # close Gripper
        time.sleep(3)
        self.gripper_ctrl.close()
        time.sleep(3)

        # change color
        self.led_ctrl.set_led(comp='all', r=255, g=255, b=255, effect="on", freq=5)


    def place_object(self):
        # put down
        self.robotic_arm_ctrl.move(108, 0).wait_for_completed()
        self.robotic_arm_ctrl.move(0, -90).wait_for_completed()
    
        # open gripper
        time.sleep(3)
        self.gripper_ctrl.open()
        time.sleep(3)
    
            #  reposition Gripper
        self.robotic_arm_ctrl.move(-12, 0).wait_for_completed()
        self.robotic_arm_ctrl.move(0, 90).wait_for_completed()
        self.robotic_arm_ctrl.move(-105, 0).wait_for_completed()
    
        time.sleep(3)
        self.gripper_ctrl.close()
        time.sleep(3)
        self.led_ctrl.set_led(comp='all', r=69, g=215, b=255, effect="on", freq=5)


         


    

    



