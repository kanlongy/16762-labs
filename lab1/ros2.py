# ros2 launch stretch_core stretch_driver.launch.py

import numpy as np
import hello_helpers.hello_misc as hm


class StretchApiDemo(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)

    def main(self):
        hm.HelloNode.main(
            self,
            node_name='stretch_api_demo',
            node_topic_namespace='stretch_api_demo',
            wait_for_first_pointcloud=False
        )

        #1 Stow
        self.stow_the_robot()

        #2 Extend arm & raise lift 
        self.move_to_pose(
            {
                'joint_arm': 0.5,
                'joint_lift': 1.1,
            },
            blocking=True,
            duration=3.0
        )

        #3 Move wrist motors
        self.move_to_pose({'joint_wrist_yaw': 0.5}, blocking=True, duration=2.0)
        self.move_to_pose({'joint_wrist_pitch': -0.5}, blocking=True, duration=2.0)
        self.move_to_pose({'joint_wrist_roll': 1.0}, blocking=True, duration=2.0)

        #4 Open and close gripper
        OPEN = 0.1  
        CLOSE = 0.0

        self.move_to_pose({'gripper_aperture': OPEN}, blocking=True, duration=1.5)
        self.move_to_pose({'gripper_aperture': CLOSE}, blocking=True, duration=1.5)

        #5 Rotate head motors
        self.move_to_pose(
            {
                'joint_head_pan': np.radians(45.0),
                'joint_head_tilt': np.radians(45.0),
            },
            blocking=True,
            duration=2.5
        )

        #6 Reset back to stow 
        self.stow_the_robot()

        #7 base movement
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=True, duration=4.0)        
        self.move_to_pose({'rotate_mobile_base': np.radians(180.0)}, blocking=True, duration=6.0)
        self.move_to_pose({'translate_mobile_base': 0.5}, blocking=True, duration=4.0)

        # Stop
        self.stop_the_robot()


if __name__ == '__main__':
    node = StretchApiDemo()
    node.main()
