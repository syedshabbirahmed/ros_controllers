#! /usr/bin/env python3

import rospy
from controller import BaseController

def main():

    rospy.init_node('base_controller')

    base_controller = BaseController()

    freq = rospy.get_param('/base_controller/update_frequency')
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        
        if base_controller._received_first_rmsg:

            base_controller.take_action()

            base_controller.potential_field_control()

        rate.sleep()

if __name__ == '__main__':
    main()