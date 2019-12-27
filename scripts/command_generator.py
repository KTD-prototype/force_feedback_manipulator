#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import serial
import time
import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from kondo_b3mservo_rosdriver.msg import Multi_servo_command


initial_process_flag = 1
initial_setparam_flag = 1
the_number_of_servo = 0
target_position = []
target_velocity = []
target_torque = []
target_position_by_torque = []


# callback function called when the number of servos under control is informed
def callback_init(number):
    global the_number_of_servo
    the_number_of_servo = number.data  # get data

    # prepare paramters for servo command
    for i in range(the_number_of_servo):
        target_position.append(0)
        target_velocity.append(0)
        target_torque.append(0)
        target_position_by_torque.append(0)


def callback_get_servo_info(multi_servo_info):
    global target_position, target_velocity, target_torque, target_position_by_torque
    global the_number_of_servo, initial_process_flag
    multi_servo_command = Multi_servo_command()

    for i in range(num):
        target_position[i] = joy_msg.axes[0] * 32000  # left stick LR
        target_velocity[i] = joy_msg.axes[3] * 32767  # right stick LR
        target_torque[i] = joy_msg.axes[1] * 7000  # left stick FB
        # right sitck FR
        target_position_by_torque[i] = joy_msg.axes[0] * 32000
        if target_position_by_torque[i] > 16000:
            target_position_by_torque[i] = 16000
        elif target_position_by_torque[i] < -16000:
            target_position_by_torque[i] = -16000

        multi_servo_command.target_position.append(target_position[i])
        multi_servo_command.target_velocity.append(target_velocity[i])
        multi_servo_command.target_torque.append(target_torque[i])
        multi_servo_command.target_position_by_torque.append(
            target_position_by_torque[i])
    pub.publish(multi_servo_command)
    del multi_servo_command


def control_manipulator():
    aa
    aA


if __name__ == '__main__':
    rospy.init_node('generate_multi_command')

    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)
    rospy.Subscriber('multi_servo_info', Multi_servo_info, callback_get_servo_info, queue_size=1)
    # rospy.Subscriber('joy', Joy, callback_generate_multi_command, queue_size=1)
    pub = rospy.Publisher('multi_servo_command',
                          Multi_servo_command, queue_size=1)

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            control_manipulator()

        except IOError:
            pass

        rate.sleep()
