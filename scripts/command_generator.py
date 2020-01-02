#!/usr/bin/env python
# -*- coding: utf-8 -*-
# code for python2
import serial
import time
import rospy
from std_msgs.msg import Int16
from kondo_b3mservo_rosdriver.msg import Multi_servo_command
from kondo_b3mservo_rosdriver.msg import Multi_servo_info

# global parameters for general information
initial_process_flag = True
the_number_of_servo = 0

# global parameters for servo info
servo_angle = [0] * 2
servo_current = [0] * 2

# global parameters for servo command
target_position = []
target_velocity = []
target_torque = []
target_position_by_torque = []


# callback function called when the number of servos under control is informed
def callback_init(number):
    global the_number_of_servo
    global servo_angle, servo_current
    the_number_of_servo = number.data  # get data

    # prepare paramters for servo command
    for i in range(the_number_of_servo):
        target_position.append(0)
        target_velocity.append(0)
        target_torque.append(0)
        target_position_by_torque.append(0)


def manipulator_initialization():
    global the_number_of_servo
    multi_servo_init = Multi_servo_command()

    # generate a command to initialize servo angle
    multi_servo_init.control_mode = [0, 0]
    multi_servo_init.target_position = [0, 0]
    multi_servo_init.target_torque = [0, 0]
    multi_servo_init.target_position_by_torque = [0, 0]
    # publisht the command
    multi_servo_command_pub.publish(multi_servo_init)

    # wait for a while and inform it
    time.sleep(1)
    rospy.loginfo("position initialized!")

    # change servo control mode to drive mode and publish it
    multi_servo_init.control_mode = [8, 16]
    multi_servo_init.target_torque = [0, 0]
    multi_servo_init.target_position_by_torque = [0, 0]
    multi_servo_command_pub.publish(multi_servo_init)

    # wait for a while and inform it
    time.sleep(1)
    rospy.loginfo("operation started!")


# callback function to get informations of servos
def callback_get_servo_info(multi_servo_info):
    global servo_angle, servo_current
    servo_angle = multi_servo_info.motor_position
    servo_current = multi_servo_info.motor_current


# function to control servos
def control_manipulator(angle, current):
    global the_number_of_servo

    # setup an instance to publish servo command
    multi_servo_command = Multi_servo_command()

    # parameters for motion
    angle_command_slave = 0
    torque_command_slave = 0
    torque_gain = 10
    current_dead_zone = 72

    # modify command value to avoid over current and other harmful motion
    if angle[0] < 0:
        angle_command_slave = 0
    elif angle[0] > 9000:
        angle_command_slave = 9000
    else:
        angle_command_slave = angle[0]

    if current[1] > current_dead_zone:
        torque_command_slave = (current[1] - current_dead_zone) * torque_gain
    if angle[1] < 0 or angle[1] > 9000 or angle[0] < 0 or angle[0] > 9000:
        torque_command_slave = 0

    # store servo command into the instance of ROS message
    multi_servo_command.target_torque = [torque_command_slave, 0]
    multi_servo_command.target_position_by_torque = [0, angle_command_slave]

    # publish ROS message to command servos
    multi_servo_command_pub.publish(multi_servo_command)


if __name__ == '__main__':
    # global servo_angle, servo_current
    rospy.init_node('generate_multi_command')

    rospy.Subscriber('the_number_of_servo', Int16, callback_init, queue_size=1)
    rospy.Subscriber('multi_servo_info', Multi_servo_info, callback_get_servo_info, queue_size=1)
    # rospy.Subscriber('joy', Joy, callback_generate_multi_command, queue_size=1)
    multi_servo_command_pub = rospy.Publisher('multi_servo_command',
                                              Multi_servo_command, queue_size=1, latch=True)

    # at first loop, set manipulator to initial status
    time.sleep(2)
    manipulator_initialization()
    # if initial_process_flag == True:
    #     time.sleep(0.5)
    #     manipulator_initialization()
    #     initial_process_flag = False

    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        try:
            # control servos based on the information of them
            control_manipulator(servo_angle, servo_current)

        except IOError:
            pass

        rate.sleep()
