#!/usr/bin/env python3
# Copyright 2021 Alexander Barbie
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from typing import Union
from picarx.drivers.dcmotor import AbstractDCMotorDriver
from picarx.interfaces.actuators import MotorSide, TravelDirection
from std_msgs.msg import Int8
from picarx_msgs.msg import Test
from picarx.gpio import GPIO
import argparse

class AckermannDrive(object):

    def __init__(self) -> None:
        self.command_subscriber = None
        self.motor_left_publisher = None
        self.motor_right_publisher = None
        self.clutchgeer_publisher = None

    @property
    def command_subscriber(self):
        return self.__command_subscriber

    @command_subscriber.setter
    def command_subscriber(self, subscriber: rospy.Subscriber):
        if subscriber is None:
            self.__command_subscriber = None
        elif isinstance(subscriber, rospy.Subscriber):
            self.__command_subscriber = subscriber
        else:
            raise ValueError("Subscriber has to be of type rospy.Subscriber or None, but {} was given.".format(type(subscriber)))

    @property
    def motor_left_publisher(self):
        return self.__motor_left_publisher

    @motor_left_publisher.setter
    def motor_left_publisher(self, publisher: rospy.Publisher):
        if publisher is None:
            self.__motor_left_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_left_publisher = publisher
        else:
            raise ValueError("Subscriber has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def motor_right_publisher(self):
        return self.__motor_right_publisher

    @motor_right_publisher.setter
    def motor_right_publisher(self, publisher: rospy.Publisher):
        if publisher is None:
            self.__motor_right_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__motor_right_publisher = publisher
        else:
            raise ValueError("Subscriber has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    @property
    def clutchgeer_publisher(self):
        return self.__clutchgeer_publisher

    @clutchgeer_publisher.setter
    def clutchgeer_publisher(self, publisher: rospy.Publisher):
        if publisher is None:
            self.__clutchgeer_publisher = None
        elif isinstance(publisher, rospy.Publisher):
            self.__clutchgeer_publisher = publisher
        else:
            raise ValueError("Subscriber has to be of type rospy.Publisher or None, but {} was given.".format(type(publisher)))

    def drive(self, ros_msg):
        self.motor_left_publisher.publish(Int8(ros_msg.speed))
        self.motor_right_publisher.publish(Int8(ros_msg.speed))
        self.clutchgeer_publisher.publish(Int8(ros_msg.angle))

    def start(self):
        rospy.init_node("Ackermann driving Skill", anonymous=False)
        rospy.on_shutdown(self.stop)
        self.command_subscriber = rospy.Subscriber(rospy.get_param('~command_topic'), Test, self.drive)
        self.motor_left_publisher = rospy.Publisher(rospy.get_param('~motor_left_topic'), Int8, queue_size=5)
        self.motor_right_publisher = rospy.Publisher(rospy.get_param('~motor_right_topic'), Int8, queue_size=5)
        self.clutchgeer_publisher = rospy.Publisher(rospy.get_param('~steering_topic'), Int8, queue_size=5)
        
        rospy.spin()

    def stop(self):
        rospy.logerr("SHUTTING DOWN")

if __name__ == "__main__":
    driving_skill = AckermannDrive()
    driving_skill.start()